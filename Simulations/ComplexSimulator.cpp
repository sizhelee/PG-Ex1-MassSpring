#include "ComplexSimulator.h"

ComplexSimulator::ComplexSimulator()
{
}

const char* ComplexSimulator::getTestCasesStr() {
	return "Test";
}

void ComplexSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void ComplexSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
}

void ComplexSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawPoints();
	drawSprings();

	drawRigidBodies();
}

void ComplexSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;

	m_points.clear();
	m_springs.clear();

	m_bodies.clear();
	m_gravity = false;

	switch (m_iTestCase) {
	case 0: {
		cout << "Test 1\n";
		setMass(10.0f);
		setStiffness(40.0f);
		setDampingFactor(0.0f);

		m_collisions = false;
		m_gravity = false;

		int p0 = addMassPoint(Vec3(0, 0, 0), Vec3(-1, 0, 0), false);
		int p1 = addMassPoint(Vec3(0, 2, 0), Vec3(1, 0, 0), false);

		addSpring(p0, p1, 1);


		int b1 = addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.5, 0.5), 2);
		setOrientationOf(b1, Quat(0, -M_PI / 4, M_PI / 4));
		setVelocityOf(b1, Vec3(1.5, 0, 0));

		int b2 = addRigidBody(Vec3(2, 0, 0), Vec3(1, 0.5, 0.5), 2);
		setOrientationOf(b2, Quat(0, 0, M_PI / 2));
		setVelocityOf(b2, Vec3(-1.5, 0, 0));

	}

	}
}

void ComplexSimulator::externalForcesCalculations(float timeElapsed)
{
}

void ComplexSimulator::simulateTimestep(float timeStep)
{
	//spring system
	midpoint(timeStep, m_gravity, m_collisions);

	//rigid bodies
	// add any externalForces if needed
	addExternalForces();

	// first update the linear stuff
	euler(timeStep);

	// do rotation stuff
	angularCalculations(timeStep);

	// check collisions
	do_collisions();

	//clear forces for next step
	for (RigidBody& b : m_bodies) {
		b.ext_forces.clear();
	}
}

void ComplexSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void ComplexSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

// -------------------- MASS SPRING SYSTEM SIMULATOR --------------------------- //
void ComplexSimulator::setMass(float mass)
{
	m_fMass = mass;
}

void ComplexSimulator::setStiffness(float stiffness)
{
	m_fStiffness = stiffness;
}

void ComplexSimulator::setDampingFactor(float damping)
{
	m_fDamping = damping;
}

void ComplexSimulator::drawPoints() {
	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(1, 0, 0));

	for (const MassPoint& p : m_points) {
		DUC->drawSphere(p.position, m_pointScale);
	}
}

void ComplexSimulator::drawSprings() {
	DUC->beginLine();
	for (const Spring& s : m_springs) {
		DUC->drawLine(m_points[s.point0].position, m_springColor,
			m_points[s.point1].position, m_springColor);
	}
	DUC->endLine();
}

int ComplexSimulator::addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed)
{
	int index = m_points.size();
	m_points.emplace_back(position, Velocity, isFixed);

	return index;
}

void ComplexSimulator::addSpring(int masspoint1, int masspoint2, float initialLength)
{
	m_springs.emplace_back(masspoint1, masspoint2, initialLength);
}

int ComplexSimulator::getNumberOfMassPoints()
{
	return m_points.size();
}

int ComplexSimulator::getNumberOfSprings()
{
	return m_springs.size();
}

Vec3 ComplexSimulator::getPositionOfMassPoint(int index)
{
	return m_points[index].position;
}

Vec3 ComplexSimulator::getVelocityOfMassPoint(int index)
{
	return m_points[index].velocity;
}

void ComplexSimulator::clearForces() {
	for (MassPoint& p : m_points) {
		p.force = Vec3(0, 0, 0);
	}
}

void ComplexSimulator::applyExternalForce(Vec3 force)
{
	for (MassPoint& p : m_points) {
		p.force += m_externalForce;
	}
}

void doCollision(MassPoint& p) {
	if (p.position.y < FLOOR_LIMIT) p.position.y = FLOOR_LIMIT;
	if (p.position.y > CEILING_LIMIT) p.position.y = CEILING_LIMIT;
	if (p.position.x < LEFT_WALL_LIMIT) p.position.x = LEFT_WALL_LIMIT;
	if (p.position.x > RIGHT_WALL_LIMIT) p.position.x = RIGHT_WALL_LIMIT;
	if (p.position.z < CLOSE_WALL_LIMIT) p.position.z = CLOSE_WALL_LIMIT;
	if (p.position.z > FAR_WALL_LIMIT) p.position.z = FAR_WALL_LIMIT;

}

void ComplexSimulator::midpoint(float step, bool gravity, bool collision) {

	// take half an euler step
	std::vector<MassPoint> midpoints = m_points;
	for (MassPoint& mid : midpoints) {
		if (mid.isFixed) continue;
		mid.position += (step / 2) * mid.velocity;

		if (collision)
			doCollision(mid);
	}

	// compute forces at new position
	for (const Spring& s : m_springs) {
		// compute internal force for p0
		Vec3 internal_force = computeInternalForce(midpoints[s.point0], midpoints[s.point1], s.initialLength);
		midpoints[s.point0].force += internal_force;
		// for p1, force in opposite direction
		midpoints[s.point1].force -= internal_force;
	}

	//compute new positions
	for (MassPoint& m : midpoints) {
		if (m.isFixed)
			continue;

		// compute acceleration for m
		Vec3 acc = (m.force - m_fDamping * m.velocity) / m_fMass;

		if (gravity)
			acc += VEC3_GRAVITY;

		m.acceleration = acc;
		//compute next velocity
		m.velocity += (step / 2) * acc;
	}

	//update actual values
	for (int i = 0; i < m_points.size(); i++) {
		if (m_points[i].isFixed) continue;
		m_points[i].position += step * midpoints[i].velocity;
		m_points[i].velocity += step * midpoints[i].acceleration;

		if (collision) {
			doCollision(m_points[i]);
		}
	}

	//clear forces for next step
	clearForces();

}

//---------------------- RIGID BODY SYSTEM SIMULATOR ------------------------ //

int ComplexSimulator::getNumberOfRigidBodies()
{
	return m_bodies.size();
}

Vec3 ComplexSimulator::getPositionOfRigidBody(int i)
{
	RigidBody body = m_bodies[i];

	return body.position;
}

Vec3 ComplexSimulator::getLinearVelocityOfRigidBody(int i)
{
	return m_bodies[i].linear_vel;
}

Vec3 ComplexSimulator::getAngularVelocityOfRigidBody(int i)
{
	return m_bodies[i].angular_vel;
}

void ComplexSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	m_bodies[i].ext_forces.emplace_back(force, loc);
}

int ComplexSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	int index = m_bodies.size();
	m_bodies.emplace_back(position, size, mass);
	return index;
}

void ComplexSimulator::setOrientationOf(int i, Quat orientation)
{
	m_bodies[i].orientation = orientation;
}

void ComplexSimulator::setVelocityOf(int i, Vec3 velocity)
{
	m_bodies[i].linear_vel = velocity;
}



Vec3 ComplexSimulator::computeInternalForce(const MassPoint& p0, const MassPoint& p1, float restLength) {
	float currLen = norm(p0.position - p1.position);
	return -m_fStiffness * (currLen - restLength) * (p0.position - p1.position) / currLen;
}



void ComplexSimulator::addExternalForces()
{
	//to be filled for specific test case
}

void ComplexSimulator::drawRigidBodies()
{
	for (int i = 0; i < m_bodies.size(); i++) {
		if (m_iTestCase == 3 && i == 0) {
			DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.2, 0.75, 0.2));
		}
		else {
			DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));
		}
		DUC->drawRigidBody(obj2World(m_bodies[i]));
	}
}

void ComplexSimulator::euler(float timestep)
{
	for (RigidBody& b : m_bodies) {
		// if (b.isFixed)
		//     continue;

		 // add external forces
		Vec3 force = Vec3(0, 0, 0);

		for (const ExternalForce& f : b.ext_forces)
			force += f.m_force;

		//update position
		b.position += timestep * b.linear_vel;
		b.linear_vel += timestep * (force / b.mass);

		if (m_gravity && !b.isFixed)
			b.linear_vel += timestep * VEC3_GRAVITY;
	}
}

void ComplexSimulator::do_collisions()
{
	if (m_bodies.size() <= 1) {
		return;
	}

	for (int i = 0; i < m_bodies.size() - 1; ++i) {
		for (int j = i + 1; j < m_bodies.size(); ++j) {
			CollisionInfo info = checkCollisionSAT(obj2World(m_bodies[i]), obj2World(m_bodies[j]));

			if (info.isValid) {
				// found collision
				collide(m_bodies[i], m_bodies[j], info);
			}
		}
	}

}

Mat4 ComplexSimulator::obj2World(const RigidBody& body)
{
	Mat4 scale_mat, rot_mat, translat_mat;

	scale_mat.initScaling(body.size.x, body.size.y, body.size.z);
	rot_mat = body.orientation.getRotMat();
	translat_mat.initTranslation(body.position.x, body.position.y, body.position.z);

	return scale_mat * rot_mat * translat_mat;
}

void ComplexSimulator::collide(RigidBody& b1, RigidBody& b2, const CollisionInfo& info)
{
	// compute local collision points
	Vec3 x1 = info.collisionPointWorld - b1.position;
	Vec3 x2 = info.collisionPointWorld - b2.position;

	// compute velocities at collision point
	Vec3 v1 = b1.linear_vel + cross(b1.angular_vel, x1);
	Vec3 v2 = b2.linear_vel + cross(b2.angular_vel, x2);

	// compute relative velocity
	Vec3 v_rel = v1 - v2;

	Vec3 normal = info.normalWorld;

	auto separating = v_rel * normal;
	if (dot(v_rel, normal) > 0) {
		// bodies are separating
		return;
	}
	//compute impulse
	Vec3 J;
	Vec3 J_top = -dot((1 + bounciness) * v_rel, normal);

	Vec3 aux1 = cross(b1.current_inv_I * cross(x1, normal), x1);
	Vec3 aux2 = cross(b2.current_inv_I * cross(x2, normal), x2);
	Vec3 J_bottom = 1.0f / b1.mass + 1.0f / b2.mass + dot((aux1 + aux2), normal);
	J = J_top / J_bottom;

	//update values
	b1.linear_vel += J * normal / b1.mass;
	b2.linear_vel -= J * normal / b2.mass;

	b1.momentum += cross(x1, J * normal);
	b2.momentum -= cross(x2, J * normal);
}



void ComplexSimulator::angularCalculations(float timestep)
{
	for (RigidBody& b : m_bodies) {
		// if (b.isFixed)
		//     continue;

		 //compute torque
		Vec3 torque = Vec3(0, 0, 0);
		for (const ExternalForce& f : b.ext_forces) {
			torque += cross(f.m_contact_point, f.m_force);
		}

		//update orientation
		b.orientation += (timestep / 2) * Quat(b.angular_vel.x, b.angular_vel.y, b.angular_vel.z, 0) * b.orientation;
		//renormalize quaternions
		if (b.orientation.norm() != 0)
			b.orientation /= b.orientation.norm();

		//update momentum
		b.momentum += timestep * torque;

		//update angular velocity
		Mat4 rot = b.orientation.getRotMat();
		Mat4 rot_trans = b.orientation.getRotMat();
		rot_trans.transpose();
		Mat4 I_inv = rot * b.inverseI0 * rot_trans;
		// b.angular_vel = I_inv * b.momentum;
		b.angular_vel = I_inv.transformVector(b.momentum);
		b.current_inv_I = I_inv;
	}
}