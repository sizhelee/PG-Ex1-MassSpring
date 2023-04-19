#include "RigidBodySystemSimulator.h"

#define PI 3.141592653589

const char* RigidBodySystemSimulator::getTestCasesStr() {
    return "(1) one-step, (2) single body, (3) two-body collision, (4) Complex";
}
void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC) {
    this->DUC = DUC;

    switch (m_iTestCase)
    {
    case 1:
        TwAddVarRW(this->DUC->g_pTweakBar, "ExtForceLoc.x", TW_TYPE_FLOAT, &m_externalForceX, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "ExtForceLoc.y", TW_TYPE_FLOAT, &m_externalForceY, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "ExtForceLoc.z", TW_TYPE_FLOAT, &m_externalForceZ, "step=0.1");
        break;
    case 2:
        TwAddVarRW(this->DUC->g_pTweakBar, "Speed Obj 1", TW_TYPE_FLOAT, &m_collisionObjSpeed1, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Speed Obj 2", TW_TYPE_FLOAT, &m_collisionObjSpeed2, "step=0.1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Mass Obj 1", TW_TYPE_INT32, &m_collisionObjMass1, "step=1 min=1");
        TwAddVarRW(this->DUC->g_pTweakBar, "Mass Obj 2", TW_TYPE_INT32, &m_collisionObjMass2, "step=1 min=1");
        TwAddVarRW(this->DUC->g_pTweakBar, "bounciness", TW_TYPE_FLOAT, &m_bounciness, "step = 0.1 min = 0 max = 1");
        break;

    default: break;
    }
}

void RigidBodySystemSimulator::reset()
{
    m_mouse.x = m_mouse.y = 0;
    m_trackmouse.x = m_trackmouse.y = 0;
    m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

Mat4 RigidBodySystemSimulator::obj2World(const RigidBody& body) {
    Mat4 scaleMatrix, rotMatrix, translatMatrix;

    scaleMatrix.initScaling(body.size.x, body.size.y, body.size.z);
    rotMatrix = body.orientation.getRotMat();
    translatMatrix.initTranslation(body.position.x, body.position.y, body.position.z);

    return scaleMatrix * rotMatrix * translatMatrix;
}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {
    
    for (int i = 0; i < m_bodies.size(); i++) 
    {
        if (m_iTestCase == 2 && i == 0)
            DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.75, 0.2, 0.2));
        else if (m_iTestCase == 3 && i == 0)
            DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.2, 0.2, 0.6));
        else DUC->setUpLighting(Vec3(0, 0, 0), 0.4 * Vec3(1, 1, 1), 2000.0, Vec3(0.5, 0.5, 0.5));

        DUC->drawRigidBody(obj2World(m_bodies[i]));
    }
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase) {
    m_iTestCase = testCase;
    m_bodies.clear();

    switch (m_iTestCase)
    {
    case 0: 
    {
        std::cout << "Demo 1: A simple one-step test" << std::endl;
        Vec3 x1(0.3, 0.5, 0.25);
        int bodyDemo = addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        setOrientationOf(bodyDemo, Quat(0, 0, M_PI / 2));
        applyForceOnBody(bodyDemo, x1, Vec3(1, 1, 0));
        simulateTimestep(2);

        Vec3 v = getLinearVelocityOfRigidBody(bodyDemo);
        Vec3 w = getAngularVelocityOfRigidBody(bodyDemo);

        std::cout << "Position: " << getPositionOfRigidBody(bodyDemo) << endl;
        std::cout << "Linear velocity: " << v << endl;
        std::cout << "Angular velocity: " << w << endl;
        std::cout << "World velocity: " << v + cross(w, x1) << endl;
        std::cout << std::endl;
        m_bodies.clear();

        break;
    }
    case 1: 
    {
        std::cout << "Demo 2: Simple single body simulation" << std::endl;

        int b1 = addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.6, 0.5), 2);
        setOrientationOf(b1, Quat(0, 0, M_PI / 2));

        break;
    }
    case 2: 
    {
        std::cout << "Demo 3: Two-rigid-body collision" << std::endl;

        int b1 = addRigidBody(Vec3(0, 0, 0), Vec3(1, 0.5, 0.5), m_collisionObjMass1);
        setOrientationOf(b1, Quat(0, -M_PI / 4, M_PI / 4));
        setVelocityOf(b1, Vec3(m_collisionObjSpeed1, 0, 0));

        int b2 = addRigidBody(Vec3(2, 0, 0), Vec3(1, 0.5, 0.5), m_collisionObjMass2);
        setOrientationOf(b2, Quat(0, 0, M_PI / 2));
        setVelocityOf(b2, Vec3(-m_collisionObjSpeed2, 0, 0));

        break;
    }
    case 3: 
    {
        std::cout << "Demo 4: Complex simulation" << std::endl;

        int b1 = addRigidBody(Vec3(-0.5, 0, 0), Vec3(0.3, 0.3, 0.3), 3);
        setOrientationOf(b1, Quat(0, M_PI / 4, M_PI / 4));

        int b2 = addRigidBody(Vec3(0.1, 0.26, 0), Vec3(0.25, 0.25, 0.25), 2);
        int b3 = addRigidBody(Vec3(0.1, 0, 0), Vec3(0.25, 0.25, 0.25), 2);
        int b4 = addRigidBody(Vec3(0.1, -0.26, 0), Vec3(0.25, 0.25, 0.25), 2);
        int b5 = addRigidBody(Vec3(0.4, 0.13, 0), Vec3(0.25, 0.25, 0.25), 2);
        int b6 = addRigidBody(Vec3(0.4, -0.13, 0), Vec3(0.25, 0.25, 0.25), 2);
        int b7 = addRigidBody(Vec3(0.4, 0.39, 0), Vec3(0.25, 0.25, 0.25), 2);
        int b8 = addRigidBody(Vec3(0.4, -0.39, 0), Vec3(0.25, 0.25, 0.25), 2);

        break;
    }
    default:
        break;
    }
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed) {
    
    Point2D change;
    change.x = m_trackmouse.x - m_oldtrackmouse.x;
    change.y = m_trackmouse.y - m_oldtrackmouse.y;
    if (change.x || change.y)
    {
        Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix()).inverse();
        Vec3 originalView = Vec3((float)change.x, (float)-change.y, 0);

        Vec3 originalWorld = worldViewInv.transformVectorNormal(originalView);
        originalWorld = originalWorld * 0.001f;

        m_externalForce = originalWorld * 70;
        m_externalForce.y = -m_externalForce.y;
        m_externalForce.x = -m_externalForce.x;
    }
    else m_externalForce = (0, 0, 0);
}

void RigidBodySystemSimulator::addExternalForces() {
    switch (m_iTestCase)
    {
    case 1:
        applyForceOnBody(0, Vec3(m_externalForceX, m_externalForceY, m_externalForceZ), m_externalForce);
        break;
    case 3:
        applyForceOnBody(0, Vec3(0, 0, 0), m_externalForce);
        break;
    default:
        break;
    }
}

void RigidBodySystemSimulator::angularCalculate(float timestep) {
    for (RigidBody& b : m_bodies) 
    {
        //update orientation
        b.orientation += (timestep / 2) * Quat(b.angular_vel.x, b.angular_vel.y, b.angular_vel.z, 0) * b.orientation;
        if (b.orientation.norm() != 0)
            b.orientation /= b.orientation.norm();

        //compute torque and update momentum
        Vec3 torque = Vec3(0, 0, 0);
        for (const ExternalForce& f : b.ext_forces)
            torque += cross(f.m_contact_point, f.m_force);
        b.momentum += timestep * torque;

        //update angular velocity
        Mat4 rot = b.orientation.getRotMat(), rot_trans = b.orientation.getRotMat();
        rot_trans.transpose();
        Mat4 I_inv = rot * b.inverseI0 * rot_trans;
        b.angular_vel = I_inv.transformVector(b.momentum);
        b.current_inv_I = I_inv;
    }
}

void RigidBodySystemSimulator::eulerCalculate(float timestep) {

    for (RigidBody& b : m_bodies) 
    {
        Vec3 force = Vec3(0, 0, 0);

        for (const ExternalForce& f : b.ext_forces)
            force += f.m_force;

        b.position += timestep * b.linear_vel;
        b.linear_vel += timestep * (force / b.mass);
    }

}

void RigidBodySystemSimulator::collide(RigidBody& b1, RigidBody& b2, const CollisionInfo& info) {

    Vec3 x1 = info.collisionPointWorld - b1.position;
    Vec3 x2 = info.collisionPointWorld - b2.position;

    Vec3 v1 = b1.linear_vel + cross(b1.angular_vel, x1);
    Vec3 v2 = b2.linear_vel + cross(b2.angular_vel, x2);
    Vec3 V = v1 - v2;

    Vec3 normal = info.normalWorld;

    auto separating = V * normal;
    if (dot(V, normal) > 0) return;

    Vec3 J_top = -dot((1 + m_bounciness) * V, normal);

    Vec3 aux1 = cross(b1.current_inv_I * cross(x1, normal), x1);
    Vec3 aux2 = cross(b2.current_inv_I * cross(x2, normal), x2);
    Vec3 J_bottom = 1.0f / b1.mass + 1.0f / b2.mass + dot((aux1 + aux2), normal);
    Vec3 J = J_top / J_bottom;

    b1.linear_vel += J * normal / b1.mass;
    b2.linear_vel -= J * normal / b2.mass;
    b1.momentum += cross(x1, J * normal);
    b2.momentum -= cross(x2, J * normal);
}

void RigidBodySystemSimulator::collisionCalculate() {

    if (m_bodies.size() <= 1)   return;

    for (int i = 0; i < m_bodies.size() - 1; ++i) 
    {
        for (int j = i + 1; j < m_bodies.size(); ++j) 
        {
            CollisionInfo info = checkCollisionSAT(obj2World(m_bodies[i]), obj2World(m_bodies[j]));
            if (info.isValid)
                collide(m_bodies[i], m_bodies[j], info);
        }
    }
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep) {

    addExternalForces();

    eulerCalculate(timeStep);

    angularCalculate(timeStep);

    collisionCalculate();

    for (RigidBody& b : m_bodies)
        b.ext_forces.clear();
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
    m_oldtrackmouse.x = x;
    m_oldtrackmouse.y = y;
    m_trackmouse.x = x;
    m_trackmouse.y = y;
}


//---------------------- EXTRA FUNCTIONS -----------------------//


int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
    return m_bodies.size();
}

Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
    RigidBody body = m_bodies[i];

    return body.position;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
    return m_bodies[i].linear_vel;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
    return m_bodies[i].angular_vel;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
    m_bodies[i].ext_forces.emplace_back(force, loc);
}

int RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
    int index = m_bodies.size();
    m_bodies.emplace_back(position, size, mass);
    return index;
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
    m_bodies[i].orientation = orientation;
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
    m_bodies[i].linear_vel = velocity;
}

