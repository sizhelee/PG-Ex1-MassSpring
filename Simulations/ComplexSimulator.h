#pragma once

#ifndef COMPLEXSIMULATOR_h
#define COMPLEXSIMULATOR_h
#endif

#include "Simulator.h"
#include "collisionDetect.h"
#include <vector>

#define FLOOR_LIMIT -0.5f
#define CEILING_LIMIT 0.5f
#define LEFT_WALL_LIMIT -0.5f
#define RIGHT_WALL_LIMIT 0.5f
#define FAR_WALL_LIMIT 0.5f
#define CLOSE_WALL_LIMIT -0.5f

#define VEC3_GRAVITY Vec3(0, -0.81f, 0)

struct MassPoint {
	Vec3 position;
	Vec3 velocity;
	Vec3 force;
	bool isFixed;
	Vec3 acceleration;

	MassPoint(Vec3 position, Vec3 velocity, bool isFixed) : position{ position }, velocity{ velocity }, isFixed{ isFixed }, force{ Vec3(0,0,0) } {}

	void print() {
		std::cout << "[p=(" << this->position.x << ", " << this->position.y << ", " << this->position.z << "), v=("
			<< this->velocity.x << ", " << this->velocity.y << ", " << this->velocity.z << "), f=("
			<< this->force.x << ", " << this->force.y << ", " << this->force.z << "), isFixed = "
			<< this->isFixed << "]" << endl;
	}
};

struct Spring {
public:
	int point0;
	int point1;
	//float stiffness;
	float initialLength;

	Spring(int point0, int point1, float initialLength) :
		point0{ point0 }, point1{ point1 }, initialLength{ initialLength } {}

	void print() {
		std::cout << "[p0 = " << this->point0 << ", p1 = " << this->point1
			<< ", L = " << this->initialLength << std::endl;
	}
};


struct ExternalForce {
	Vec3 m_force;
	Vec3 m_contact_point;

	ExternalForce(Vec3 force, Vec3 contact_point) : m_force{ force }, m_contact_point{ contact_point } {}
};

struct RigidBody {

	//Vec3 center;
	Vec3 position;
	Vec3 linear_vel;
	Vec3 angular_vel;
	Vec3 momentum;
	bool isFixed;

	//length, height, width
	Vec3 size;
	double mass;
	Quat orientation;
	std::vector<ExternalForce> ext_forces;

	Mat4 inverseI0;
	Mat4 current_inv_I;


	RigidBody(Vec3 position, Vec3 size, double mass, bool fixed = false)
		: position{ position }, size{ size }, mass{ mass }, isFixed{ fixed }
	{
		momentum = Vec3(0, 0, 0);
		linear_vel = Vec3(0, 0, 0);
		angular_vel = Vec3(0, 0, 0);

		double l = size.x;
		double h = size.y;
		double w = size.z;
		inverseI0.initId();
		inverseI0.value[0][0] = 12.0f / (mass * (h * h + w * w));
		inverseI0.value[1][1] = 12.0f / (mass * (l * l + w * w));
		inverseI0.value[2][2] = 12.0f / (mass * (l * l + h * h));
		inverseI0.value[3][3] = 1.0f;
		orientation = Quat(0, 0, 0);
	}

};


class ComplexSimulator : public Simulator {
public:
	// Construtors
	ComplexSimulator();

	// Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	//MassSpringSystemFunctions

	// Specific Functions
	void setMass(float mass);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 Velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void applyExternalForce(Vec3 force);

	//RigidBodySystem functions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	int addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	// MassSpringSystem attributes
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	bool m_collisions = false;
	bool m_gravity = false;
	std::vector<MassPoint> m_points;
	std::vector<Spring> m_springs;

	float m_pointScaleVal = 0.05f;
	Vec3 m_pointScale = Vec3(m_pointScaleVal, m_pointScaleVal, m_pointScaleVal);
	Vec3 m_springColor = Vec3(0.0f, 0.5f, 1.0f);


	// MassSpringSystem methods 
	void drawPoints();
	void drawSprings();
	void clearForces();
	Vec3 computeInternalForce(const MassPoint& p0, const MassPoint& p1, float restLength);

	void midpoint(float step, bool gravity, bool collisions);


	// Ridgid bodies attributes
	float bounciness = 0.3;

	std::vector<RigidBody> m_bodies;

	void addExternalForces();
	void drawRigidBodies();
	void euler(float timestep);
	void angularCalculations(float timestep);
	void do_collisions();
	Mat4 obj2World(const RigidBody& body);
	void collide(RigidBody& b1, RigidBody& b2, const CollisionInfo& info);
};
