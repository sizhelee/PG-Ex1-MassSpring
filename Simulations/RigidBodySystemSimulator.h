#ifndef RIGIDBODYSYSTEMSIMULATOR_h
#define RIGIDBODYSYSTEMSIMULATOR_h
#include "Simulator.h"
//add your header for your rigid body system, for e.g.,
//#include "rigidBodySystem.h" 
#include "collisionDetect.h"
#include <vector>

#define TESTCASEUSEDTORUNTEST 2

struct ExternalForce {
	Vec3 m_force;
	Vec3 m_contact_point;

	ExternalForce(Vec3 force, Vec3 contact_point) : m_force{ force }, m_contact_point{ contact_point } {}
};

struct RigidBody {

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

		double l = size.x, h = size.y, w = size.z;
		inverseI0.initId();
		inverseI0.value[0][0] = 12.0f / (mass * (h * h + w * w));
		inverseI0.value[1][1] = 12.0f / (mass * (l * l + w * w));
		inverseI0.value[2][2] = 12.0f / (mass * (l * l + h * h));
		inverseI0.value[3][3] = 1.0f;
		orientation = Quat(0, 0, 0);
	}

};

class RigidBodySystemSimulator :public Simulator {
public:
	// Construtors
	RigidBodySystemSimulator() {};

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

	// ExtraFunctions
	int getNumberOfRigidBodies();
	Vec3 getPositionOfRigidBody(int i);
	Vec3 getLinearVelocityOfRigidBody(int i);
	Vec3 getAngularVelocityOfRigidBody(int i);
	void applyForceOnBody(int i, Vec3 loc, Vec3 force);
	int addRigidBody(Vec3 position, Vec3 size, int mass);
	void setOrientationOf(int i, Quat orientation);
	void setVelocityOf(int i, Vec3 velocity);

private:
	// Attributes
	// add your RigidBodySystem data members, for e.g.,
	// RigidBodySystem * m_pRigidBodySystem; 
	Vec3 m_externalForce;

	float m_externalForceX = 0;
	float m_externalForceY = 0;
	float m_externalForceZ = 0;
	float m_bounciness = 0.3;

	// UI Attributes
	float m_collisionObjSpeed1 = 1.5;
	float m_collisionObjSpeed2 = 1.5;
	int m_collisionObjMass1 = 2;
	int m_collisionObjMass2 = 2;

	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	std::vector<RigidBody> m_bodies;

	void addExternalForces();
	void eulerCalculate(float timestep);
	void angularCalculate(float timestep);
	void collisionCalculate();
	Mat4 obj2World(const RigidBody& body);
	void collide(RigidBody& b1, RigidBody& b2, const CollisionInfo& info);
};
#endif