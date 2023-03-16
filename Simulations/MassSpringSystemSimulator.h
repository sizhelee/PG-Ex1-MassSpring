#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change


class MassSpringSystemSimulator:public Simulator{
public:

	struct Masspoint {
		float mp_pass;
		Vec3 mp_position;
		Vec3 mp_velocity;
		Vec3 mp_force;
		bool mp_isFixed;
	};

	struct Spring {
		int smp1index;
		int smp2index;
		Masspoint s_mp1, s_mp2;
		float s_stiffness;
		float s_initLength;
		float s_currLength;
	};

	// Construtors
	MassSpringSystemSimulator();
	
	// UI Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float timeStep);
	void onClick(int x, int y);
	void onMouse(int x, int y);

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

	Vec3 ComputeForce(Spring s);
	float PointDistance(Vec3 a, Vec3 b);

	void ApplyForce(Spring s, Vec3 F);

	void eulerIntegratePositions(float timeStep);
	void eulerIntegrateVelocity(float timeStep);

	
	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}

private:
	// Data Attributes
	float m_fMass;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;
	vector<Masspoint> masspoints, masspoints_tmp;
	vector<Spring> springs, springs_tmp;

	// UI Attributes
	Vec3 m_externalForce;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
};
#endif