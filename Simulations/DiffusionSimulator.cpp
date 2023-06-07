#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include <cstdlib>
#include <ctime>
using namespace std;


DiffusionSimulator::DiffusionSimulator()
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();

	m_rows = 16;
	m_cols = 16;

	T = new Grid(m_rows, m_cols);

	srand(time(NULL));
}

const char* DiffusionSimulator::getTestCasesStr() {
	return "Explicit, Explicit(random), unstable, "
		"Implicit, Implicit(random), stable";
}

void DiffusionSimulator::reset() {
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	TwAddVarRW(this->DUC->g_pTweakBar, "Rows", TW_TYPE_INT32, &m_rows, "step=1 min=5 max=100");
	TwAddVarRW(this->DUC->g_pTweakBar, "Columns", TW_TYPE_INT32, &m_cols, "step=1 min=5 max=100");

	if (m_iTestCase == 1 || m_iTestCase == 4)
		TwAddVarRW(this->DUC->g_pTweakBar, "Random points", TW_TYPE_INT32, &nb_rand_points, "step=1 min=1 max=10");
}

void fillTRand(Grid* T, int n) 
{
	for (int point = 0; point < n; point++) 
	{
		int i = (rand() % (T->rows - 2)) + 1;
		int j = (rand() % (T->cols - 2)) + 1;

		int sign = pow(-1, rand() % 2);
		float val = sign * static_cast <float> (rand()) / static_cast <float> (RAND_MAX);

		T->data[i][j] = val;
	}
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);

	delete T;
	T = new Grid(m_rows, m_cols);

	diffusion_constant = 4;

	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		break;
	case 1:
		cout << "Explicit(random)\n";
		fillTRand(T, nb_rand_points);
		break;
	case 2:
		cout << "unstable(Explicit)\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		diffusion_constant = 300;
		break;
	case 3:
		cout << "Implicit\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		break;
	case 4:
		cout << "Implicit(random)\n";
		fillTRand(T, nb_rand_points);
		break;
	case 5:
		cout << "stable(Implicit)\n";
		T->data[T->rows / 2][T->cols / 2] = 1;
		diffusion_constant = 300;
		break;
	default:
		cout << "No Test!\n";
		break;
	}
}

Grid* DiffusionSimulator::ExplicitDiffusion(float timeStep) {
	Grid* newT = new Grid(T->rows, T->cols);

	for (int i = 1; i < T->rows - 1; i++) 
	{
		for (int j = 1; j < T->cols - 1; j++) 
		{
			float x_val = (T->data[i + 1][j] - 2 * T->data[i][j] + T->data[i - 1][j]);
			float y_val = (T->data[i][j + 1] - 2 * T->data[i][j] + T->data[i][j - 1]);

			newT->data[i][j] = T->data[i][j] + timeStep * diffusion_constant * (x_val + y_val);
		}
	}

	delete T;
	return newT;
}

void setupB(std::vector<Real>& b, int N, const Grid* T) 
{
	int k = 0;
	for (int i = 0; i < T->rows; i++)
		for (int j = 0; j < T->cols; j++)
			b.at(k++) = T->data[i][j];

	b.at(0) = 0;
	b.at(N - 1) = 0;
}

void setupA(SparseMatrix<Real>& A, double factor, int rows, int cols) 
{
	int N = rows * cols;
	for (int i = 0; i < N; i++)
		A.set_element(i, i, 1); // 对角线元素设为 1

	for (int row = 0; row < N; row++) 
	{
		int i = row / cols, j = row % cols;
		if (i == 0 || i == rows - 1 || j == 0 || j == cols - 1)
			continue;

		A.set_element(row, (i - 1) * cols + j, -factor);
		A.set_element(row, (i + 1) * cols + j, -factor);
		A.set_element(row, i * cols + j, 1 + 4 * factor);
		A.set_element(row, i * cols + (j - 1), -factor);
		A.set_element(row, i * cols + (j + 1), -factor);
	}
}


void DiffusionSimulator::ImplicitDiffusion(float timeStep)
{
	// 求解线性方程
	const int N = T->rows * T->cols;
	SparseMatrix<Real>* A = new SparseMatrix<Real>(N);
	std::vector<Real>* b = new std::vector<Real>(N);

	setupA(*A, diffusion_constant * timeStep, T->rows, T->cols);
	setupB(*b, N, T);

	Real pcg_target_residual = 1e-05, pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;

	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	std::vector<Real> x(N);
	for (int j = 0; j < N; ++j) 
		x[j] = 0.0;

	solver.solve(*A, *b, x, ret_pcg_residual, ret_pcg_iterations, 0);

	int k = 0;
	for (int i = 0; i < T->rows; i++)
		for (int j = 0; j < T->cols; j++)
			T->data[i][j] = x[k++];

	// 边界设为 0
	for (int i = 0; i < T->rows; i++)
		for (int j = 0; j < T->cols; j++)
			if (i == 0 || j == 0 || i == T->rows - 1 || j == T->cols - 1)
				T->data[i][j] = 0;

	delete b;
	delete A;
}


void DiffusionSimulator::simulateTimestep(float timeStep)
{
	switch (m_iTestCase)
	{
	case 0: 
	case 1: 
	case 2: 
	case 3:
		T = ExplicitDiffusion(timeStep);
		break;
	case 4: 
	case 5: 
	case 6: 
	case 7:
		ImplicitDiffusion(timeStep);
		break;
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	constexpr float x_step = 0.04, y_step = 0.04;
	constexpr float sphere_scale = 0.02;

	for (int i = 0; i < T->rows; i++)
	{
		for (int j = 0; j < T->cols; j++)
		{
			float val = T->data[i][j];
			if (i == 0 || j == 0 || i == T->rows - 1 || j == T->cols - 1)
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, Vec3(0.8, 0.8, 0.8));
			else if (val > 0)
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 50 * Vec3(0, val, 0));
			else
				DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 50 * Vec3(-val, 0, 0));

			Vec3 position((double)x_step * j - 0.5, -(double)y_step * i + 0.5, 0);
			DUC->drawSphere(position, sphere_scale);
		}
	}
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}