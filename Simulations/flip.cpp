#include "flip.h"
#include <list>
#include <math.h>
#include <algorithm>

#define FLUID_CELL 0
#define SOLID_CELL 1
#define AIR_CELL 2

#define random(x) rand()%(x)

float gravity = -9.81;

float clamp(float x, float min, float max)
{
	if (x < min)
		return min;
	else if (x > max)
		return max;
	else
		return x;
}

const char* FlipSimulator::getTestCasesStr() {
	return "(1) PIC method";
}

void FlipSimulator::initUI(DrawingUtilitiesClass* DUC) {
	this->DUC = DUC;
}

void FlipSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void FlipSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext) {

	for (int i = 0; i < m_particlePos.size(); i++)
	{
		DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
		DUC->drawSphere(m_particlePos[i], (0.01, 0.01, 0.01));
	}
}

void FlipSimulator::notifyCaseChanged(int testCase) {

	switch (testCase)
	{
	case 0:
	{
		std::cout << "Demo 1" << std::endl;
		setupScene(20);
		break;
	}
	default:
		break;
	}
}

void FlipSimulator::externalForcesCalculations(float timeElapsed) {

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

void FlipSimulator::onClick(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
}

void FlipSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void FlipSimulator::integrateParticles(float timeStep) 
{
	for (int i = 0; i < m_iNumSpheres; i++) {
		m_particleVel[i][1] += timeStep * gravity;
		m_particlePos[i] += m_particleVel[i] * timeStep;
	}
}

void FlipSimulator::transferVelocities(bool toGrid, float flipRatio)
{
	float h = m_h;
	float h1 = m_fInvSpacing;
	float h2 = 0.5 * h;

	if (toGrid) 
	{
		for (int i = 0; i < 3; i++)
			prevVel[i] = std::vector<float>(m_iNumCells);
		for (int i = 0; i < 3; i++)
			dvel[i] = std::vector<float>(m_iNumCells);
		for (int i = 0; i < 3; i++)
			vel[i] = std::vector<float>(m_iNumCells);

		for (int i = 0; i < m_iNumCells; i++)
			m_type[i] = m_s[i] == 0.0 ? AIR_CELL : FLUID_CELL;

		for (int i = 0; i < m_iNumSpheres; i++) 
		{
			Vec3 nowPos = m_particlePos[i];

			float xi = clamp(floor(nowPos[0] * h1), 0, m_iCellX - 1);
			float yi = clamp(floor(nowPos[1] * h1), 0, m_iCellY - 1);
			float zi = clamp(floor(nowPos[2] * h1), 0, m_iCellZ - 1);

			float cellNr = xi * m_iCellZ * m_iCellY + yi * m_iCellZ + zi;
			if (m_type[cellNr] == AIR_CELL)
				m_type[cellNr] = FLUID_CELL;
		}
	}

	for (int component = 0; component < 3; component++) 
	{

		float dx = component == 0 ? h2 : 0.0;
		float dy = component == 1 ? h2 : 0.0;
		float dz = component == 2 ? h2 : 0.0;

		std::vector<float> f = vel[component];
		std::vector<float> prevF = prevVel[component];
		std::vector<float> d = dvel[component];


		for (int i = 0; i < m_iNumSpheres; i++) 
		{
			Vec3 nowPos = m_particlePos[i];
			float x = nowPos[0];
			float y = nowPos[1];
			float z = nowPos[2];

			x = clamp(x, h, (m_iCellX - 1) * h);
			y = clamp(y, h, (m_iCellY - 1) * h);
			z = clamp(y, h, (m_iCellZ - 1) * h);

			float x0 = min(floor((x - dx) * h1), float(m_iCellX - 2));
			float tx = ((x - dx) - x0 * h) * h1;
			float x1 = min(x0 + 1, float(m_iCellX - 2));

			float y0 = min(floor((y - dy) * h1), float(m_iCellY - 2));
			float ty = ((y - dy) - y0 * h) * h1;
			float y1 = min(y0 + 1, float(m_iCellY - 2));

			float z0 = min(floor((z - dz) * h1), float(m_iCellZ - 2));
			float tz = ((z - dz) - z0 * h) * h1;
			float z1 = min(z0 + 1, float(m_iCellZ - 2));

			float sx = 1.0 - tx;
			float sy = 1.0 - ty;
			float sz = 1.0 - tz;

			float d0 = sx * sy * sz;
			float d1 = tx * sy * sz;
			float d2 = tx * ty * sz;
			float d3 = sx * ty * sz;
			float d4 = sx * sy * tz;
			float d5 = tx * sy * tz;
			float d6 = tx * ty * tz;
			float d7 = sx * ty * tz;

			//std::cout << d0 << " " << d1 << " " << d2 << " " << d3 << " " << d4 << " " << d5 << " " << d6 << " " << d7 << std::endl;

			int n1 = m_iCellY * m_iCellZ, n2 = m_iCellZ;

			float nr0 = x0 * n1 + y0 * n2 + z0;
			float nr1 = x1 * n1 + y0 * n2 + z0;
			float nr2 = x1 * n1 + y1 * n2 + z0;
			float nr3 = x0 * n1 + y1 * n2 + z0;
			float nr4 = x0 * n1 + y0 * n2 + z1;
			float nr5 = x1 * n1 + y0 * n2 + z1;
			float nr6 = x1 * n1 + y1 * n2 + z1;
			float nr7 = x0 * n1 + y1 * n2 + z1;

			//std::wcout << nr0 << " " << nr1 << " " << nr2 << " " << nr3 << " " << nr4 << " " << nr5 << " " << nr6 << " " << nr7 << std::endl;

			if (toGrid) {
				float pv = m_particleVel[i][component];
				f[nr0] += pv * d0;  d[nr0] += d0;
				f[nr1] += pv * d1;  d[nr1] += d1;
				f[nr2] += pv * d2;  d[nr2] += d2;
				f[nr3] += pv * d3;  d[nr3] += d3;
				f[nr4] += pv * d4;  d[nr4] += d4;
				f[nr5] += pv * d5;  d[nr5] += d5;
				f[nr6] += pv * d6;  d[nr6] += d6;
				f[nr7] += pv * d7;  d[nr7] += d7;
			}
			else {
				float offset = component == 0 ? n1 : 1;
				if (component == 1)
					offset = n2;
				float valid0 = m_type[nr0] != AIR_CELL ? 1.0 : 0.0;
				float valid1 = m_type[nr1] != AIR_CELL ? 1.0 : 0.0;
				float valid2 = m_type[nr2] != AIR_CELL ? 1.0 : 0.0;
				float valid3 = m_type[nr3] != AIR_CELL ? 1.0 : 0.0;
				float valid4 = m_type[nr4] != AIR_CELL ? 1.0 : 0.0;
				float valid5 = m_type[nr5] != AIR_CELL ? 1.0 : 0.0;
				float valid6 = m_type[nr6] != AIR_CELL ? 1.0 : 0.0;
				float valid7 = m_type[nr7] != AIR_CELL ? 1.0 : 0.0;

				float v = m_particleVel[i][component];
				float d = valid0 * d0 + valid1 * d1 + valid2 * d2 + valid3 * d3 + valid4 * d4 + 
					valid5 * d5 + valid6 * d6 + valid7 * d7;

				if (d > 0.0) {

					float picV = (
						valid0 * d0 * f[nr0] + valid1 * d1 * f[nr1] + valid2 * d2 * f[nr2] +
						valid3 * d3 * f[nr3] + valid4 * d4 * f[nr4] + valid5 * d5 * f[nr5] + 
						valid6 * d6 * f[nr6] + valid7 * d7 * f[nr7]) / d;
					float corr = (
						valid0 * d0 * (f[nr0] - prevF[nr0]) + 
						valid1 * d1 * (f[nr1] - prevF[nr1]) + 
						valid2 * d2 * (f[nr2] - prevF[nr2]) + 
						valid3 * d3 * (f[nr3] - prevF[nr3]) + 
						valid4 * d4 * (f[nr4] - prevF[nr4]) + 
						valid5 * d5 * (f[nr5] - prevF[nr5]) +
						valid6 * d6 * (f[nr6] - prevF[nr6]) +
						valid7 * d7 * (f[nr7] - prevF[nr7])
						) / d;
					float flipV = v + corr;

					m_particleVel[i][component] = (1.0 - flipRatio) * picV + flipRatio * flipV;
					//std::cout << component << ": " << picV << " " << flipV << std::endl;
				}
			}
		}

		if (toGrid) {
			for (int i = 0; i < f.size(); i++) {
				if (d[i] > 0.0)
					f[i] /= d[i];
			}

			// restore solid cells

			for (int i = 0; i < m_iCellX; i++) 
			{
				for (int j = 0; j < m_iCellY; j++) 
				{
					for (int k = 0; k < m_iCellZ; k++)
					{
						int n1 = m_iCellY * m_iCellZ, n2 = m_iCellZ;
						int solid = m_type[i * n1 + j * n2 + k] == SOLID_CELL;
						if (solid || (i > 0 && m_type[(i - 1) * n1 + j * n2 + k] == SOLID_CELL))
							vel[0][i * n1 + j * n2 + k] = prevVel[0][i * n1 + j * n2 + k];
						if (solid || (j > 0 && m_type[i * n1 + (j - 1) * n2 + k] == SOLID_CELL))
							vel[1][i * n1 + j * n2 + k] = prevVel[1][i * n1 + j * n2 + k];
						if (solid || (k > 0 && m_type[i * n1 + j * n2 + k - 1] == SOLID_CELL))
							vel[2][i * n1 + j * n2 + k] = prevVel[2][i * n1 + j * n2 + k];
					}
				}
			}
		}
	}
}

void FlipSimulator::pushParticlesApart(int numIters)
{
	float colorDiffusionCoeff = 0.001;

	for (int i = 0; i < m_iNumSpheres; i++) {
		float x = m_particlePos[i][0];
		float y = m_particlePos[i][1];
		float z = m_particlePos[i][2];

		float xi = clamp(floor(x * m_fInvSpacing), 0, m_iCellX - 1);
		float yi = clamp(floor(y * m_fInvSpacing), 0, m_iCellY - 1);
		float zi = clamp(floor(z * m_fInvSpacing), 0, m_iCellZ - 1);

		float cellNr = xi * m_iCellY * m_iCellZ + yi * m_iCellZ + zi;
		m_numCellParticles[cellNr]++;
	}

	//std::cout << "BBBBBBBB" << std::endl;

	// partial sums

	int first = 0;

	for (int i = 0; i < m_iNumCells; i++) {
		first += m_numCellParticles[i];
		m_firstCellParticle[i] = first;
	}
	m_firstCellParticle[m_iNumCells] = first;		// guard

	//std::cout << "CCCCCCCC" << std::endl;

	// fill particles into cells

	for (int i = 0; i < m_iNumSpheres; i++) {
		float x = m_particlePos[i][0];
		float y = m_particlePos[i][1];
		float z = m_particlePos[i][2];

		float xi = clamp(floor(x * m_fInvSpacing), 0, m_iCellX - 1);
		float yi = clamp(floor(y * m_fInvSpacing), 0, m_iCellY - 1);
		float zi = clamp(floor(z * m_fInvSpacing), 0, m_iCellZ - 1);

		float cellNr = xi * m_iCellY * m_iCellZ + yi * m_iCellZ + zi;
		m_firstCellParticle[cellNr]--;

		if (m_firstCellParticle[cellNr] >= 0)
			m_cellParticleIds[m_firstCellParticle[cellNr]] = i;
	}

	//std::cout << "AAAAAAAA" << std::endl;

	// push particles apart

	float minDist = 2.0 * m_particleRadius;
	float minDist2 = minDist * minDist;

	for (int iter = 0; iter < numIters; iter++) {

		for (int i = 0; i < m_iNumSpheres; i++) {
			float px = m_particlePos[i][0];
			float py = m_particlePos[i][1];
			float pz = m_particlePos[i][2];

			float pxi = floor(px * m_fInvSpacing);
			float pyi = floor(py * m_fInvSpacing);
			float pzi = floor(pz * m_fInvSpacing);

			float x0 = max(pxi - 1, 0.0f);
			float y0 = max(pyi - 1, 0.0f);
			float z0 = max(pzi - 1, 0.0f);

			float x1 = min(pxi + 1, float(m_iCellX - 1));
			float y1 = min(pyi + 1, float(m_iCellY - 1));
			float z1 = min(pzi + 1, float(m_iCellZ - 1));

			for (int xi = x0; xi <= x1; xi++) 
			{
				for (int yi = y0; yi <= y1; yi++) 
				{
					for (int zi = z0; zi <= z1; zi++)
					{
						int cellNr = xi * m_iCellY * m_iCellZ + yi * m_iCellZ + zi;
						int first = m_firstCellParticle[cellNr];
						int last = m_firstCellParticle[cellNr + 1];
						for (int j = first; j < last; j++) {
							int id = m_cellParticleIds[j];
							if (id == i)
								continue;
							float qx = m_particlePos[id][0];
							float qy = m_particlePos[id][1];
							float qz = m_particlePos[id][2];

							float dx = qx - px;
							float dy = qy - py;
							float dz = qz - pz;

							float d2 = dx * dx + dy * dy + dz * dz;
							if (d2 > minDist2 || d2 == 0.0)
								continue;
							float d = sqrt(d2);
							float s = 0.5 * (minDist - d) / d;
							dx *= s;
							dy *= s;
							dz *= s;

							m_particlePos[i][0] -= dx;
							m_particlePos[i][1] -= dy;
							m_particlePos[i][2] -= dz;
							m_particlePos[id][0] += dx;
							m_particlePos[id][1] += dy;
							m_particlePos[id][2] += dz;

							// diffuse colors

							for (int k = 0; k < 3; k++) {
								float color0 = m_particleColor[i][k];
								float color1 = m_particleColor[id][k];
								float color = (color0 + color1) * 0.5;
								m_particleColor[i][k] = color0 + (color - color0) * colorDiffusionCoeff;
								m_particleColor[id][k] = color1 + (color - color1) * colorDiffusionCoeff;
							}
						}
					}
				}
			}
		}
	}
}

void FlipSimulator::handleParticleCollisions(Vec3 obstaclePos, float obstacleRadius, Vec3 obstacleVel)
{
	/*
	* numX: 15, numY: 25, numZ: 15
	* dx: 0.03, dy: 0.02598, dz: 0.03
	* m_h: 0.05, point_r: 0.015
	* m_iCellX: 25, m_iCellY: 25, m_iCellZ: 25
	*/

	float h = m_h;
	float r = m_particleRadius;
	float or = obstacleRadius;
	float or2 = or *or ;
	float minDist = obstacleRadius + r;
	float minDist2 = minDist * minDist;

	float minX = h + r - 0.5;
	float maxX = (m_iCellX - 1) * h - r - 0.5;
	float minY = h + r - 0.5;
	float maxY = (m_iCellY - 1) * h - r - 0.5;
	float minZ = h + r - 0.5;
	float maxZ = (m_iCellZ - 1) * h - r - 0.5;


	for (int i = 0; i < m_iNumSpheres; i++) {
		float x = m_particlePos[i][0];
		float y = m_particlePos[i][1];
		float z = m_particlePos[i][2];

		float dx = x - obstaclePos[0];
		float dy = y - obstaclePos[1];
		float dz = z - obstaclePos[2];
		float d2 = dx * dx + dy * dy + dz * dz;

		// obstacle collision

		if (d2 < minDist2) {

			m_particleVel[i] = obstacleVel;
		}

		// wall collisions

		if (x < minX) {
			x = minX;
			m_particleVel[i][0] = 0.0;

		}
		if (x > maxX) {
			x = maxX;
			m_particleVel[i][0] = 0.0;
		}
		if (y < minY) {
			y = minY;
			m_particleVel[i][1] = 0.0;
		}
		if (y > maxY) {
			y = maxY;
			m_particleVel[i][1] = 0.0;
		}
		if (z < minZ) {
			z = minZ;
			m_particleVel[i][2] = 0.0;
		}
		if (z > maxZ) {
			z = maxZ;
			m_particleVel[i][2] = 0.0;
		}
		m_particlePos[i] = Vec3(x, y, z);
	}
}

void FlipSimulator::updateParticleDensity()
{
	int n1 = m_iCellY * m_iCellZ;
	int n2 = m_iCellZ;
	float h = m_h;
	float h1 = m_fInvSpacing;
	float h2 = 0.5 * h;

	std::vector<float> d = m_particleDensity;
	d.resize(m_iNumCells, 0.0f);

	for (int i = 0; i < m_iNumSpheres; i++) {
		float x = m_particlePos[i][0];
		float y = m_particlePos[i][1];
		float z = m_particlePos[i][2];

		x = clamp(x, h, (m_iCellX - 1) * h);
		y = clamp(y, h, (m_iCellY - 1) * h);
		z = clamp(z, h, (m_iCellZ - 1) * h);

		float x0 = floor((x - h2) * h1);
		float tx = ((x - h2) - x0 * h) * h1;
		float x1 = min(x0 + 1, float(m_iCellX - 2));

		float y0 = floor((y - h2) * h1);
		float ty = ((y - h2) - y0 * h) * h1;
		float y1 = min(y0 + 1, float(m_iCellY - 2));

		float z0 = floor((z - h2) * h1);
		float tz = ((z - h2) - z0 * h) * h1;
		float z1 = min(z0 + 1, float(m_iCellZ - 2));

		float sx = 1.0 - tx;
		float sy = 1.0 - ty;
		float sz = 1.0 - tz;

		if (x0 < m_iCellX && y0 < m_iCellY && z0 < m_iCellZ) d[x0*n1+y0*n2+z0] += sx * sy * sz;
		if (x1 < m_iCellX && y0 < m_iCellY && z0 < m_iCellZ) d[x1*n1+y0*n2+z0] += tx * sy * sz;
		if (x1 < m_iCellX && y1 < m_iCellY && z0 < m_iCellZ) d[x1*n1+y1*n2+z0] += tx * ty * sz;
		if (x0 < m_iCellX && y1 < m_iCellY && z0 < m_iCellZ) d[x0*n1+y1*n2+z0] += sx * ty * sz;
		if (x0 < m_iCellX && y0 < m_iCellY && z1 < m_iCellZ) d[x0*n1+y0*n2+z1] += sx * sy * tz;
		if (x1 < m_iCellX && y0 < m_iCellY && z1 < m_iCellZ) d[x1*n1+y0*n2+z1] += tx * sy * tz;
		if (x1 < m_iCellX && y1 < m_iCellY && z1 < m_iCellZ) d[x1*n1+y1*n2+z1] += tx * ty * tz;
		if (x0 < m_iCellX && y1 < m_iCellY && z1 < m_iCellZ) d[x0*n1+y1*n2+z1] += sx * ty * tz;

	}

	if (m_particleRestDensity == 0.0) {
		float sum = 0.0;
		int numFluidCells = 0;

		for (int i = 0; i < m_iNumCells; i++) {
			if (m_type[i] == FLUID_CELL) {
				sum += d[i];
				numFluidCells++;
			}
		}

		if (numFluidCells > 0)
			m_particleRestDensity = sum / numFluidCells;
	}
}

void FlipSimulator::solveIncompressibility(int numIters, float dt, float overRelaxation, bool compensateDrift)
{
	m_p.clear();  m_p.resize(m_iNumCells, 0.0);
	for (int i = 0; i < 2; i++)
		prevVel[i] = vel[i];

	int n1 = m_iCellY * m_iCellZ, n2 = m_iCellZ;
	float cp = m_particleRestDensity * m_h / dt;

	for (int i = 0; i < m_iNumCells; i++) {
		float u = vel[0][i];
		float v = vel[1][i];
	}

	for (int iter = 0; iter < numIters; iter++) 
	{

		for (int i = 1; i < m_iCellX - 1; i++) 
		{
			for (int j = 1; j < m_iCellY - 1; j++) 
			{
				for (int k = 1; k < m_iCellZ - 1; k++) 
				{
					if (m_type[i * n1 + j * n2 + k] != FLUID_CELL)
						continue;

					float center = i * n1 + j * n2 + k;
					float left = (i - 1) * n1 + j;
					float right = (i + 1) * n1 + j;
					float bottom = i * n1 + j - 1;
					float top = i * n1 + j + 1;

					float s = m_s[center];
					float sx0 = m_s[left];
					float sx1 = m_s[right];
					float sy0 = m_s[bottom];
					float sy1 = m_s[top];
					s = sx0 + sx1 + sy0 + sy1;
					if (s == 0.0)
						continue;

					float div = vel[0][right] - vel[0][center] +
						vel[1][top] - vel[1][center];

					if (m_particleRestDensity > 0.0 && compensateDrift) {
						float k = 1.0;
						float compression = m_particleDensity[i*n1+j*n2+k] - m_particleRestDensity;
						if (compression > 0.0)
							div = div - k * compression;
					}

					float p = -div / s;
					p *= overRelaxation;
					m_p[center] += cp * p;

					vel[0][center] -= sx0 * p;
					vel[0][right] += sx1 * p;
					vel[1][center] -= sy0 * p;
					vel[1][top] += sy1 * p;
				}
			}
		}
	}
}

void FlipSimulator::updateParticleColors()
{
	float h1 = m_fInvSpacing;

	for (int i = 0; i < m_iNumSpheres; i++) {

		float s = 0.01;

		m_particleColor[i][0] = clamp(m_particleColor[i][0] - s, 0.0, 1.0);
		m_particleColor[i][1] = clamp(m_particleColor[i][1] - s, 0.0, 1.0);
		m_particleColor[i][2] = clamp(m_particleColor[i][2] - s, 0.0, 1.0);

		float x = m_particlePos[i][0];
		float y = m_particlePos[i][1];
		float z = m_particlePos[i][2];
		
		float xi = clamp(floor(x * h1), 1, m_iCellX - 1);
		float yi = clamp(floor(y * h1), 1, m_iCellY - 1);
		float zi = clamp(floor(z * h1), 1, m_iCellZ - 1);

		float cellNr = xi * m_iCellY * m_iCellZ + yi * m_iCellZ + zi;

		float d0 = m_particleRestDensity;

		if (d0 > 0.0) {
			float relDensity = m_particleDensity[cellNr] / d0;
			if (relDensity < 0.7) {
				float s = 0.8;
				m_particleColor[i][0] = s;
				m_particleColor[i][1] = s;
				m_particleColor[i][2] = 1.0;
			}
		}
	}
}