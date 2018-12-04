// ArmDynamics.h

#ifndef _ARMDYNAMICS_h
#define _ARMDYNAMICS_h


#include <cmath>
#include <array>
using namespace std;

class ArmDynamics {
public:
	array<double, 3> ComputeOutputTorque(array<double, 3> controlAccelerations, array<double, 3> thetaDesired, array<double, 3> dThetaDesired);

private:
	const double g = 9.815;

	// Indexes
	const short i_11 = 0, i_12 = 1, i_13 = 2, i_21 = 3, i_22 = 4, i_23 = 5, i_31 = 6, i_32 = 7, i_33 = 8;

	const double l[3] = { 0.067, 0.224, 0.149 }; // [m] Link lengths
	const double lc[3] = { 0.04, 0.17670, 0.13468 }; // [m] Lengths to CoM
	const double m[3] = { 0.1956, 0.227, 0.285 }; // [kg] Link mass
	// Inertia tensors [kg*m^2]
	const double I1[9] = { 0.00039968, 0.00000007, -0.00000547, 0.00000007, 0.00037926, -0.00000609, -0.00000547, -0.00000609, 0.00005511 };
	const double I2[9] = { 0.00005997, -0.00014512, 0.00002275, -0.00014512, 0.00761485,  -0.00000052, 0.00002275, -0.00000052, 0.00760132 };
	const double I3[9] = { 0.00019476, -0.00014581, 0.00003861, -0.00014581, 0.00612172, 0.00000075, 0.00003861, -0.00000075, 0.00601887 };
	void getCoriolis(const double theta[3], double B[9]);
	void getCentrifugal(const double theta[3], double C[9]);
	void addInertia(const double theta[3], const double ddTheta[3], double tau[3]);
	void addVelocity(const double theta[3], const double dTheta[3], double tau[3]);
	void addFriction(const double dTheta[3], double tau[3]);
	void addGravity(const double theta[3], double tau[3]);
};

#endif