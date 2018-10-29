// ArmDynamics.h

#ifndef _ARMDYNAMICS_h
#define _ARMDYNAMICS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <cmath>
#include <array>
using namespace std;

class ArmDynamics {

public:
	void get_Coriolis(const double theta[3], double B[9]);
	void get_Centrifugal(const double theta[3], double C[9]);
	void add_Inertia(const double theta[3], const double dd_theta[3], double tau[3]);
	void add_Velocity(const double theta[3], const double d_theta[3], double tau[3]);
	void add_Friction(const double d_theta[3], double tau[3]);
	void add_Gravity(const double theta[3], double tau[3]);
	array<double, 3> ComputeOutputTorque(array<double, 3> controlAccelerations, array<double, 3> thetaDesired, array<double, 3> dThetaDesired);

protected:
	static constexpr double g = 9.815;

	// Indexes
	static constexpr short i_11 = 0, i_12 = 1, i_13 = 2, i_21 = 3, i_22 = 4, i_23 = 5, i_31 = 6, i_32 = 7, i_33 = 8;

	static constexpr double l[3] = { -0.065, 0.22, 0.2779 }; // [m] Link lengths
	static constexpr double lc[3] = { -0.03167744, 0.15053580, 0.12966557 }; // [m] Lengths to CoM
	static constexpr double m[3] = { 0.18481027, 0.2148, 0.33003303 }; // [kg] Link mass
																	   // Inertia tensors [kg*m^2]
	static constexpr double I1[9] = { 0.00031543, 0, -0.00000002, 0, 0.00042781, 0.00000456, -0.00000002, 0.00000456, 0.00008106 };
	static constexpr double I2[9] = { 0.00623621, 0, -0.00001874, 0, 0.00769713, 0.00001319, -0.00001874, 0.00001319, 0.00006731 };
	static constexpr double I3[9] = { 0.00677702, 0, -0.00003893, 0, 0.00635728, 0, -0.00003893, 0, 0.00019629 };

};

#endif