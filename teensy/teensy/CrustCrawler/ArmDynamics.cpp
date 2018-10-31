// 
// 
// 

#include "ArmDynamics.h"
void ArmDynamics::add_Gravity(const double theta[3], double tau[3]) {
	tau[0] += 0;
	//tau[1] += g * (l[1] * m[2] * sin(theta[1]) + lc[1] * m[1] * sin(theta[1]) + lc[2] * m[2] * sin(theta[1] + theta[2] - (M_PI / 2)));
	tau[1] += 4*g*(sin(theta[2])*cos(theta[1])*cos(theta[1])*cos(theta[2])*lc[1] * m[1] + sin(theta[1]) * (cos(theta[2]) * cos(theta[2]) * lc[1] * m[1] + (lc[0] * m[0]) / 2.0 - (lc[1] * m[1]) / 2.0)*cos(theta[1]) - (sin(theta[2]) * cos(theta[2]) * lc[1] * m[1]) / 2.0);
	//tau[2] += g * lc[2] * m[2] * sin(theta[1] + theta[2] - (M_PI / 2));
	tau[2] += 4 * lc[1] * m[1] * g*(cos(theta[1])*cos(theta[1])*cos(theta[2])*sin(theta[2]) + sin(theta[1])*(cos(theta[2])*cos(theta[2]) - 0.5) * cos(theta[1]) - (sin(theta[2]) * cos(theta[3])) / 2.0);
}

array<double, 3> ArmDynamics::ComputeOutputTorque(array<double, 3> controlAccelerations, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback)
{
	double tau[3] = { 0, 0, 0 };

	add_Inertia(thetaFeedback.data(), controlAccelerations.data(), tau);
	add_Velocity(thetaFeedback.data(), dThetaFeedback.data(), tau);
    add_Gravity(thetaFeedback.data(), tau);

	tau[2] *= 4;

	array<double, 3> returnArray = { tau[0], tau[1], tau[2] };
	return returnArray;
}

void ArmDynamics::get_Coriolis(const double theta[3], double B[9]) {
	const double B_temp = (I3[i_12] * cos(theta[0])*cos(theta[1])*cos(theta[2])) / 2 - (I3[i_11] * cos(theta[1])*cos(theta[2])*sin(theta[0])) / 2 + (I3[i_32] * cos(theta[0])*cos(theta[1])*sin(theta[2])) / 2 + (I3[i_32] * cos(theta[0])*cos(theta[2])*sin(theta[1])) / 2 - (I3[i_12] * cos(theta[0])*sin(theta[1])*sin(theta[2])) / 2 - (I3[i_31] * cos(theta[1])*sin(theta[0])*sin(theta[2])) / 2 - (I3[i_31] * cos(theta[2])*sin(theta[0])*sin(theta[1])) / 2 + (I3[i_11] * sin(theta[0])*sin(theta[1])*sin(theta[2])) / 2;
	const double B_temp2 = (I3[i_31] * cos(theta[0])*cos(theta[1])*cos(theta[2])) / 2 - (I3[i_11] * cos(theta[0])*cos(theta[1])*sin(theta[2])) / 2 - (I3[i_11] * cos(theta[0])*cos(theta[2])*sin(theta[1])) / 2 + (I3[i_32] * cos(theta[1])*cos(theta[2])*sin(theta[0])) / 2 - (I3[i_12] * cos(theta[1])*sin(theta[0])*sin(theta[2])) / 2 - (I3[i_12] * cos(theta[2])*sin(theta[0])*sin(theta[1])) / 2 - (I3[i_31] * cos(theta[0])*sin(theta[1])*sin(theta[2])) / 2 - (I3[i_32] * sin(theta[0])*sin(theta[1])*sin(theta[2])) / 2;

	B[i_11] = (I2[i_31] * cos(theta[0])*cos(theta[1])) / 2 - (I2[i_11] * cos(theta[0])*sin(theta[1])) / 2 + (I2[i_32] * cos(theta[1])*sin(theta[0])) / 2 - (I2[i_12] * sin(theta[0])*sin(theta[1])) / 2 + B_temp2;
	B[i_12] = B_temp2;
	B[i_13] = I3[i_21] * cos(theta[0]) + I3[i_22] * sin(theta[0]);

	B[i_21] = (I2[i_12] * cos(theta[0])*cos(theta[1])) / 2 - (I2[i_11] * cos(theta[1])*sin(theta[0])) / 2 + (I2[i_32] * cos(theta[0])*sin(theta[1])) / 2 - (I2[i_31] * sin(theta[0])*sin(theta[1])) / 2 + B_temp;
	B[i_22] = B_temp;
	B[i_23] = 0;

	B[i_31] = B_temp;
	B[i_32] = B_temp;
	B[i_33] = l[1] * lc[2] * m[2] * sin(theta[2]);
}

void ArmDynamics::get_Centrifugal(const double theta[3], double C[9]) {
	C[i_11] = 0;
	C[i_12] = (I2[i_21] * cos(theta[0])) / 2 + (I3[i_21] * cos(theta[0])) / 2 + (I2[i_22] * sin(theta[0])) / 2 + (I3[i_22] * sin(theta[0])) / 2;
	C[i_13] = (I3[i_21] * cos(theta[0])) / 2 + (I3[i_22] * sin(theta[0])) / 2;

	C[i_21] = (I3[i_13] * cos(theta[1] + theta[2])) / 2 + (I3[i_33] * sin(theta[1] + theta[2])) / 2 + (I2[i_13] * cos(theta[1])) / 2 + (I2[i_33] * sin(theta[1])) / 2 - (pow(lc[2], 2)*m[2] * sin(2 * theta[1] + 2 * theta[2])) / 2 - (pow(l[1], 2)*m[2] * sin(2 * theta[1])) / 2 - (pow(lc[1], 2)*m[1] * sin(2 * theta[1])) / 2 - l[1] * lc[2] * m[2] * sin(2 * theta[1] + theta[2]);
	C[i_22] = 0;
	C[i_23] = 0;

	C[i_31] = (I3[i_13] * cos(theta[1] + theta[2])) / 2 + (I3[i_33] * sin(theta[1] + theta[2])) / 2 - (pow(lc[2], 2)*m[2] * sin(2 * theta[1] + 2 * theta[2])) / 2 + (l[1] * lc[2] * m[2] * sin(theta[2])) / 2 - (l[1] * lc[2] * m[2] * sin(2 * theta[1] + theta[2])) / 2;
	C[i_32] = l[1] * lc[2] * m[2] * sin(theta[2]);
	C[i_33] = 0;
}

void ArmDynamics::add_Inertia(const double theta[3], const double dd_theta[3], double tau[3]) {
	const double M[9] = {
		I1[i_33] + I3[i_33] * cos(theta[1] + theta[2]) - I3[i_13] * sin(theta[1] + theta[2]) + (pow(l[1],2)*m[2]) / 2 + (pow(lc[1],2)*m[1]) / 2 + (pow(lc[2],2)*m[2]) / 2 + I2[i_33] * cos(theta[1]) - I2[i_13] * sin(theta[1]) - (pow(lc[2],2)*m[2] * cos(2 * theta[1] + 2 * theta[2])) / 2 - (pow(l[1],2)*m[2] * cos(2 * theta[1])) / 2 - (pow(lc[1],2)*m[1] * cos(2 * theta[1])) / 2 + l[1] * lc[2] * m[2] * cos(theta[2]) - l[1] * lc[2] * m[2] * cos(2 * theta[1] + theta[2]),
		I2[i_23] / 2 + I3[i_23] / 2 + (I2[i_32] * cos(theta[0])*cos(theta[1])) / 2 - (I2[i_12] * cos(theta[0])*sin(theta[1])) / 2 - (I2[i_31] * cos(theta[1])*sin(theta[0])) / 2 + (I2[i_11] * sin(theta[0])*sin(theta[1])) / 2 + (I3[i_32] * cos(theta[0])*cos(theta[1])*cos(theta[2])) / 2 - (I3[i_12] * cos(theta[0])*cos(theta[1])*sin(theta[2])) / 2 - (I3[i_12] * cos(theta[0])*cos(theta[2])*sin(theta[1])) / 2 - (I3[i_31] * cos(theta[1])*cos(theta[2])*sin(theta[0])) / 2 + (I3[i_11] * cos(theta[1])*sin(theta[0])*sin(theta[2])) / 2 + (I3[i_11] * cos(theta[2])*sin(theta[0])*sin(theta[1])) / 2 - (I3[i_32] * cos(theta[0])*sin(theta[1])*sin(theta[2])) / 2 + (I3[i_31] * sin(theta[0])*sin(theta[1])*sin(theta[2])) / 2,
		I3[i_23] / 2 + (I3[i_32] * cos(theta[0])*cos(theta[1])*cos(theta[2])) / 2 - (I3[i_12] * cos(theta[0])*cos(theta[1])*sin(theta[2])) / 2 - (I3[i_12] * cos(theta[0])*cos(theta[2])*sin(theta[1])) / 2 - (I3[i_31] * cos(theta[1])*cos(theta[2])*sin(theta[0])) / 2 + (I3[i_11] * cos(theta[1])*sin(theta[0])*sin(theta[2])) / 2 + (I3[i_11] * cos(theta[2])*sin(theta[0])*sin(theta[1])) / 2 - (I3[i_32] * cos(theta[0])*sin(theta[1])*sin(theta[2])) / 2 + (I3[i_31] * sin(theta[0])*sin(theta[1])*sin(theta[2])) / 2,


		0,
		m[2] * pow(l[1],2) + 2 * m[2] * cos(theta[2])*l[1] * lc[2] + m[1] * pow(lc[1],2) + m[2] * pow(lc[2],2) + I2[i_22] * cos(theta[0]) + I3[i_22] * cos(theta[0]) - I2[i_21] * sin(theta[0]) - I3[i_21] * sin(theta[0]),
		m[2] * pow(lc[2],2) + l[1] * m[2] * cos(theta[2])*lc[2] + I3[i_22] * cos(theta[0]) - I3[i_21] * sin(theta[0]),

		0,
		0,
		m[2] * pow(lc[2],2) + I3[i_22] * cos(theta[0]) - I3[i_21] * sin(theta[0])
	};

	tau[0] += M[i_11] * dd_theta[0] + M[i_12] * dd_theta[1] + M[i_13] * dd_theta[2];
	tau[1] += M[i_12] * dd_theta[0] + M[i_22] * dd_theta[1] + M[i_23] * dd_theta[2];
	tau[2] += M[i_13] * dd_theta[0] + M[i_23] * dd_theta[1] + M[i_33] * dd_theta[2];
}

void ArmDynamics::add_Velocity(const double theta[3], const double d_theta[3], double tau[3]) {
	double B[9], C[9];
	get_Coriolis(theta, B);
	get_Centrifugal(theta, C);

	// V = B*[v1*v2; v1*v3; v2*v3] + C*[v1^2; v2^2; v3^2]
	const double v1v2 = d_theta[0] * d_theta[1];
	const double v1v3 = d_theta[0] * d_theta[2];
	const double v2v3 = d_theta[1] * d_theta[2];

	const double v1v1 = d_theta[0] * d_theta[0];
	const double v2v2 = d_theta[1] * d_theta[1];
	const double v3v3 = d_theta[2] * d_theta[2];

	// add Coriolis terms
	tau[0] += B[i_11] * v1v2 + B[i_12] * v1v3 + B[i_13] * v2v3;
	tau[1] += B[i_21] * v1v2 + B[i_22] * v1v3 + B[i_23] * v2v3;
	tau[2] += B[i_31] * v1v2 + B[i_32] * v1v3 + B[i_33] * v2v3;

	// add Centrifugal terms
	tau[0] += C[i_11] * v1v1 + C[i_12] * v2v2 + C[i_13] * v3v3;
	tau[1] += C[i_21] * v1v1 + C[i_22] * v2v2 + C[i_23] * v3v3;
	tau[2] += C[i_31] * v1v1 + C[i_32] * v2v2 + C[i_33] * v3v3;

	// add Friction
	add_Friction(d_theta, tau);
}

void ArmDynamics::add_Friction(const double d_theta[3], double tau[3]) {
	for (int i = 0; i < 3; ++i) {
		if (d_theta[i] != 0)
			tau[i] += copysign(0.08, d_theta[i]);
	}
}