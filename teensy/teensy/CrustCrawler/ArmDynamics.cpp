// 
// 
// 

#include "ArmDynamics.h"
void ArmDynamics::addGravity(const double theta[3], double tau[3]) {
	tau[0] += 0;
	tau[1] += g * (-sin(theta[1])*l[1] * m[2] - sin(theta[1]) * lc[1] * m[1] - sin(theta[1] + theta[2]) * lc[2] * m[2]);
	tau[2] += g * lc[2] * m[2] * -sin(theta[1] + theta[2]);
}

array<double, 3> ArmDynamics::ComputeOutputTorque(array<double, 3> controlAccelerations, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback)
{
	double tau[3] = { 0, 0, 0 };

	addInertia(thetaFeedback.data(), controlAccelerations.data(), tau);
	//addVelocity(thetaFeedback.data(), dThetaFeedback.data(), tau);
    addGravity(thetaFeedback.data(), tau);
	//tau[2] *= 4;

	array<double, 3> returnArray = { tau[0], tau[1], tau[2]};
	return returnArray;
}

void ArmDynamics::getCoriolis(const double theta[3], double B[9]) {
	const double temp1 = (2 * I3[i_12] + 2 * I3[i_21]) * cos(theta[0]) * cos(theta[0]);
	const double temp2 = 2 * sin(theta[0]) * cos(theta[0]) * (I3[i_11] - I3[i_22]) - I3[i_12] - I3[i_21];
	const double temp3 = (-2 * I3[i_12] - 2 * I3[i_21]) * cos(theta[0]) * cos(theta[0]) + 2 * sin(theta[0]) * cos(theta[0]) * (I3[i_11] - I3[i_22]) + I3[i_12] + I3[i_21];

	B[i_11] = 4 * sin(theta[2]) * m[2] * lc[2] * (cos(theta[2]) * lc[2] + l[1]) * cos(theta[1]) * cos(theta[1]) + 4 * sin(theta[1]) * (cos(theta[2]) * cos(theta[2]) * m[2] * lc[2] * lc[2] + cos(theta[2]) * l[1] * m[2] * lc[2] + ((l[1] * l[1]) / 2.0 - (lc[2] * lc[2]) / 2.0) * m[2] + (m[1] * lc[1] * lc[1]) / 2.0) * cos(theta[1]) - 2 * sin(theta[2]) * m[2] * lc[2] * (cos(theta[2]) * lc[2] + l[1]);
	B[i_12] = 2 * m[2] * lc[2] * (sin(theta[2]) * (2 * cos(theta[2]) * lc[2] + l[1]) * cos(theta[1]) * cos(theta[1]) + sin(theta[1]) * (2 * lc[2] * cos(theta[2]) * cos(theta[2]) + l[1] * cos(theta[2]) - lc[2]) * cos(theta[1]) - sin(theta[2]) * (cos(theta[2]) * lc[2] + l[1]));
	B[i_13] = temp1 - temp2;

	B[i_21] = (-2 * I2[i_12] - 2 * I2[i_21] - 2 * I3[i_12] - 2 * I3[i_21]) * cos(theta[1]) * cos(theta[1]) + 2 * sin(theta[0]) * cos(theta[0]) * (I2[i_11] - I2[i_22] + I3[i_11] - I3[i_22]) + I2[i_12] + I2[i_21] + I3[i_12] + I3[i_21];
	B[i_22] = temp1 + temp2;
	B[i_23] = - 2 * m[2] * lc[2] * l[1] * sin(theta[2]);

	B[i_31] = temp3;
	B[i_32] = temp3;
	B[i_33] = 0;
}

void ArmDynamics::getCentrifugal(const double theta[3], double C[9]) {
	C[i_11] = 0;
	C[i_12] = ((2 * I2[i_12] + 2 * I2[i_21] + 2 * I3[i_12] + 2 * I3[i_21]) * cos(theta[0]) * cos(theta[0])) / 2.0 - sin(theta[0]) * cos(theta[0]) * (I2[i_11] - I2[i_22] + I3[i_11] - I3[i_22]) - I2[i_12] / 2.0 - I2[i_21] / 2.0 - I3[i_12] / 2.0 - I3[i_21] / 2.0;
	C[i_13] = ((2 * I3[i_12] + 2 * I3[i_21]) * cos(theta[0]) * cos(theta[0])) / 2.0 - sin(theta[0]) * cos(theta[0]) * (I3[i_11] - I3[i_22]) - I3[i_12] / 2.0 - I3[i_21] / 2.0;

	C[i_21] = -2 * sin(theta[2]) * m[2] * lc[2] * (cos(theta[2]) * lc[2] + l[1]) * cos(theta[1]) * cos(theta[1]) - (2 * cos(theta[2]) * cos(theta[2]) * m[2] * lc[2] * lc[2] + 2 * cos(theta[2]) * l[1] * m[2] * lc[2] + (l[1] * l[1] - lc[2] * lc[2]) * m[2] + m[1] * lc[1] * lc[1]) * sin(theta[1]) * cos(theta[1])
			  + ((I2[i_13] + I2[i_31] + I3[i_13] + I3[i_31]) * cos(theta[0])) / 2.0 + m[2] * lc[2] * lc[2] * sin(theta[2]) * cos(theta[2]) + ((I2[i_32] + I2[i_23] + I3[i_32] + I3[i_23]) * sin(theta[0])) / 2.0 + m[2] * lc[2] * sin(theta[2]) * l[1];
	C[i_22] = 0;
	C[i_23] = 0;

	C[i_31] = -2 * (cos(theta[2]) * lc[2] + l[1] / 2.0) * m[2] * sin(theta[2]) * lc[2] * cos(theta[1]) * cos(theta[1]) - 2 * sin(theta[1]) * cos(theta[1]) * m[2] * lc[2] * (lc[2] * cos(theta[2]) * cos(theta[2]) + (l[1] * cos(theta[2])) / 2.0 - lc[2] / 2.0) + m[2] * lc[2] * lc[2] * sin(theta[2]) * cos(theta[2])
		      + m[2] * lc[2] * sin(theta[2]) * l[1] + ((I3[i_13] + I3[i_31]) * cos(theta[0])) / 2.0 + ((I3[i_23] + I3[i_32]) * sin(theta[0])) / 2.0;
	C[i_32] = l[1] * lc[2] * m[2] * sin(theta[2]);
	C[i_33] = 0;
}

void ArmDynamics::addInertia(const double theta[3], const double ddTheta[3], double tau[3]) {
	const double temp1 = ((-I2[i_23] - I2[i_32] - I3[i_23] - I3[i_32]) * cos(theta[0])) / 2.0 + ((I2[i_13] + I2[i_31] + I3[i_13] + I3[i_31]) * sin(theta[0])) / 2.0;
	const double temp2 = ((-I3[i_23] - I3[i_32]) * cos(theta[0])) / 2.0 + ((I3[i_13] + I3[i_31]) * sin(theta[0])) / 2.0;
	const double temp3 = (-I3[i_11] + I3[i_22]) * cos(theta[0]) * cos(theta[0]) - sin(theta[0]) * cos(theta[0]) * (I3[i_12] + I3[i_21]) + m[2] * lc[2] * lc[2] + I3[i_11];

	const double M[9] = {
		(-2.0 * m[2] * lc[2] * lc[2] * cos(theta[2]) * cos(theta[2]) - 2.0 * m[2] * l[1] * lc[2] * cos(theta[2]) + (-(l[1] * l[1]) + lc[2] * lc[2]) * m[2] - m[1] * lc[1] * lc[1]) * cos(theta[1]) * cos(theta[1]) + 2 * sin(theta[1])*sin(theta[2]) * m[2] * lc[2] * (cos(theta[2]) * lc[2] + l[1]) * cos(theta[1]) + m[2] * lc[2] * lc[2] * cos(theta[2]) * cos(theta[2]) + 2 * m[2] * l[1] * lc[2] * cos(theta[2]) + m[2] * l[1] * l[1] + m[1] * lc[1] * lc[1] + I1[i_33] + I2[i_33] + I3[i_33],
		temp1,
		temp2,


		temp1,
		(I2[i_22] - I3[i_11] + I3[i_22] - I2[i_11]) * cos(theta[0]) * cos(theta[0]) - (I2[i_12] + I2[i_21] + I3[i_12] + I3[i_21]) * sin(theta[0]) * cos(theta[0]) + 2 * m[2] * l[1] * lc[2] * cos(theta[2]) + (l[1] * l[1] + lc[2] * lc[2]) * m[2] + m[1] * lc[1] * lc[1] + I3[i_11] + I2[i_11],
		(-I3[i_11] + I3[i_22]) * cos(theta[0]) * cos(theta[0]) - sin(theta[0]) * cos(theta[0]) * (I3[i_12] + I3[i_21]) + m[2] * l[1] * lc[2] * cos(theta[2]) + m[2] * lc[2] * lc[2] + I3[i_11],

		temp2,
		temp3 + m[2] * l[1] * lc[2] * cos(theta[2]),
		temp3
	};

	tau[0] += M[i_11] * ddTheta[0] + M[i_12] * ddTheta[1] + M[i_13] * ddTheta[2];
	tau[1] += M[i_12] * ddTheta[0] + M[i_22] * ddTheta[1] + M[i_23] * ddTheta[2];
	tau[2] += M[i_13] * ddTheta[0] + M[i_23] * ddTheta[1] + M[i_33] * ddTheta[2];
}

void ArmDynamics::addVelocity(const double theta[3], const double dTheta[3], double tau[3]) {
	double B[9], C[9];
	getCoriolis(theta, B);
	getCentrifugal(theta, C);

	// V = B*[v1*v2; v1*v3; v2*v3] + C*[v1^2; v2^2; v3^2]
	const double v1v2 = dTheta[0] * dTheta[1];
	const double v1v3 = dTheta[0] * dTheta[2];
	const double v2v3 = dTheta[1] * dTheta[2];

	const double v1v1 = dTheta[0] * dTheta[0];
	const double v2v2 = dTheta[1] * dTheta[1];
	const double v3v3 = dTheta[2] * dTheta[2];

	// add Coriolis terms
	tau[0] += B[i_11] * v1v2 + B[i_12] * v1v3 + B[i_13] * v2v3;
	tau[1] += B[i_21] * v1v2 + B[i_22] * v1v3 + B[i_23] * v2v3;
	tau[2] += B[i_31] * v1v2 + B[i_32] * v1v3 + B[i_33] * v2v3;

	// add Centrifugal terms
	tau[0] += C[i_11] * v1v1 + C[i_12] * v2v2 + C[i_13] * v3v3;
	tau[1] += C[i_21] * v1v1 + C[i_22] * v2v2 + C[i_23] * v3v3;
	tau[2] += C[i_31] * v1v1 + C[i_32] * v2v2 + C[i_33] * v3v3;

	// add Friction
	addFriction(dTheta, tau);
}

void ArmDynamics::addFriction(const double dTheta[3], double tau[3]) {
	for (int i = 0; i < 3; ++i) {
		if (dTheta[i] != 0)
			tau[i] += copysign(0.08, dTheta[i]);
	}
}