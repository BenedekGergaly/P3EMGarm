#include "ArmControl.h"

ArmControl::ArmControl()
{
	integralValues = { 0.0, 0.0, 0.0 };
}

ArmControl::~ArmControl()
{
}

array<double, 3> ArmControl::ComputeControlTorque(array<double, 3> thetaDesired, array<double, 3> dThetaDesired, 
	array<double, 3> ddThetaDesired, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback)
{
	if (lastTime == 0) lastTime = utilities.secondsDouble();
	array<double, 3> computedAcceleration;
	for (int i = 0; i < 3; i++)
	{
		double positionError = (thetaDesired[i] - thetaFeedback[i]);
		integralValues[i] += positionError * (utilities.secondsDouble() - lastTime);
		computedAcceleration[i] = positionError * Kp + integralValues[i] * Kpi + (dThetaDesired[i] - dThetaFeedback[i]) * Kv + ddThetaDesired[i];
	}
	utilities.LogArray("computed acc", computedAcceleration);
	//Some stuff here from the dynamics calculating the torque
	array<double, 3> torque = dynamics.ComputeOutputTorque(computedAcceleration, thetaFeedback, dThetaFeedback); 
	lastTime = utilities.secondsDouble();

	return torque;
}

//Moves with maximum acceleration and velocity
array<double, 3> ArmControl::ComputeControlTorque(array<double, 3> thetaDesired, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback)
{
	array<double, 3> acceleration = { 0,0,0 };
	array<double, 3> velocity = { 0,0,0 };
	return ComputeControlTorque(thetaDesired, acceleration, velocity, thetaFeedback, dThetaFeedback);
}

void ArmControl::resetIntegral()
{
	integralValues = { 0,0,0 };
}
