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
	static int stickyFrictionCounter = 0;
	if (lastTime == 0) lastTime = utilities.secondsDouble();
	array<double, 3> computedAcceleration;
	for (int i = 0; i < 3; i++)
	{
		double positionError = (thetaDesired[i] - thetaFeedback[i]);
		integralValues[i] += positionError * (utilities.secondsDouble() - lastTime);
		computedAcceleration[i] = positionError * Kp + integralValues[i] * Kpi + (dThetaDesired[i] - dThetaFeedback[i]) * Kv + ddThetaDesired[i];
		//wip sticky friction workaround
		if (dThetaFeedback[i] == 0 && abs(positionError) > 0.1)
		{
			if (stickyFrictionCounter == 5) {
				computedAcceleration[i] *= 1; //utilities.Pause();
			}
			else stickyFrictionCounter++;
		}
		else stickyFrictionCounter = 0;
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
