#include "ArmControl.h"

ArmControl::ArmControl()
{
	integralValues = { 0.0, 0.0, 0.0 };
	servoHelper = new ServoHelper(dxl);
}

ArmControl::~ArmControl()
{
}

array<double, 3> ArmControl::ComputeControlTorque(array<double, 3> thetaDesired, array<double, 3> dThetaDesired, 
	array<double, 3> ddThetaDesired, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback)
{
	static array<int, 3> stickyFrictionCounter = {0,0,0};
	if (lastTime == 0) lastTime = utilities.secondsDouble();
	array<double, 3> computedAcceleration;
	for (int i = 0; i < 3; i++)
	{
		double positionError = (thetaDesired[i] - thetaFeedback[i]);
		integralValues[i] += positionError * (utilities.secondsDouble() - lastTime);
		computedAcceleration[i] = positionError * Kp[i] + integralValues[i] * Ki[i] + (dThetaDesired[i] - dThetaFeedback[i]) * Kv[i] + ddThetaDesired[i];
		//wip sticky friction workaround
		//joint 2 0.8Nm, joint 3 around 0.6Nm
		if (abs(dThetaFeedback[i]) < 0.02 && abs(positionError) > 0.05)
		{
			if (stickyFrictionCounter[i] == 5) 
			{
				computedAcceleration[i] *= 1;
			}
			else stickyFrictionCounter[i] += 1;
		}
		else stickyFrictionCounter[i] = 0;
	}
	//utilities.LogArray("computed acc", computedAcceleration);
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
