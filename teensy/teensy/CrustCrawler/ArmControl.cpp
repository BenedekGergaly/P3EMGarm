// 
// 
// 

#include "ArmControl.h"

ArmControl::ArmControl()
{
}

ArmControl::~ArmControl()
{
}

array<double, 3> ArmControl::ComputeControlTorque(array<double, 3> thetaDesired, array<double, 3> dThetaDesired, 
	array<double, 3> ddThetaDesired, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback)
{
	array<double, 3> computedAcceleration;
	for (int i = 0; i < 3; i++)
	{
		computedAcceleration[i] = (thetaDesired[i] - thetaFeedback[i]) * KP + (dThetaDesired[i] - dThetaFeedback[i]) * KV + ddThetaDesired[i];
	}

	//Some stuff here from the dynamics calculating the torque
	array<double, 3> torque = { 0, 0, 0 }; //TODO: Provide method here

	return torque;
}

double ArmControl::ComputeOutputCurrent(double desiredTorque, ServoType servoType)
{
	switch (servoType)
	{
	case ServoType::MX28:
		return desiredTorque * K_MX28;
	case ServoType::MX64:
		return desiredTorque * K_MX64;
	case ServoType::MX106
		return desiredTorque * K_MX128;
	default:
		return 0.0;
	}
}
