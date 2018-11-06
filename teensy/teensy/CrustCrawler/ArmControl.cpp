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
	array<double, 3> torque = dynamics.ComputeOutputTorque(computedAcceleration, thetaFeedback, dThetaFeedback); 

	return torque;
}

//Moves with maximum acceleration and velocity
array<double, 3> ArmControl::ComputeControlTorque(array<double, 3> thetaDesired, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback)
{
	array<double, 3> acceleration = { 0,0,0 };
	array<double, 3> velocity = { 0,0,0 };
	return ComputeControlTorque(thetaDesired, acceleration, velocity, thetaFeedback, dThetaFeedback);
}

double ArmControl::ComputeOutputCurrent(double desiredTorque, ServoType servoType)
{
	switch (servoType)
	{
	case ServoType::MX28:
		return desiredTorque * K_MX28;
	case ServoType::MX64:
		return desiredTorque * K_MX64;
	case ServoType::MX106:
		return desiredTorque * K_MX106;
	default:
		return 0.0;
	}
}

int16_t ArmControl::ConvertCurrentToSignalValue(double currentInAmps)
{
	double currentInMilliAmps = currentInAmps * 1000.0;
	int16_t value = (int16_t)roundf(currentInMilliAmps / (SUPPLIED_CURRENT_UNIT));
	return -value; // We want torque the other way
}

//Returns radians pr. sec
double ArmControl::ConvertVelocitySignal(int16_t signal)
{
	return signal * VELOCITY_UNIT;
}

//Returns radians
double ArmControl::ConvertPositionSignal(int16_t signal)
{
	return signal * POSITION_UNIT;
}
