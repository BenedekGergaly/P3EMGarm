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
		computedAcceleration[i] = (thetaDesired[i] - thetaFeedback[i]) * kpTemp + (dThetaDesired[i] - dThetaFeedback[i]) * kvTemp + ddThetaDesired[i];
	}
	LogArray("computed acceleration", computedAcceleration);
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
	return value; // We want torque the other way - NO
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

double ArmControl::ReadPositionRad(int id)
{
	int32_t pos;
	dxl.readPosition(id, pos);
	return ConvertPositionSignal(pos);
}

double ArmControl::ReadVelocityRad(int id)
{
	int32_t vel;
	dxl.readVelocity(id, vel);
	return ConvertVelocitySignal(vel);
}

array<double, 3> ArmControl::ReadPositionRadArray()
{
	array<double, 3> output;
	for (int i = 0; i < 3; i++)
	{
		int32_t pos;
		dxl.readPosition(i+1, pos);
		output[i] = ConvertPositionSignal(pos);
	}
	return output;
}

array<double, 3> ArmControl::ReadVelocityRadArray()
{
	array<double, 3> output;
	for (int i = 0; i < 3; i++)
	{
		int32_t vel;
		dxl.readVelocity(i+1, vel);
		output[i] = ConvertVelocitySignal(vel);
	}
	return output;
}

void ArmControl::SendTorquesAllInOne(array<double, 3> torques)
{
	double current1 = ComputeOutputCurrent(torques[0], ServoType::MX106);
	double current2 = ComputeOutputCurrent(torques[1], ServoType::MX106);
	double current3 = ComputeOutputCurrent(torques[2], ServoType::MX64);
	int16_t signal1 = ConvertCurrentToSignalValue(current1);
	int16_t signal2 = ConvertCurrentToSignalValue(current2);
	int16_t signal3 = ConvertCurrentToSignalValue(current3);
	dxl.setGoalCurrent(1, signal1);
	dxl.setGoalCurrent(2, signal2);
	dxl.setGoalCurrent(3, signal3);
}

bool ArmControl::CheckOverspeed(double speedLimit)
{
	bool triggered = 0;
	for (int i = 1; i < 4; i++)
	{
		if (ReadVelocityRad(i) > speedLimit)
		{
			Serial.print("WARNING: Overspeed servo #");
			Serial.println(i);
			Serial.println("Stopping all!");
			triggered = 1;
		}
	}
	if (triggered)
	{
		SoftEstop();
	}
	return triggered;
}

void ArmControl::SoftEstop()
{
	for (int j = 1; j < 4; j++)
	{
		dxl.torqueEnable(j, 0);
		dxl.setOperatingMode(j, 3);
		dxl.torqueEnable(j, 1);
	}
	Serial.println("[INFO] Control: SoftEstop has been called");
}

void ArmControl::LogArray(String text, array<double, 3> data)
{
	Serial.print(text);
	Serial.print(": ");
	Serial.print(data[0]);
	Serial.print("   ");
	Serial.print(data[1]);
	Serial.print("   ");
	Serial.println(data[2]);
}
