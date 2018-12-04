#include "ServoHelper.h"



ServoHelper::ServoHelper(servo &dynamixel)
{
	this->dynamixel = &dynamixel;
}

ServoHelper::~ServoHelper()
{
}

double ServoHelper::ReadPositionRad(int id) //includes angle compensation
{
	int32_t pos;
	dynamixel->readPosition(id, pos);
	double angle = utilities.ConvertPositionSignal(pos);
	if (angle > 2.2 && id != 1) return angle - 2 * PI;
	else return angle;
}

double ServoHelper::ReadVelocityRad(int id)
{
	int32_t vel;
	dynamixel->readVelocity(id, vel);
	if(vel)
	return utilities.ConvertVelocitySignal(vel);
}

array<double, 3> ServoHelper::ReadPositionRadArray()
{
	array<double, 3> output;
	for (int i = 0; i < 3; i++)
	{
		output[i] = ReadPositionRad(i + 1);
	}
	return output;
}

array<double, 3> ServoHelper::ReadVelocityRadArray()
{
	array<double, 3> output;
	for (int i = 0; i < 3; i++)
	{
		int32_t vel;
		dynamixel->readVelocity(i + 1, vel);
		output[i] = utilities.ConvertVelocitySignal(vel);
	}
	return output;
}

void ServoHelper::LEDsOn()
{
	for (int i = 0; i < 5; i++)
	{
		dynamixel->setLED(i + 1, 1);
	}
}

void ServoHelper::LEDsOff()
{
	for (int i = 0; i < 5; i++)
	{
		dynamixel->setLED(i + 1, 0);
	}
}

void ServoHelper::SendTorquesAllInOne(array<double, 3> torques)
{
	int stopFlag = false;
	for (int i = 0; i < 3; i++)
	{
		if (abs(torques[i]) > 30)
		{
			Serial.print("[ERROR] Control: Overtorque on servo #");
			Serial.println(i + 1);
			utilities.Log("Value", torques[i]);
			stopFlag = 1;
		}
	}
	if (stopFlag)
	{
		SoftEstop();
		utilities.Pause();
	}
	int pwm1 = utilities.ComputeOutputPWM(torques[0], ServoType::MX106);
	int pwm2 = utilities.ComputeOutputPWM(torques[1], ServoType::MX106);
	int pwm3 = utilities.ComputeOutputPWM(torques[2], ServoType::MX64);
	pwm1 = constrain(pwm1, -885, 885);
	pwm2 = constrain(pwm2, -885, 885);
	pwm3 = constrain(pwm3, -885, 885);
	dynamixel->setGoalPwm(1, pwm1);
	dynamixel->setGoalPwm(2, pwm2);
	dynamixel->setGoalPwm(3, pwm3);
}

bool ServoHelper::CheckOverspeed(double speedLimit)
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

void ServoHelper::SoftEstop()
{
	for (int j = 1; j <= 3; j++)
	{
		dynamixel->torqueEnable(j, 0);
		dynamixel->setOperatingMode(j, 3);
		dynamixel->torqueEnable(j, 1);
	}
	Serial.println("[INFO] Control: SoftEstop has been called");
	LEDsOn();
}