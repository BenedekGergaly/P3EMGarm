#include "Utilities.h"


Utilities::Utilities()
{
}


Utilities::~Utilities()
{
}

void Utilities::LogArray(String text, array<double, 3> data)
{
	Serial.print(text);
	Serial.print(": ");
	Serial.print(data[0]);
	Serial.print("   ");
	Serial.print(data[1]);
	Serial.print("   ");
	Serial.println(data[2]);
}

void Utilities::Log(String text, double data)
{
	Serial.print(text);
	Serial.print(": ");
	Serial.println(data);
}

void Utilities::Pause()
{
	Serial.println("Paused. Send r to resume or x to reset.");
	while (!Serial.available());
	if (Serial.read() == 'x')
	{
		WRITE_RESTART(0x5FA0004);
	}
}

double Utilities::millisDouble()
{
	return (double)micros() / 1000;
}

double Utilities::secondsDouble()
{
	return millisDouble() / 1000;
}

Point3D<double> Utilities::ArrayToPoint(array<double, 3> a)
{
	return Point3D<double>(a[0], a[1], a[2]);
}

//Returns radians pr. sec
double Utilities::ConvertVelocitySignal(int16_t signal)
{
	return signal * VELOCITY_UNIT;
}

//Returns radians
double Utilities::ConvertPositionSignal(int16_t signal)
{
	return signal * POSITION_UNIT;
}

double Utilities::ComputeOutputPWM(double desiredTorque, ServoType servoType)
{
	switch (servoType)
	{
	case ServoType::MX64:
		return lround(216.85*desiredTorque);// +16;
	case ServoType::MX106:
		return lround(125.48*desiredTorque);// +18;
	default:
		return 0.0;
	}
}