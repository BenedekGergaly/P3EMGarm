#include "Utilities.h"

Utilities::Utilities()
{
}


Utilities::~Utilities()
{
}

// Logs an array into the serial monitor
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

// Logs to serial
void Utilities::Log(String text, double data)
{
	Serial.print(text);
	Serial.print(": ");
	Serial.println(data);
}

//halts program, be sure to call softEstop before or it might fall/smash
void Utilities::Pause()
{
	bool f = 1;
	Serial.println("Paused. Send r to resume or x to reset.");
	while (f)
	{
		while (!Serial.available());
		if (Serial.peek() == 'x')
		{
			WRITE_RESTART(0x5FA0004); //sets reset bit on teensy, usb reconnect required afterwards
		}
		else if (Serial.peek() == 'r')
		{
			f = 0;
		}
		else
		{
			Serial.read();
		}
	}
}

double Utilities::millisDouble()
{
	return (double)micros() / 1000.0;
}

double Utilities::secondsDouble()
{
	return millisDouble() / 1000.0;
}

Point3D<double> Utilities::ArrayToPoint(array<double, 3> a)
{
	return Point3D<double>(a[0], a[1], a[2]);
}

//Returns radians per sec
double Utilities::ConvertVelocitySignal(int16_t signal)
{
	return signal * VELOCITY_UNIT;
}

//Returns radians
double Utilities::ConvertPositionSignal(int16_t signal)
{
	return signal * POSITION_UNIT;
}

//Computes the PWM values to obtain required torque. Calculated from experiments with the robot.
double Utilities::ComputeOutputPWM(double desiredTorque, ServoType servoType)
{
	switch (servoType)
	{
		case ServoType::MX64:
			if (desiredTorque >= 0)
			{
				return lround(209.4336*desiredTorque);
			}
			else
			{
				return lround(196.95*desiredTorque);
			}
		case ServoType::MX106:
			if (desiredTorque >= 0)
			{
				return lround(123.6096*desiredTorque);
			}
			else
			{
				return lround(131.4082*desiredTorque);
			}
		default:
			return 0.0;
	}
}
