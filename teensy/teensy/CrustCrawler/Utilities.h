#pragma once

#include <array>
#include <cmath>
#include <Arduino.h>
#include "Models/Point3D.h"
#include "servo.h"
#define RESTART_ADDR       0xE000ED0C
#define READ_RESTART()     (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

enum class ServoType { MX28, MX64, MX106 };

class Utilities
{
public:
	Utilities();
	~Utilities();

	void LogArray(String text, array<double, 3> data);
	void Log(String text, double data);
	void Pause();

	Point3D<double> ArrayToPoint(array<double, 3> a);
	double millisDouble();
	double secondsDouble();

	double ConvertVelocitySignal(int16_t signal);
	double ConvertPositionSignal(int16_t signal);
	double ComputeOutputPWM(double desiredTorque, ServoType servoType);

private:
	const double VELOCITY_UNIT = 0.023980823895; //radian/second pr. value
	const double POSITION_UNIT = 0.0015358897; //Radians pr. value
};

