// ArmControl.h

#ifndef _ARMCONTROL_h
#define _ARMCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <array>
#include "ArmDynamics.h"
using namespace std;

enum class ServoType { MX28, MX64, MX106  };

class ArmControl
{
public:
	ArmControl();
	~ArmControl();
	array<double, 3> ComputeControlTorque(array<double, 3> thetaDesired,
		array<double, 3> dThetaDesired,
		array<double, 3> ddThetaDesired,
		array<double, 3> thetaFeedback,
		array<double, 3> dThetaFeedback);
	array<double, 3> ComputeControlTorque(array<double, 3> thetaDesired, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback);
	double ComputeOutputCurrent(double desiredTorque, ServoType servoType);
	int16_t ConvertCurrentToSignalValue(double current);
	double ConvertVelocitySignal(int16_t signal);
	double ConvertPositionSignal(int16_t signal);

private:
	const double KV = 6.5;
	const double KP = 10.56;
	const double K_MX28 = 0.76724;
	const double K_MX64 = 0.85185;
	const double K_MX106 = 0.72165;
	const double SUPPLIED_CURRENT_UNIT = 3.36; //mA
	const double VELOCITY_UNIT = 0.023980823895; //radian/second pr. value
	const double POSITION_UNIT = 0.0015358897; //Radians pr. value

	ArmDynamics dynamics;

private:
};

#endif

