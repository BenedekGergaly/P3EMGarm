// ArmControl.h

#ifndef _ARMCONTROL_h
#define _ARMCONTROL_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <array>
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
	double ComputeOutputCurrent(double desiredTorque, ServoType servoType);

private:
	const double KV = 6.5;
	const double KP = 10.56;
	const double K_MX28 = 0.76724;
	const double K_MX64 = 0.85185;
	const double K_MX128 = 0.72165;

private:
};

#endif

