// ArmControl.h

#ifndef _ARMCONTROL_h
#define _ARMCONTROL_h

#include <array>
#include "ArmDynamics.h"
#include "servo.h"
#include "Arduino.h"
#include "Utilities.h"

using namespace std;

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
	void resetIntegral();

	double Kp, Kv, Kpi;
	double lastTime = 0;
	array<double, 3> integralValues;
private:
	ArmDynamics dynamics;
	servo dxl;
	Utilities utilities;
};

#endif

