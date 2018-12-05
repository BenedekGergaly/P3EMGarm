// ArmControl.h

#ifndef _ARMCONTROL_h
#define _ARMCONTROL_h

#include <array>
#include "ArmDynamics.h"
#include "servo.h"
#include "Arduino.h"
#include "Utilities.h"
#include "ServoHelper.h"

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
	array<double, 3> ComputeControlTorque(array<double, 3> thetaDesired, array<double, 3> thetaFeedback, array<double, 3> dThetaFeedback); //Overload using 0 acceleration and velocity
	void resetIntegral(); //Resets the integral values

	array<double, 3> Kp, Kv, Ki; //Individual gains for Kp, Kv and Ki
	double lastTime = 0; //Used for integral value
	array<double, 3> integralValues; //The array used to store the previous integral values over time
private:
	ArmDynamics dynamics;
	servo dxl;
	Utilities utilities;
	ServoHelper * servoHelper;
};

#endif

