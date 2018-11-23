// ArmControl.h

#ifndef _ARMCONTROL_h
#define _ARMCONTROL_h

#include <array>
#include "ArmDynamics.h"
#include "servo.h"

using namespace std;

enum class ServoType { MX28, MX64, MX106 };

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
	double ComputeOutputPWM(double desiredTorque, ServoType servoType);
	double ConvertVelocitySignal(int16_t signal);
	double ConvertPositionSignal(int16_t signal);

	double ReadPositionRad(int id);
	double ReadVelocityRad(int id);
	array<double, 3> ReadPositionRadArray();
	array<double, 3> ReadVelocityRadArray();
	void SendTorquesAllInOne(array<double, 3> torques);

	bool CheckOverspeed(double speedLimit);
	void SoftEstop();
	void LogArray(String text, array<double, 3> data);
	void Log(String text, double data);
	void Pause();

	double Kp, Kv;
private:
	const double VELOCITY_UNIT = 0.023980823895; //radian/second pr. value
	const double POSITION_UNIT = 0.0015358897; //Radians pr. value

	ArmDynamics dynamics;
	servo dxl;
};

#endif

