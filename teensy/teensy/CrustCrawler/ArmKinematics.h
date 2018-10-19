// ArmKinematics.h

#ifndef _ARMKINEMATICS_h
#define _ARMKINEMATICS_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "Models/Point3D.h"
#include "Models/KinematicInverseAngles.h"
#include "math.h"

class ArmKinematics
{
public:
	ArmKinematics();
	~ArmKinematics();
	Point3D<double> ForwardKinematics(double theta1, double theta2, double theta3);
	KinematicInverseAngles InverseKinematics(Point3D<double> &coordinates) const;
private:
	double** getT14Matrix(double theta1, Point3D<double> &coordinates) const;

private:
};

#endif

