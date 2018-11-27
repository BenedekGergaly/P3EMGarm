// ArmKinematics.h

#ifndef _ARMKINEMATICS_h
#define _ARMKINEMATICS_h

#include "Models/Point3D.h"
#include "Models/KinematicInverseAngles.h"
#include "math.h"

class ArmKinematics
{
public:
	ArmKinematics();
	~ArmKinematics();
	Point3D<double> ForwardKinematics(double theta1, double theta2, double theta3);
	KinematicInverseAngles InverseKinematics(Point3D<double> coordinates) const;
private:
	double** getT14Matrix(double theta1, Point3D<double> &coordinates) const;
	Point3D<double> getT14Pos(double theta1, Point3D<double> &coordinates) const;

	const double length1 = 67.0; //base to zero
	const double length2 = 224.0; //1 to 2
	const double length3 = 263.0; //3 to wrist

private:
};

#endif

