// ArmKinematics.h

#ifndef _ARMKINEMATICS_h
#define _ARMKINEMATICS_h

#include "Models/Point3D.h"
#include "Models/KinematicInverseAngles.h"
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>

class ArmKinematics
{
public:
	ArmKinematics();
	~ArmKinematics();
	Point3D<double> ForwardKinematics(double theta1, double theta2, double theta3);
	KinematicInverseAngles InverseKinematics(Point3D<double> &coordinates) const;
private:
	double** getT14Matrix(double theta1, Point3D<double> &coordinates) const;
	Point3D<double> getT14Pos(double theta1, Point3D<double> &coordinates) const;

private:
};

#endif

