#include "ArmKinematics.h"


ArmKinematics::ArmKinematics()
{
}

ArmKinematics::~ArmKinematics()
{

}

// Calculates the forward dynamics
Point3D<double> ArmKinematics::ForwardKinematics(double theta1, double theta2, double theta3)
{
	double x = -cos(theta1)*(263 * sin(theta2 + theta3) + 224 * sin(theta2));
	double y = -sin(theta1)*(263 * sin(theta2 + theta3) + 224 * sin(theta2));
	double z = 263 * cos(theta2 + theta3) + 224 * cos(theta2) + 237;
	return Point3D<double>(x, y, z);
}


//Calculates 2 inverse kinematics solutions
KinematicInverseAngles ArmKinematics::InverseKinematics(Point3D<double> coordinates) const
{
	KinematicInverseAngles angles;
	double theta1 = atan2(-coordinates.getY(), -coordinates.getX());
	angles.SolutionOne[0] = theta1;
	angles.SolutionTwo[0] = theta1;
	auto t14Vector = getT1TPos(theta1, coordinates);
	double len1 = sqrt(pow((double)t14Vector.getX(), 2) + pow((double)t14Vector.getZ(), 2));
	double len2 = length2;
	double len3 = length3;
	double temp1 = pow(len1, 2) + pow(len2, 2) - pow(len3, 2);
	double temp2 = 2.0 * len1 * len2;
	double fraction = temp1 / temp2;
	fraction = fraction > 1.00 ? 1.00000 : fraction;
	double phi1 = acos(fraction);
	double phi2 = atan2((double)t14Vector.getZ(), -t14Vector.getX());
	double theta21 = (M_PI / 2.0) - phi1 - phi2;
	double theta22 = (M_PI / 2.0) + phi1 - phi2;

	angles.SolutionOne[1] = theta21;
	angles.SolutionTwo[1] = theta22;

	double cTheta3 = -((pow(len2, 2) + pow(len3, 2) - pow(len1, 2)) / (2 * len2 * len3));
	cTheta3 = cTheta3 > 1.00 ? 1.00000 : cTheta3;
	double theta31 = atan2(sqrt(1.0 - pow(cTheta3, 2)), cTheta3);
	double theta32 = atan2(-sqrt(1.0 - pow(cTheta3, 2)), cTheta3);

	angles.SolutionOne[2] = theta31;
	angles.SolutionTwo[2] = theta32;

	return angles;
}

Point3D<double> ArmKinematics::getT1TPos(double theta, Point3D<double>& coordinates) const
{
	double x = coordinates.getX()*cos(theta) + coordinates.getY()*sin(theta);
	double y = coordinates.getY()*cos(theta) - coordinates.getX()*sin(theta);
	double z = -237 + coordinates.getZ();
	Point3D<double> points = Point3D<double>(x, y, z);

	return points;
}
