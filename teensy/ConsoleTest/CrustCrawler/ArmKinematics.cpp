#include "ArmKinematics.h"
#include <intrin.h>

ArmKinematics::ArmKinematics()
{
}

ArmKinematics::~ArmKinematics()
{
		
}

Point3D<double> ArmKinematics::ForwardKinematics(double theta1, double theta2, double theta3)
{
	double x = -cos(theta1)*(263 * sin(theta2 + theta3) + 224 * sin(theta2));
	double y = -sin(theta1)*(263 * sin(theta2 + theta3) + 224 * sin(theta2));
	double z = 263 * cos(theta2 + theta3) + 224 * cos(theta2) + 237;
	return Point3D<double>(x, y, z);
}

KinematicInverseAngles ArmKinematics::InverseKinematics(Point3D<double>& coordinates) const
{
	KinematicInverseAngles angles;
	double theta1 = atan2(-coordinates.getY(), -coordinates.getX());
	angles.SolutionOne[0] = theta1;
	angles.SolutionTwo[0] = theta1;
	auto t14Vector = getT14Pos(theta1, coordinates);
	double len1 = sqrt(pow((double)t14Vector.getX(), 2) + pow((double)t14Vector.getZ(), 2));
	double len2 = 224;
	double len3 = 149;
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
	double a = (pow(len2, 2) + pow(len3, 2) - pow(len1, 2));
	double b = (2 * len2 * len3);
	double c = a / b;
	double cTheta3 = -((pow(len2, 2) + pow(len3, 2) - pow(len1, 2)) / (2 * len2 * len3));
	cTheta3 = cTheta3 > 1.00 ? 1.00000 : cTheta3;
	double theta31 = atan2(sqrt(1.0 - pow(cTheta3, 2)), cTheta3);
	double theta32 = atan2(-sqrt(1.0 - pow(cTheta3, 2)), cTheta3);

	angles.SolutionOne[2] = theta31;
	angles.SolutionTwo[2] = theta32;

	if (theta31 == 0 || theta32 == 0)
	{
		//__debugbreak();
	}

	return angles;
}

Point3D<double> ArmKinematics::getT14Pos(double theta1, Point3D<double>& coordinates) const
{
	double x = coordinates.getX()*cos(theta1) + coordinates.getY()*sin(theta1);
	double a = coordinates.getY()*cos(theta1);
	double b = coordinates.getX()*sin(theta1);
	double y = coordinates.getY()*cos(theta1) - coordinates.getX()*sin(theta1);
	if (abs(y) < 0.0001) y = 0;
	double z = -237 + coordinates.getZ();
	Point3D<double> points = Point3D<double>(x, y, z);

	return points;
}

//OBSOLETE
double ** ArmKinematics::getT14Matrix(double theta1, Point3D<double>& coordinates) const
{
	//           ROW COL
	double** matrix = new double*[4]{
		new double[4] { 0, 0, 0, 0 },
		new double[4] { 0, 0, 0, 0 },
		new double[4] { 0, 0, 0, 0 },
		new double[4] { 0, 0, 0, 0 }
	};

	matrix[0][0] = cos(theta1);
	matrix[0][1] = sin(theta1);
	matrix[0][2] = 0;
	matrix[0][3] = coordinates.getX()*cos(theta1) + coordinates.getY()*sin(theta1);
	matrix[1][0] = -sin(theta1);
	matrix[1][1] = cos(theta1);
	matrix[1][2] = 0;
	matrix[1][3] = coordinates.getY()*cos(theta1) - coordinates.getX()*sin(theta1);
	matrix[2][0] = 0;
	matrix[2][1] = 0;
	matrix[2][2] = 1;
	matrix[2][3] = -240 + coordinates.getZ();
	matrix[3][0] = 0;
	matrix[3][1] = 0;
	matrix[3][2] = 0;
	matrix[3][3] = 1;

	return matrix;
}
