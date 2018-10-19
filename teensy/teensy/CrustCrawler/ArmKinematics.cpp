#include "ArmKinematics.h"

ArmKinematics::ArmKinematics()
{
}

ArmKinematics::~ArmKinematics()
{
		
}

Point3D<double> ArmKinematics::ForwardKinematics(double theta1, double theta2, double theta3)
{
	double x = -cos(theta1)*(270 * sin(theta2 + theta3) + 221 * sin(theta2));
	double y = -sin(theta1)*(270 * sin(theta2 + theta3) + 221 * sin(theta2));
	double z = 270 * cos(theta2 + theta3) + 221 * cos(theta2) + 240;
	return Point3D<double>(x, y, z);
}

KinematicInverseAngles ArmKinematics::InverseKinematics(Point3D<double>& coordinates) const
{
	KinematicInverseAngles angles;
	double theta1 = atan2(-coordinates.getY(), -coordinates.getX());
	angles.SolutionOne[0] = theta1;
	angles.SolutionTwo[0] = theta1;
	double ** t14Matrix = getT14Matrix(theta1, coordinates);
	double len1 = pow((double)t14Matrix[0][3], 2) + pow((double)t14Matrix[2][3], 2);
	double len2 = 235;
	double len3 = 270;
	double phi1 = acos((len1 + pow(len2, 2) - pow(len3, 2) / 2 * len1*len2));
	double phi2 = atan2((double)t14Matrix[2][3], -t14Matrix[0][3]);
	double theta21 = (M_PI / 2) - phi1 - phi2;
	double theta22 = (M_PI / 2) + phi1 - phi2;

	angles.SolutionOne[1] = theta21;
	angles.SolutionTwo[1] = theta22;

	double cTheta3 = -((pow(len2, 2) + pow(len3, 2) - len1) / 2 * len2*len3);
	double theta31 = atan2(sqrt(1.0 - pow(cTheta3, 2)), cTheta3);
	double theta32 = atan2(-sqrt(1.0 - pow(cTheta3, 2)), cTheta3);

	angles.SolutionOne[2] = theta31;
	angles.SolutionTwo[2] = theta32;

	return angles;
}

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
