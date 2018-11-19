#pragma once

#include <array>
#include <cmath>
#include <Arduino.h>
#include "ArmControl.h"
#include "ArmKinematics.h"
#include "Models/Point3D.h"

using namespace std;

class ArmTrajectory
{
public:
	ArmTrajectory();
	~ArmTrajectory();

	double maxJointSpeed = 0.8; //rad/s

	void setNewGoal(array<double, 3> currentAngles, array<double, 3> goalAnglesT, array<double, 3> goalAccelerationsT, double desiredTimeT);
	void startContinousMove(array<double, 3> currentAngles, array<double, 3> goalAccelerationsT, array<double, 3> goalVelocityT);
	void stopContinousMove();
	void startContinousCartesianMove(array<double, 3> cartesianSpeedT);
	void stopContinousCartesianMove();
	array<array<double, 3>, 3> calculate();
	array<array<double, 3>, 3> calculateContinousMove();
	array<array<double, 3>, 3> calculateContinousCartesianMove();
	void setNewCartesianGoal(array<double, 3> goalPositionT, array<double, 3> cartesianSpeedT);
	array<array<double, 3>, 3> calculateCartesian();


	array<array<double, 3>, 3> output; //1st index: servo#; 2nd index: angle, speed, acc
	bool goalReachedFlag = 0;
	bool continousMoveFlag = 0;

	double time = 2500; //debug stuff

private:
	array<double, 3> goalAngles, goalSpeeds, goalAccelerations, startAngles;
	double startTime, desiredTime;
	array<double, 3> tb;
	bool newGoalFlag = 0;
	double measureRateTempTime, measureRateTempCounter;
	double currentRate;
	double millisDouble();
	double secsDouble();
	ArmControl control;
	double tempTime;
	array<double, 3> cartesianGoalPosition, cartesianSpeed, cartesianPosition, cartesianPositionNew, angles, angleDifference, cartesianDifference;
	ArmKinematics kinematics;
	Point3D<double> arrayToPoint(array<double, 3> a);
};

