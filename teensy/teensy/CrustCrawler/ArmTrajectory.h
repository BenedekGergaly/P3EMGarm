#pragma once

#include <array>
#include <cmath>
#include <Arduino.h>
#include "servo.h"
#include "ArmControl.h"
#include "ArmKinematics.h"
#include "Models/Point3D.h"
#include "Utilities.h"
#include "ServoHelper.h"

using namespace std;

class ArmTrajectory
{
public:
	ArmTrajectory(servo &dynamixel);
	~ArmTrajectory();

	double maxJointSpeed = 0.8; //rad/s

	void setNewGoal(array<double, 3> currentAngles, array<double, 3> goalAnglesT, array<double, 3> goalAccelerationsT, double desiredTimeT);
	array<array<double, 3>, 3> calculate();

	void startContinousMove(array<double, 3> currentAngles, array<double, 3> goalAccelerationsT, array<double, 3> goalVelocityT);
	void stopContinousMove();
	array<array<double, 3>, 3> calculateContinousMove();

	void setNewCartesianGoal(array<double, 3> goalPositionT, double desiredTimeT);
	array<array<double, 3>, 3> calculateCartesian();

	void startContinousCartesianMove(array<double, 3> cartesianSpeedT);
	void stopContinousCartesianMove();
	array<array<double, 3>, 3> calculateContinousCartesianMove();

	array<array<double, 3>, 3> output; //1st index: servo#; 2nd index: angle, speed, acceleration
	bool goalReachedFlag = 0;
	bool continousMoveFlag = 0;

	int checkJointOutOfBounds(array<double, 3> input); //returns 0 if okay, joint number if failed
	void adjustInverseKinematicAngles(array<double, 3> &solution, array<double, 3> reference); //makes sure angles over 180 don't become the opposite sign during inverse kinematics

	double time = 2500; //debug stuff


private:
	ArmControl control;
	ArmKinematics kinematics;
	Utilities utilities;
	ServoHelper * servoHelper;

	array<double, 3> goalAngles, goalSpeeds, goalAccelerations, startAngles, currentSpeeds, anglesNew;
	double startTime, desiredTime;
	array<double, 3> tb;
	bool newGoalFlag = 0;
	double measureRateTempTime, measureRateTempCounter;
	double currentRate;
	double tempTime;
	double lastTime, elapsedTime;
	array<double, 3> cartesianGoalPosition, cartesianSpeed, cartesianPosition, cartesianPositionNew, angles, angleDifference, cartesianDifference;
	int cartesianPhase = 0;
	int cartesianEndCounter = 0;

	const double joint1Min = -3;
	const double joint1Max = 7;
	const double joint2Min = -2;
	const double joint2Max = 2;
	const double joint3Min = -2;
	const double joint3Max = 2;
};

