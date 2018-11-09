#pragma once

#include <array>
#include <cmath>
#include <Arduino.h>

using namespace std;

class ArmTrajectory
{
public:
	ArmTrajectory();
	~ArmTrajectory();

	void setNewGoal(array<double, 3> currentAngles, array<double, 3> goalAnglesT, array<double, 3> goalAccelerationsT, double desiredTimeT);
	void startContinousMove(array<double, 3> currentAngles, array<double, 3> goalAccelerationsT, array<double, 3> goalVelocityT);
	void stopContinousMove();
	array<array<double, 3>, 3> calculate();
	array<array<double, 3>, 3> calculateContinousMove();

	bool goalReachedFlag = 0;
	bool continousMoveFlag = 0;

private:
	array<double, 3> goalAngles, goalSpeeds, goalAccelerations, startAngles;
	double startTime, desiredTime;
	array<array<double, 3>, 3> output; //1st index: servo#; 2nd index: angle, speed, acc
	array<double, 3> tb;
	bool newGoalFlag = 0;
	int measureRateTempTime, measureRateTempCounter;
	double currentRate;
};

