#include "ArmTrajectory.h"
#include <iostream>



ArmTrajectory::ArmTrajectory()
{
}


ArmTrajectory::~ArmTrajectory()
{
}

void ArmTrajectory::setNewGoal(array<double, 3> currentAngles, array<double, 3> goalAnglesT, array<double, 3> goalAccelerationsT, double desiredTimeT)
{
	continousMoveFlag = 0;
	newGoalFlag = 1;
	goalAngles = goalAnglesT;
	goalAccelerations = goalAccelerationsT;
	startAngles = currentAngles;
	desiredTime = desiredTimeT / 1000;
	startTime = time / 1000;
	for (int i = 0; i < 3; i++)
	{
		if (goalAngles[i] - startAngles[i] < 0 && goalAccelerations[i] > 0)
		{
			goalAccelerations[i] *= -1;
		}
		tb[i] = 0.5*desiredTime
			- abs((sqrt(pow(goalAccelerations[i], 2)*pow(desiredTime, 2) - (4 * goalAccelerations[i] * (goalAngles[i] - startAngles[i])))
				/ (2 * goalAccelerations[i])));
		if (tb[i] > desiredTime / 2)
		{
			std::cout << "[ERROR] Trajectory: Acceleration too low or time too short for trajectory generation for servo #" << i << std::endl;
		
		}
	}
}


array<array<double, 3>, 3> ArmTrajectory::calculate()
{
	goalReachedFlag = 0;
	for (int i = 0; i < 3; i++)
	{
		output[i][0] = goalAngles[i];
		if (time / 1000 - startTime < tb[i]) //start curve
		{
			output[i][1] = goalAccelerations[i] * (time / 1000 - startTime);
			output[i][2] = goalAccelerations[i];
		}
		else if (time / 1000 - startTime > desiredTime - tb[i] && time / 1000 - startTime < desiredTime) //end curve
		{
			output[i][1] = goalAccelerations[i]*tb[i] - goalAccelerations[i] * (tb[i] - (desiredTime-time/1000));
			output[i][2] = -goalAccelerations[i];
		}
		else if (time / 1000 - startTime > desiredTime) //finished
		{
			goalReachedFlag = 1;
			output[i][1] = 0;
			output[i][2] = 0;
		}
		else if (time/1000 > tb[i] && time/1000 < desiredTime-tb[i]) //middle curve
		{
			output[i][1] = goalAccelerations[i] * tb[i];
			output[i][2] = 0;
		}
	}
	if (goalReachedFlag && newGoalFlag)
	{
		std::cout << "[INFO] Trajectory: Goal reached" << std::endl;
		newGoalFlag = 0;
	}

	return output;
}

void ArmTrajectory::startContinousMove(array<double, 3> currentAngles, array<double, 3> goalAccelerationsT, array<double, 3> goalVelocityT)
{
	continousMoveFlag = 1;
	goalReachedFlag = 0;
	goalAccelerations = goalAccelerationsT;
	startAngles = currentAngles;
	goalSpeeds = goalVelocityT;
	startTime = time / 1000;
	measureRateTempCounter = 0;
	for (int i = 0; i < 3; i++)
	{
		output[i][0] = startAngles[i];
		output[i][1] = 0;
		output[i][2] = 0;
	}
}

void ArmTrajectory::stopContinousMove()
{
	continousMoveFlag = 0;
}

array<array<double, 3>, 3> ArmTrajectory::calculateContinousMove()
{
	if (measureRateTempCounter == 0)
	{
		measureRateTempTime = time / 1000;
		measureRateTempCounter++;
	}
	else if (measureRateTempCounter == 1)
	{
		currentRate = time/1000 - measureRateTempTime;
		measureRateTempCounter = -1;
	}
	else if (continousMoveFlag == 1) //start and middle curve
	{
		for (int i = 0; i < 3; i++)
		{
			output[i][0] = output[i][0] + (output[i][1] * currentRate);
			if (output[i][1] < goalSpeeds[i])
			{
				output[i][1] = (time / 1000 - startTime) * goalAccelerations[i];
				output[i][2] = goalAccelerations[i];
			}
			else
			{
				output[i][2] = 0;
			}
		}
		return output;
	}
	else if (continousMoveFlag == 0 && goalReachedFlag == 0)//end curve
	{
		int stopCheck = 0;
		for (int i = 0; i < 3; i++)
		{
			if (output[i][1] * goalSpeeds[i] <= 0) //stopped
			{
				output[i][1] = 0;
				output[i][2] = 0;
				stopCheck++;
			}
			else //not stopped yet
			{
				output[i][0] = output[i][0] + (output[i][1] * currentRate);
				output[i][1] = output[i][1] - (currentRate*goalAccelerations[i]);
				output[i][2] = -goalAccelerations[i];
			}
		}
		if (stopCheck == 3)
		{
			goalReachedFlag = 1;
			std::cout << "[INFO] Trajectory: All joints stopped" << std::endl;
		}
		return output;
	}

	for (int i = 0; i < 3; i++)
	{
		output[i][0] = startAngles[i];
		output[i][1] = 0;
		output[i][2] = 0;
	}
	return output;
}