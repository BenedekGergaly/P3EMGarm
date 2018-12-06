#include "ArmTrajectory.h"

ArmTrajectory::ArmTrajectory(servo &dynamixel)
{
	servoHelper = new ServoHelper(dynamixel);
}

ArmTrajectory::~ArmTrajectory()
{
}

//#######################################################################################################################
void ArmTrajectory::setNewGoal(array<double, 3> currentAngles, array<double, 3> goalAnglesT, array<double, 3> goalAccelerationsT, double desiredTimeT)
{
	continousMoveFlag = false;
	newGoalFlag = true;
	goalReachedFlag = false;
	goalAngles = goalAnglesT;
	goalAccelerations = goalAccelerationsT;
	startAngles = currentAngles;
	desiredTime = desiredTimeT/1000;
	startTime = utilities.secondsDouble();
	measureRateTempCounter = 0;
	for (int i = 0; i < 3; i++)
	{
		if (goalAngles[i] - startAngles[i] < 0 && goalAccelerations[i] > 0) //make sure acceleration and speed has the same sign
		{
			goalAccelerations[i] *= -1;
		}
		//tb is duration of parabolic part
		tb[i] = 0.5*desiredTime 
			- abs((sqrt(pow(goalAccelerations[i], 2)*pow(desiredTime, 2) - (4 * goalAccelerations[i] * (goalAngles[i] - startAngles[i])))
			/ (2 * goalAccelerations[i])));
		if (isnan(tb[i]))
		{
			Serial.print("[ERROR] Trajectory: Tb is nan for servo #");
			Serial.println(i+1);
			Serial.println("Acceleration too low or time too short! Aborting!");
			Serial.println(desiredTime);
			Serial.println(goalAccelerations[i]);
			Serial.println(goalAngles[i]);
			Serial.println(startAngles[i]);
			servoHelper->SoftEstop();
			utilities.Pause();
		}
		if (tb[i] > desiredTime / 2)
		{
			Serial.print("[ERROR] Trajectory: Acceleration too low or time too short for trajectory generation for servo #");
			Serial.println(i+1);
			Serial.println("Aborting!");
			servoHelper->SoftEstop();
			utilities.Pause();
		}
		output[i][0] = servoHelper->ReadPositionRad(i + 1);
		output[i][1] = 0;
		output[i][2] = 0;
	}
	tempTime = utilities.millisDouble();
}

array<array<double, 3>, 3> ArmTrajectory::calculate()
{
	goalReachedFlag = false;
	for (int i = 0; i < 3; i++)
	{
		output[i][0] += output[i][1] * ((utilities.millisDouble()-tempTime)/1000); //goal angle is increased by speed*time_elapsed
		if (utilities.millisDouble()/1000 - startTime < tb[i]) //beginning parabolic part
		{
			output[i][1] = goalAccelerations[i] * (utilities.millisDouble()/1000 - startTime);
			output[i][2] = goalAccelerations[i];
		}
		else if (utilities.millisDouble()/1000-startTime > desiredTime-tb[i] && utilities.millisDouble()/1000 - startTime < desiredTime) //ending parabolic part
		{
			output[i][1] = goalAccelerations[i] * tb[i] - goalAccelerations[i] * (tb[i] - (desiredTime - (utilities.millisDouble() / 1000-startTime)));
			output[i][2] = -goalAccelerations[i];
		}
		else if (utilities.millisDouble()/1000-startTime > desiredTime) //goal reached
		{
			goalReachedFlag = true;
			output[i][1] = 0;
			output[i][2] = 0;
		}
		else if (utilities.millisDouble() / 1000 - startTime > tb[i] && utilities.millisDouble() / 1000 - startTime < desiredTime - tb[i])//middle linear part
		{
			output[i][1] = goalAccelerations[i] * tb[i];
			output[i][2] = 0;
		}
	}
	if (goalReachedFlag && newGoalFlag)
	{
		Serial.println("[INFO] Trajectory: Goal reached");
		newGoalFlag = false;
	}
	Serial.print("trajectory outputs: ");
	Serial.print(output[2][0]);
	Serial.print("   ");
	Serial.print(output[2][1]);
	Serial.print("   ");
	Serial.println(output[2][2]);
	Serial.println(tb[2]);
	tempTime = utilities.millisDouble();
	return output;
}

//#######################################################################################################################

void ArmTrajectory::startContinousMove(array<double, 3> currentAngles, array<double, 3> goalAccelerationsT, array<double, 3> goalVelocityT)
{
	continousMoveFlag = true;
	goalReachedFlag = false;
	goalAccelerations = goalAccelerationsT;
	startAngles = currentAngles;
	goalSpeeds = goalVelocityT;
	lastTime = utilities.secondsDouble();
	startTime = utilities.secondsDouble();
	for (int i = 0; i < 3; i++)
	{
		output[i][0] = startAngles[i];
		output[i][1] = 0;
		output[i][2] = 0;
	}
}

void ArmTrajectory::stopContinousMove()
{
	continousMoveFlag = false;
}

array<array<double, 3>, 3> ArmTrajectory::calculateContinousMove()
{
	elapsedTime = utilities.secondsDouble() - lastTime;
	lastTime = utilities.secondsDouble();
	if (continousMoveFlag == true) //start and middle curve
	{
		for (int i = 0; i < 3; i++)
		{
			output[i][0] = output[i][0] + (output[i][1] * elapsedTime);
			if (abs(output[i][1]) < abs(goalSpeeds[i]))
			{
				output[i][1] = (utilities.millisDouble() / 1000 - startTime) * goalAccelerations[i];
				output[i][2] = goalAccelerations[i];
			}
			else
			{
				output[i][2] = 0;
			}
		}
		if (int id = checkJointOutOfBounds({ output[0][0], output[1][0], output[2][0] }))
		{
			continousMoveFlag = false;
			utilities.Log("[WARNING] Trajectory: This joint reached its limit", id);
			Serial.println("Stopping gracefully");
		}
		return output;
	}
	else if (continousMoveFlag == false)//end curve
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
				output[i][0] = output[i][0] + (output[i][1] * elapsedTime);
				output[i][1] = output[i][1] - (elapsedTime*goalAccelerations[i]);
				output[i][2] = -goalAccelerations[i];
			}
		}
		if (stopCheck == 3)
		{
			goalReachedFlag = true;
			static bool log = 1;
			if (log)
			{
				Serial.println("[INFO] Trajectory: All joints stopped");
				log = 0;
			}
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

//#######################################################################################################################

void ArmTrajectory::setNewCartesianGoal(array<double, 3> goalPositionT, double desiredTimeT)
{
	cartesianGoalPosition = goalPositionT;
	desiredTime = desiredTimeT / 1000;
	goalReachedFlag = false;
	array<double, 3> angles = servoHelper->ReadPositionRadArray();
	cartesianPosition = kinematics.ForwardKinematics(angles[0], angles[1], angles[2]).getArray();
	//check if trajectory is valid
	for (int i = 0; i < 3; i++)
	{
		cartesianSpeed[i] = (cartesianGoalPosition[i] - cartesianPosition[i]) / 100;
	}
	for (int i = 0; i < 100; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			cartesianPosition[j] += cartesianSpeed[i];
		}
		array<double, 3> solution = kinematics.InverseKinematics(utilities.ArrayToPoint(cartesianPosition)).SolutionOne;
		//solution[0] = adjustJoint1Angle(solution[0], angles[0]);
		for (int j = 0; j < 3; j++)
		{
			if (isnan(solution[j]))
			{
				servoHelper->SoftEstop();
				utilities.LogArray("[ERROR] Trajectory: Cartesian trajectory is invalid. Invalid coordinates in trajectory", solution);
				Serial.println("Ignoring this goal");
				utilities.Pause();
				return;
			}
		}
		if (int id = checkJointOutOfBounds(solution))
		{
			servoHelper->SoftEstop();
			utilities.Log("[ERROR] Trajectory: Cartesian trajectory is invalid. At least this joint would be invalid", id);
			Serial.println("Ignoring this goal");
			utilities.Pause();
			return;
		}
	}
	//assign starting values
	cartesianPosition = kinematics.ForwardKinematics(angles[0], angles[1], angles[2]).getArray();
	for (int i = 0; i < 3; i++)
	{
		cartesianSpeed[i] = (cartesianGoalPosition[i] - cartesianPosition[i]) / desiredTime;
		output[i][0] = servoHelper->ReadPositionRad(i + 1);
		output[i][1] = 0;
		output[i][2] = 0;
	}
	utilities.LogArray("cartSpeed", cartesianSpeed);
	measureRateTempCounter = 0;
	startTime = utilities.secondsDouble();
}

array<array<double, 3>, 3> ArmTrajectory::calculateCartesian()
{
	int interpolatorRelativeRate = 10;
	if (measureRateTempCounter == 0)
	{
		measureRateTempTime = utilities.secondsDouble();
		measureRateTempCounter++;
	}
	else if (measureRateTempCounter == 1)
	{
		currentRate = utilities.secondsDouble() - measureRateTempTime;
		Serial.println(currentRate);
		measureRateTempCounter = -1;
	}
	else if (goalReachedFlag == false && measureRateTempCounter == -1)
	{
		if (cartesianPhase == 0)
		{
			//interpolation
			cartesianPosition = kinematics.ForwardKinematics(output[0][0], output[1][0], output[2][0]).getArray();
			utilities.LogArray("cart1", cartesianPosition);
			for (int i = 0; i < 3; i++)
			{
				cartesianPosition[i] += cartesianSpeed[i] * currentRate * interpolatorRelativeRate;
			}
			utilities.LogArray("cart2", cartesianPosition);
			goalAngles = kinematics.InverseKinematics(utilities.ArrayToPoint(cartesianPosition)).SolutionOne; //elbow up
			//goalAngles[0] = adjustJoint1Angle(goalAngles[0], output[0][0]);
			utilities.LogArray("kinematic solution", goalAngles);
			for (int i = 0; i < 3; i++)
			{
				//control.Log("old speed", output[i][1]);
				double endSpeed = 2 * (goalAngles[i] - output[i][0]) / (currentRate * interpolatorRelativeRate) - output[i][1]; //s=(v0+v1)/2*t solve for v1 --> v1 = (2 s)/t - v0
				goalAccelerations[i] = (endSpeed - output[i][1]) / (currentRate * interpolatorRelativeRate);
				//control.Log("endSpeed", endSpeed);
			}
			//control.Pause();
		}
		//trajectory gen
		for (int i = 0; i < 3; i++)
		{
			output[i][2] = goalAccelerations[i];
			output[i][1] += goalAccelerations[i] * currentRate;
			output[i][0] += output[i][1] * currentRate;
		}

		cartesianPhase++;
		if (cartesianPhase == interpolatorRelativeRate) cartesianPhase = 0;
		if (utilities.secondsDouble() - startTime > desiredTime) goalReachedFlag = true;

		utilities.LogArray("goal position", cartesianGoalPosition);
		utilities.LogArray("output[][] position", kinematics.ForwardKinematics(output[0][0], output[1][0], output[2][0]).getArray());
		utilities.LogArray("measured position", kinematics.ForwardKinematics(servoHelper->ReadPositionRad(1), servoHelper->ReadPositionRad(2),
			servoHelper->ReadPositionRad(3)).getArray());

		return output;
	}

	if (goalReachedFlag)
	{
		Serial.println("Goal reached");
		utilities.LogArray("goal position", cartesianGoalPosition);
		utilities.LogArray("output[ ][ ] position", kinematics.ForwardKinematics(output[0][0], output[1][0], output[2][0]).getArray());
		utilities.LogArray("measured position", kinematics.ForwardKinematics(servoHelper->ReadPositionRad(1), servoHelper->ReadPositionRad(2),
			servoHelper->ReadPositionRad(3)).getArray());

		servoHelper->SoftEstop();
		utilities.Pause();
	}

	for (int i = 0; i < 3; i++)
	{
		output[i][0] = servoHelper->ReadPositionRad(i + 1);
		output[i][1] = 0;
		output[i][2] = 0;
	}
	return output;
}

//#######################################################################################################################

void ArmTrajectory::startContinousCartesianMove(array<double, 3> cartesianSpeedT)
{
	continousMoveFlag = true;
	goalReachedFlag = false;
	cartesianSpeed = cartesianSpeedT;
	startTime = utilities.secondsDouble();
	lastTime = utilities.secondsDouble();
	measureRateTempCounter = 0;
	cartesianPhase = 0;
	cartesianEndCounter = 0;
	for (int i = 0; i < 3; i++)
	{
		output[i][0] = servoHelper->ReadPositionRad(i+1);
		output[i][1] = 0;
		output[i][2] = 0;
	}
	angles = servoHelper->ReadPositionRadArray();
	cartesianPosition = kinematics.ForwardKinematics(servoHelper->ReadPositionRad(1), servoHelper->ReadPositionRad(2), servoHelper->ReadPositionRad(3)).getArray();
}

void ArmTrajectory::stopContinousCartesianMove()
{
	continousMoveFlag = false;
}

array<array<double, 3>, 3> ArmTrajectory::calculateContinousCartesianMove()
{
	int interpolatorRelativeRate = 10;
	elapsedTime = utilities.secondsDouble() - lastTime;
	lastTime = utilities.secondsDouble();
	if(continousMoveFlag == true)
	{
		if (cartesianPhase == 0)
		{
			//interpolation
			cartesianPosition = kinematics.ForwardKinematics(output[0][0], output[1][0], output[2][0]).getArray();
			//utilities.LogArray("cart1", cartesianPosition);
			for (int i = 0; i < 3; i++)
			{
				cartesianPosition[i] += cartesianSpeed[i] * elapsedTime * interpolatorRelativeRate;
			}
			//utilities.LogArray("cart2", cartesianPosition);
			goalAngles = kinematics.InverseKinematics(utilities.ArrayToPoint(cartesianPosition)).SolutionOne; //maybe need logic to decide solotion 1 vs 2
			//utilities.LogArray("undajusted 1", goalAngles);
			array<double, 3> goalAngles2 = kinematics.InverseKinematics(utilities.ArrayToPoint(cartesianPosition)).SolutionTwo;
			//utilities.LogArray("unadjusted 2", goalAngles2);
			adjustInverseKinematicAngles(goalAngles, {output[0][0], output[1][0], output[2][0] });
			//utilities.LogArray("inv kinematic solution", goalAngles);
			//servoHelper->SoftEstop();
			//utilities.Pause();
			for (int i = 0; i < 3; i++)
			{
				//control.Log("old speed", output[i][1]);
				double endSpeed = 2 * (goalAngles[i] - output[i][0]) / (elapsedTime * interpolatorRelativeRate) - output[i][1]; //s=(v0+v1)/2*t solve for v1 --> v1 = (2 s)/t - v0
				goalAccelerations[i] = (endSpeed - output[i][1]) / (elapsedTime * interpolatorRelativeRate);
				//control.Log("endSpeed", endSpeed);
			}
			//control.Pause();
		}
		//check limits
		if (int badJoint = checkJointOutOfBounds(goalAngles))
		{
			utilities.Log("[WARNING] Trajectory: At least this joint reached its limit", badJoint);
			continousMoveFlag = false;
			for (int i = 0; i < 3; i++)
			{
				output[i][0] = servoHelper->ReadPositionRad(i + 1);
			}
		}
		else
		{
			//trajectory gen
			for (int i = 0; i < 3; i++)
			{
				output[i][2] = goalAccelerations[i];
				output[i][1] += goalAccelerations[i] * elapsedTime;
				output[i][0] += output[i][1] * elapsedTime;
			}
		}

		cartesianPhase++;
		if (cartesianPhase == interpolatorRelativeRate) cartesianPhase = 0;

		//utilities.LogArray("output[][] position", kinematics.ForwardKinematics(output[0][0], output[1][0], output[2][0]).getArray());
		//utilities.LogArray("measured position", kinematics.ForwardKinematics(servoHelper->ReadPositionRad(1), servoHelper->ReadPositionRad(2),
			//servoHelper->ReadPositionRad(3)).getArray());

		return output;
	}
	else if (continousMoveFlag == false && goalReachedFlag == false)
	{
		//outputs the last valid values for x more seconds, then assumes goal is reached
		double endWaitTime = 1;
		for (int i = 0; i < 3; i++)
		{
			output[i][2] = 0;
			output[i][1] = 0;
		}
		cartesianEndCounter++;
		if (cartesianEndCounter > endWaitTime / elapsedTime)
		{
			goalReachedFlag = true;
		}
	}
	//utilities.LogArray("output[][] position", kinematics.ForwardKinematics(output[0][0], output[1][0], output[2][0]).getArray());
	//utilities.LogArray("measured position", kinematics.ForwardKinematics(servoHelper->ReadPositionRad(1), servoHelper->ReadPositionRad(2),
	//	servoHelper->ReadPositionRad(3)).getArray());
	return output;
}
//#######################################################################################################################

int ArmTrajectory::checkJointOutOfBounds(array<double, 3> input)
{
	if (input[0] < joint1Min || input[0] > joint1Max) return 1;
	if (input[1] < joint2Min || input[1] > joint2Max) return 2;
	if (input[2] < joint3Min || input[2] > joint3Max) return 3;
	return 0;
}

void ArmTrajectory::adjustInverseKinematicAngles(array<double, 3>& solution, array<double, 3> reference)
{
	if (abs(reference[0] - solution[0]) > 2.8 && abs(reference[0] - solution[0]) < 3.4)
	{
		if (solution[0] > 0) solution[0] -= PI;
		else solution[0] += PI;
		solution[1] *= -1;
		solution[2] *= -1;
	}
}

void ArmTrajectory::printDebug()
{
	utilities.Log("elapsedTime", elapsedTime);
	utilities.Log("contMovFlag", continousMoveFlag);
	utilities.Log("goalReachedFlag", goalReachedFlag);
	utilities.LogArray("TgoalAcc", goalAccelerations);
	utilities.LogArray("TstartAng", startAngles);
	utilities.LogArray("TgoalSpeed", goalSpeeds);
}

