//CORRECT WIRE COLORS: black, green, yellow, blue, red to 3.3v

#include <array>
#include <EEPROM.h>
#include <Arduino.h>
#include "servo.h"
#include "CrustCrawler/ArmKinematics.h"
#include "CrustCrawler/ArmControl.h"
#include "CrustCrawler/ArmTrajectory.h"
#include "CrustCrawler/ArmDynamics.h"
#include "CrustCrawler/Utilities.h"
#include "CrustCrawler/ServoHelper.h"

const int rest = 0;
const int waveIn = 1;
const int waveOut = 2;
const int fist = 3;
const int fingerSpread = 4;
const int doubleTap = 5;
const int none = -1;
const int position = 3;
const int pwm = 16;

int pose = none; //rest, waveIn, waveOut, fist, fingerSpread, doubleTap
int currentControlAxis = 3; //1, 2, 3 = x, y, z mode
bool gripperState = 0;
int cycleFrequency;
int cycleTime;
unsigned long timer = 0;
bool enableJointWaypointLoop = false;
bool enableJointContinousLoop = false;
bool enableCartesianWaypointLoop = false;
bool enableCartesianContinousLoop = false;
bool enableDebug = false;
int currentWaypointID = 1;
int debugPhase = 1;

array<double, 3> desiredAngles;
array<double, 3> desiredAccelerations;
array<double, 3> currentAngles;

array<double, 3> continousSpeeds = {0,0,0};
array<double, 3> continousAccelerations = {0,0,0};
array<double, 3> cartesianContinousSpeeds = { 0,0,0 };

servo dxl;
ArmKinematics kinematics;
ArmControl control;
ArmTrajectory trajectory = ArmTrajectory(dxl);
ArmDynamics dynamics;
Utilities utilities;
ServoHelper servoHelper = ServoHelper(dxl);

using namespace std;

void applyJointWaypointMove()
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();

	array<array<double, 3>, 3> inputs = trajectory.calculate();
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}
	utilities.LogArray("Current position", feedbackPosition);
	utilities.LogArray("desired position", desiredAngles);
	utilities.LogArray("Current velocity", feedbackVelocity);
	utilities.LogArray("desired velocity", desiredSpeeds);

	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	utilities.LogArray("Torques", torques);
	servoHelper.SendTorquesAllInOne(torques);
	Serial.println("=================");
}

void applyJointContinousMove()
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();
	utilities.LogArray("Current position", feedbackPosition);
	utilities.LogArray("Current velocity", feedbackVelocity);

	array<array<double, 3>, 3> inputs = trajectory.calculateContinousMove();
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}
	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	utilities.LogArray("Torques", torques);
	utilities.LogArray("Angles", desiredAngles);
	utilities.LogArray("speeds", desiredSpeeds);
	utilities.LogArray("accelerations", desiredAccelerations);
	servoHelper.SendTorquesAllInOne(torques);
	Serial.println("=================");
}

void applyCartesianWaypointMove()
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();
	//control.LogArray("Current position", feedbackPosition);
	//control.LogArray("Current velocity", feedbackVelocity);

	array<array<double, 3>, 3> inputs = trajectory.calculateCartesian();
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}
	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	//control.LogArray("Angles", desiredAngles);
	//control.LogArray("Speeds", desiredSpeeds);
	//control.LogArray("Accelerations", desiredAccelerations);
	//control.LogArray("Torques", torques);
	servoHelper.SendTorquesAllInOne(torques);
	//Serial.println("=================");
}

void applyCartesianContinousMove()
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();
	utilities.LogArray("Current position", feedbackPosition);
	utilities.LogArray("Current velocity", feedbackVelocity);

	array<array<double, 3>, 3> inputs = trajectory.calculateContinousCartesianMove();
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}
	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	//control.LogArray("Angles", desiredAngles);
	//control.LogArray("Speeds", desiredSpeeds);
	//control.LogArray("Accelerations", desiredAccelerations);
	//control.LogArray("Torques", torques);
	servoHelper.SendTorquesAllInOne(torques);
	Serial.println("=================");
}

void debug()
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();
	utilities.LogArray("Current position", feedbackPosition);
	utilities.LogArray("Current velocity", feedbackVelocity);
	array<double, 3> desiredSpeeds = { 0,0,0 };
	array<double, 3> desiredAngles = { 0,0,0 };
	if (debugPhase == 1)
	{
		desiredAngles = { 0,0,0 };
	}
	else
	{
		desiredAngles = { 0,1.57, -1.57 };
	}

	array<double, 3> desiredAccelerations = { 0,0,0 };
	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	utilities.LogArray("Angles", desiredAngles);
	utilities.LogArray("Speeds", desiredSpeeds);
	utilities.LogArray("Accelerations", desiredAccelerations);
	utilities.LogArray("Torques", torques);
	servoHelper.SendTorquesAllInOne(torques);
	Serial.println("=================");
}

void printEEPROMvalues()
{
    Serial.print("Frequency: ");
    Serial.println(cycleFrequency);
	utilities.LogArray("Kp", control.Kp);
	utilities.LogArray("Kv", control.Kv);
	utilities.LogArray("Ki", control.Ki);
}

void gripper(bool b) //1=close, 0=open
{
	if (b == 0 && gripperState == 1)
    {

        dxl.torqueEnable(4,0);
        dxl.torqueEnable(5,0);
        dxl.setOperatingMode(4,position);
        dxl.setOperatingMode(5,position);
        dxl.torqueEnable(4,1);
        dxl.torqueEnable(5,1);
        dxl.setGoalPosition(4,2600);
        dxl.setGoalPosition(5,3200);
		gripperState = 0;
    }
	else if (b == 1 && gripperState == 0)
    {
        dxl.torqueEnable(4,0);
        dxl.torqueEnable(5,0);
        dxl.setOperatingMode(4,pwm);
        dxl.setOperatingMode(5,pwm);
        dxl.torqueEnable(4,1);
        dxl.torqueEnable(5,1);
		dxl.setGoalPwm(4, -150);
		dxl.setGoalPwm(5, -150);
		gripperState = 1;
    }
}

void enableTorqueForAll()
{
	dxl.torqueEnable(1, 0);
	dxl.torqueEnable(2, 0);
	dxl.torqueEnable(3, 0);
	dxl.setOperatingMode(1, pwm);
	dxl.setOperatingMode(2, pwm);
	dxl.setOperatingMode(3, pwm);
	dxl.torqueEnable(1, 1);
	dxl.torqueEnable(2, 1);
	dxl.torqueEnable(3, 1);
	Serial.println("[INFO] Enabled torque for all servos");
	servoHelper.LEDsOff();
}

void commandDecoder()
{
	if (Serial.available() > 0)
	{
		switch (Serial.read())
		{
		case 'a': //pose input
			pose = Serial.parseInt();
			break;
		case 'r': //retrieve EEPROM values
			printEEPROMvalues();
			break;
		case 'f': //change frequency
			cycleFrequency = Serial.parseInt();
			EEPROM.put(100, cycleFrequency);
			cycleTime = 1000000 / cycleFrequency;
			break;
		case 't': //torque enable/disable ex: t3,1  joint3 enable
		{
			int id = Serial.parseInt();
			Serial.read();
			bool enable = Serial.parseInt();
			dxl.torqueEnable(id, enable);
			break;
		}
		case 'x': //"reset"
			setup();
			break;
		case 'p': //read position
		{
			int id = Serial.parseInt();
			Serial.print("[INFO] Main: Make sure servo is not in position mode!");
			double position = servoHelper.ReadPositionRad(id);
			Serial.println(position);
			break;
		}
		case 'o': //set operating mode 0=current, 3=position 16=pwm
		{
			int id = Serial.parseInt();
			Serial.read();
			int mode = Serial.parseInt();
			dxl.setOperatingMode(id, mode);
			break;
		}
		case 'g': //set goal position
		{
			int id = Serial.parseInt();
			Serial.read();
			int32_t pos = Serial.parseInt();
			dxl.setGoalPosition(id, pos);
			break;
		}
		case 'w': //set goal pwm
		{
			int id = Serial.parseInt();
			Serial.read();
			int data = Serial.parseInt();
			dxl.setGoalPwm(id, data);
			break;
		}
		case 'b': //grip 0=open 1=close
			gripper(Serial.parseInt());
			break;
		case 'n': //update Kp
		{
			int joint = Serial.parseInt()-1;
			Serial.read();
			double value = Serial.parseFloat();
			EEPROM.put(200 + joint * 10, value);
			utilities.Log("Joint", joint + 1);
			utilities.Log("Kp changed to", value);
			control.Kp[joint] = value;
			break;
		}
		case 'm': //update Kv
		{
			int joint = Serial.parseInt() - 1;
			Serial.read();
			double value = Serial.parseFloat();
			EEPROM.put(300 + joint * 10, value);
			utilities.Log("Joint", joint + 1);
			utilities.Log("Kv changed to", value);
			control.Kv[joint] = value;
			break;
		}
		case 'c': //update Ki
		{
			int joint = Serial.parseInt() - 1;
			Serial.read();
			double value = Serial.parseFloat();
			EEPROM.put(400 + joint * 10, value);
			utilities.Log("Joint", joint + 1);
			utilities.Log("Ki changed to", value);
			control.Ki[joint] = value;
			break;
		}
		case 'k': //Joint waypoint mode
		{
			enableTorqueForAll();
			enableJointWaypointLoop = true;
			trajectory.goalReachedFlag = 1;
			break;
		}
		case 'h': //put in PWM mode
			enableTorqueForAll();
			break;
		case 'j': //Cartesian waypoint mode
		{
			enableTorqueForAll();
			enableCartesianWaypointLoop = true;
			array<double, 3> cartesianGoal = { 260, 0, 200 };
			trajectory.setNewCartesianGoal(cartesianGoal, 3000);
			break;
		}
		case 'i': //check kinematics
		{
			dxl.torqueEnable(1, 0);
			dxl.torqueEnable(2, 0);
			dxl.torqueEnable(3, 0);
			dxl.setOperatingMode(1, pwm);
			dxl.setOperatingMode(2, pwm);
			dxl.setOperatingMode(3, pwm);
			array<double, 3> joint = servoHelper.ReadPositionRadArray();
			utilities.LogArray("joints", joint);
			array<double, 3> cartPos = kinematics.ForwardKinematics(servoHelper.ReadPositionRad(1), servoHelper.ReadPositionRad(2),
				servoHelper.ReadPositionRad(3)).getArray();
			utilities.LogArray("EE cartesian position", cartPos);
			array<double, 3> sol1, sol2;
			sol1 = kinematics.InverseKinematics(utilities.ArrayToPoint(cartPos)).SolutionOne;
			sol2 = kinematics.InverseKinematics(utilities.ArrayToPoint(cartPos)).SolutionTwo;
			utilities.LogArray("unadjusted 1", sol1);
			utilities.LogArray("unadjusted 2", sol2);
			trajectory.adjustInverseKinematicAngles(sol1, joint);
			trajectory.adjustInverseKinematicAngles(sol2, joint);
			utilities.LogArray("adjusted 1", sol1);
			utilities.LogArray("adjusted 2", sol2);
			break;
		}
		case 's': //stop
			servoHelper.SoftEstop();
			enableJointWaypointLoop = 0;
			enableJointContinousLoop = 0;
			enableCartesianWaypointLoop = 0;
			enableCartesianContinousLoop = 0;
			enableDebug = 0;
			break;
		case 'l': //loosen
			dxl.torqueEnable(1, 0);
			dxl.torqueEnable(2, 0);
			dxl.torqueEnable(3, 0);
			servoHelper.LEDsOff();
			break;
		case 'q':
			enableDebug = true;
			enableTorqueForAll();
			control.integralValues = { 0,0,0 };
			if (debugPhase == 1)
			{
				debugPhase = 2;
			}
			else
			{
				debugPhase = 1;
			}
			break;
		case ',':
			Serial.read();
			break;
		default:
			Serial.find(',');
			Serial.println("ERROR: Unknown command");
			break;
		}
	}
}

void poseDecoder()
{
	static bool enableFlag = false;
	switch (pose)
	{
	case doubleTap:
		currentControlAxis += 1;
		if (currentControlAxis == 4) currentControlAxis = 1;
		pose = none;
		utilities.Log("Current axis", currentControlAxis);
		break;
	case fist:
		gripper(1);
		pose = none;
		break;
	case fingerSpread:
		gripper(0);
		pose = none;
		break;
	case rest:
		trajectory.stopContinousMove();
		break;
	case waveIn:
		if (enableFlag == false)
		{
			enableTorqueForAll();
			enableFlag = true;
		}
		//enableCartesianContinousLoop = true;
		enableJointContinousLoop = true;
		pose = none;
		switch (currentControlAxis)
		{
		case 1:
			continousSpeeds = { 0.3,0,0 };
			continousAccelerations = { 2,0,0 };
			cartesianContinousSpeeds = { 60, 0, 0 };
			break;
		case 2:
			continousSpeeds = { 0,0.3,0 };
			continousAccelerations = { 0,2,0 };
			cartesianContinousSpeeds = {0,60,0};
			break;
		case 3:
			continousSpeeds = { 0,0,0.3 };
			continousAccelerations = { 0,0,2 };
			cartesianContinousSpeeds = {0,0,60};
			break;
		}
		if (trajectory.goalReachedFlag)
		{
			trajectory.startContinousMove(servoHelper.ReadPositionRadArray(), continousAccelerations, continousSpeeds);
			//trajectory.startContinousCartesianMove(cartesianContinousSpeeds);
		}
		else
		{
			Serial.println("[WARNING] Main: Movement in progress, ignoring command");
		}
		break;
	case waveOut:
		if (enableFlag == false)
		{
			enableTorqueForAll();
			enableFlag = true;
		}
		//enableCartesianContinousLoop = true;
		pose = none;
		enableJointContinousLoop = true;
		switch (currentControlAxis)
		{
		case 1:
			continousSpeeds = { 0.3,0,0 };
			continousAccelerations = { -2,0,0 };
			cartesianContinousSpeeds = {-60, 0,0};
			break;
		case 2:
			continousSpeeds = { 0,0.3,0 };
			continousAccelerations = { 0,-2,0 };
			cartesianContinousSpeeds = {0,-60,0};
			break;
		case 3:
			continousSpeeds = { 0,0,-0.3 };
			continousAccelerations = { 0,0,-2 };
			cartesianContinousSpeeds = {0,0,-60};
			break;
		}
		if (trajectory.goalReachedFlag)
		{
			trajectory.startContinousMove(servoHelper.ReadPositionRadArray(), continousAccelerations, continousSpeeds);
			//trajectory.startContinousCartesianMove(cartesianContinousSpeeds);
		}
		else
		{
			Serial.println("[WARNING] Main: Movement in progress, ignoring command");
		}
		break;
	default:
		break;
	}
}

void waypointIDdecoder()
{
	switch (currentWaypointID)
	{
	case 1:
		control.resetIntegral();
		desiredAngles = { 0,-1.57,1.57 };
		desiredAccelerations = { 5,5,5 };
		currentAngles = servoHelper.ReadPositionRadArray();
		Serial.println(currentAngles[2]);
		trajectory.setNewGoal(currentAngles, desiredAngles, desiredAccelerations, 2000);
		currentWaypointID += 1;
		break;
	case 2:
		control.resetIntegral();
		desiredAngles = { 0,1.57,-1.57 };
		desiredAccelerations = { 3,3,3 };
		currentAngles = servoHelper.ReadPositionRadArray();
		Serial.println(currentAngles[2]);
		trajectory.setNewGoal(currentAngles, desiredAngles, desiredAccelerations, 10000);
		currentWaypointID = 1;
		break;
	}
}

//##############################################################################################################################################################
void setup()
{    
    Serial.begin(115200);
    Serial.setTimeout(100);

    Serial1.begin(3000000);
    Serial1.transmitterEnable(2);

    EEPROM.get(100,cycleFrequency);
    cycleTime = 1000000/cycleFrequency; //in microsec
	for (int i = 0; i < 3; i++)
	{
		EEPROM.get(200 + i * 10, control.Kp[i]);
		EEPROM.get(300 + i * 10, control.Kv[i]);
		EEPROM.get(400 + i * 10, control.Ki[i]);
	}

    for (int i=0;i<5;i++)
    {
        if(!dxl.doPing(i+1, 1000))
        {
            Serial.print("[ERROR] Main: No response from servo #");
            Serial.println(i+1);
        }
    }
    dxl.setOperatingMode(4, pwm);
    dxl.setOperatingMode(5, pwm);
    dxl.torqueEnable(4, 1);
    dxl.torqueEnable(5, 1);
	servoHelper.SoftEstop();
	enableCartesianContinousLoop = 0;
	enableCartesianWaypointLoop = 0;
	enableDebug = 0;
	enableJointContinousLoop = 0;
	enableJointWaypointLoop = 0;
	trajectory.goalReachedFlag = true;

	Serial.println("INITIALIZED");
}

//##############################################################################################################################################################
void loop() 
{
	commandDecoder(); //decodes incoming serial commands
	poseDecoder(); //performes actions based on incoming pose
	
    if(micros()-timer > cycleTime)
    {
        float rateError = micros()-timer-cycleTime;
        unsigned int actualFrequency = 1000000/(micros()-timer);
        if(rateError > cycleTime * 0.1)
        {
            Serial.print("[WARNING]: Missed target rate, actual frequency: ");
            Serial.print(actualFrequency);
            Serial.println(" Hz");
        }
        timer = micros();

		if (trajectory.goalReachedFlag && enableJointWaypointLoop == true) //when waypoint has been reached and it's in joint waypoint mode
		{
			waypointIDdecoder(); //cycles between waypoints
		}
		if (enableJointWaypointLoop && trajectory.goalReachedFlag == 0)
		{
			applyJointWaypointMove();
		}

		if (enableJointContinousLoop)
		{
			applyJointContinousMove();
		}

		if (enableCartesianWaypointLoop)
		{
			applyCartesianWaypointMove();
		}
		if (enableCartesianContinousLoop)
		{
			applyCartesianContinousMove();
		}

		if (enableDebug) debug();
    }
}
