//CORRECT WIRE COLORS: black, green, yellow, blue, red to 3.3v

#include <array>
#include <EEPROM.h>
#include <Arduino.h>
#include "servo.h"
#include "CrustCrawler/ArmKinematics.h"
#include "CrustCrawler/ArmControl.h"
#include "CrustCrawler/ArmTrajectory.h"
#include "CrustCrawler/ArmDynamics.h"

constexpr auto rest = 0;
constexpr auto waveIn = 1;
constexpr auto waveOut = 2;
constexpr auto fist = 3;
constexpr auto fingerSpread = 4;
constexpr auto doubleTap = 5;
constexpr auto none = -1;
constexpr auto position = 3;
constexpr auto pwm = 16;

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
int currentWaypointID = 1;

array<double, 3> desiredAngles;
array<double, 3> desiredAccelerations;
array<double, 3> currentAngles;

array<double, 3> continousSpeeds = {0,0,0};
array<double, 3> continousAccelerations = {0,0,0};

servo dxl;
ArmKinematics kinematics;
ArmControl control;
ArmTrajectory trajectory;
ArmDynamics dynamics;

using namespace std;

void applyJointWaypointMove()
{
	array<double, 3> feedbackPosition = control.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = control.ReadVelocityRadArray();
	control.LogArray("Current position", feedbackPosition);
	control.LogArray("Current velocity", feedbackVelocity);

	array<array<double, 3>, 3> inputs = trajectory.calculate();
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}

	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	control.LogArray("Torques", torques);
	control.SendTorquesAllInOne(torques);
	Serial.println("=================");
}

void applyJointContinousMove()
{
	array<double, 3> feedbackPosition = control.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = control.ReadVelocityRadArray();
	control.LogArray("Current position", feedbackPosition);
	control.LogArray("Current velocity", feedbackVelocity);

	array<array<double, 3>, 3> inputs = trajectory.calculateContinousMove();
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}
	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	control.LogArray("Torques", torques);
	control.LogArray("Angles", desiredAngles);
	control.LogArray("speeds", desiredSpeeds);
	control.LogArray("accelerations", desiredAccelerations);
	control.SendTorquesAllInOne(torques);
	Serial.println("=================");
}

void applyCartesianWaypointMove()
{
	array<double, 3> feedbackPosition = control.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = control.ReadVelocityRadArray();
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
	control.SendTorquesAllInOne(torques);
	//Serial.println("=================");
}

void printEEPROMvalues()
{
    Serial.print("Frequency: ");
    Serial.println(cycleFrequency);
	Serial.print("kp, kv: ");
	Serial.print(control.Kp);
	Serial.print("   ");
	Serial.println(control.Kv);
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
			double position = control.ReadPositionRad(id);
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
			double temp = Serial.parseFloat();
			Serial.print("new kp is: ");
			Serial.println(temp);
			EEPROM.put(200, temp);
			control.Kp = temp;
			break;
		}
		case 'm': //update Kv
		{
			double temp = Serial.parseFloat();
			Serial.print("new kv is: ");
			Serial.println(temp);
			EEPROM.put(300, temp);
			control.Kv = temp;
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
			array<double, 3> cartPos = kinematics.ForwardKinematics(control.ReadPositionRad(1), control.ReadPositionRad(2),
				control.ReadPositionRad(3)).getArray();
			control.LogArray("EE cartesian position", cartPos);
			control.LogArray("Inverse kinematic solution 1: ", kinematics.InverseKinematics(trajectory.arrayToPoint(cartPos)).SolutionOne);
			control.LogArray("Inverse kinematic solution 2: ", kinematics.InverseKinematics(trajectory.arrayToPoint(cartPos)).SolutionTwo);
			break;
		}
		case 's': //stop
			control.SoftEstop();
			enableJointWaypointLoop = 0;
			enableJointContinousLoop = 0;
			break;
		case 'l': //loosen
			dxl.torqueEnable(1, 0);
			dxl.torqueEnable(2, 0);
			dxl.torqueEnable(3, 0);
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
	switch (pose)
	{
	case doubleTap:
		currentControlAxis += 1;
		if (currentControlAxis == 4) currentControlAxis = 1;
		pose = none;
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
		enableJointContinousLoop = true;
		pose = none;
		switch (currentControlAxis)
		{
		case 1:
			continousSpeeds = { 0.3,0,0 };
			continousAccelerations = { 2,0,0 };
			break;
		case 2:
			continousSpeeds = { 0,0.3,0 };
			continousAccelerations = { 0,2,0 };
			break;
		case 3:
			continousSpeeds = { 0,0,0.3 };
			continousAccelerations = { 0,0,2 };
			break;
		}
		if (trajectory.goalReachedFlag)
		{
			trajectory.startContinousMove(control.ReadPositionRadArray(), continousAccelerations, continousSpeeds);
		}
		else
		{
			Serial.println("[WARNING] Main: Movement in progress, ignoring command");
		}
		break;
	case waveOut:
		pose = none;
		enableJointContinousLoop = true;
		switch (currentControlAxis)
		{
		case 1:
			continousSpeeds = { 0.3,0,0 };
			continousAccelerations = { -2,0,0 };
			break;
		case 2:
			continousSpeeds = { 0,0.3,0 };
			continousAccelerations = { 0,-2,0 };
			break;
		case 3:
			continousSpeeds = { 0,0,-0.3 };
			continousAccelerations = { 0,0,-2 };
			break;
		}
		if (trajectory.goalReachedFlag)
		{
			trajectory.startContinousMove(control.ReadPositionRadArray(), continousAccelerations, continousSpeeds);
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
		desiredAngles = { 0,-1.57,1.57 };
		desiredAccelerations = { 3,15,15 };
		currentAngles = control.ReadPositionRadArray();
		Serial.println(currentAngles[2]);
		trajectory.setNewGoal(currentAngles, desiredAngles, desiredAccelerations, 1500);
		currentWaypointID += 1;
		break;
	case 2:
		desiredAngles = { 0,0,0 };
		desiredAccelerations = { 3,3,3 };
		currentAngles = control.ReadPositionRadArray();
		Serial.println(currentAngles[2]);
		trajectory.setNewGoal(currentAngles, desiredAngles, desiredAccelerations, 3000);
		currentWaypointID += 1;
		break;
	case 3:
		desiredAngles = { 0,1.57,-1.57 };
		desiredAccelerations = { 3,15,15 };
		currentAngles = control.ReadPositionRadArray();
		Serial.println(currentAngles[2]);
		trajectory.setNewGoal(currentAngles, desiredAngles, desiredAccelerations, 3000);
		currentWaypointID = 1;
		break;
	}
}

//##############################################################################################################################################################
void setup()
{    
    Serial.begin(115200);
    Serial.setTimeout(100);

    Serial1.begin(1000000);
    Serial1.transmitterEnable(2);

    EEPROM.get(100,cycleFrequency);
    cycleTime = 1000000/cycleFrequency; //in microsec
	EEPROM.get(200, control.Kp);
	EEPROM.get(300, control.Kv);

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
	control.SoftEstop();
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
    }
}
