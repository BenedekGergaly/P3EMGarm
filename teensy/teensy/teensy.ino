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

#define REST 0
#define WAVE_IN 1
#define WAVE_OUT 2
#define FIST 3
#define FINGER_SPREAD 4
#define DOUBLE_TAP 5
#define POSE_NONE -1
#define MODE_POSITION 3
#define MODE_PWM 16
#define CARTESIAN 0
#define JOINT 1

int pose = POSE_NONE; //rest, waveIn, waveOut, fist, fingerSpread, doubleTap
int currentControlAxis = 3; //1, 2, 3 = x, y, z mode or respective joints is joint mode
bool gripperState = 0; //gripper open/closed
int cycleFrequency; //main loop freq
int cycleTime; //main loop cycle time
unsigned long timer = 0; //main loop timer
bool enableJointWaypointLoop = false;
bool enableJointContinousLoop = false;
bool enableCartesianWaypointLoop = false;
bool enableCartesianContinousLoop = false;
bool enableDebug = false;
int currentWaypointID = 1;
int debugPhase = 1;
int controlMode = JOINT;

array<double, 3> desiredAngles;
array<double, 3> desiredAccelerations;
array<double, 3> currentAngles;

array<double, 3> continousSpeeds = {0,0,0};
array<double, 3> continousAccelerations = {0,0,0};
array<double, 3> cartesianContinousSpeeds = { 0,0,0 };

array<double, 3> userWaypoint = { 0,0,0 };

servo dxl;
ArmKinematics kinematics;
ArmControl control;
ArmTrajectory trajectory = ArmTrajectory(dxl);
ArmDynamics dynamics;
Utilities utilities;
ServoHelper servoHelper = ServoHelper(dxl);

using namespace std;

void applyJointWaypointMove() //called by main loop when joint waypoint is running
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();

	array<array<double, 3>, 3> inputs = trajectory.calculate(); //calculates current cycle of trajectory, note: all parameters are fed in prior to trajectory start
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++) //transfer trajectory outputs to control sytem readable arrays
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}

	//utilities.LogArray("Current position", feedbackPosition);
	//utilities.LogArray("desired position", desiredAngles);
	//utilities.LogArray("Current velocity", feedbackVelocity);
	//utilities.LogArray("desired velocity", desiredSpeeds);

	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	//utilities.LogArray("Torques", torques);
	servoHelper.SendTorquesAllInOne(torques);
	//Serial.println("=================");
	if (trajectory.goalReachedFlag) //when waypoint is reached switch back to continous joint mode
	{
		enableJointWaypointLoop = false;
		enableJointContinousLoop = true;
	}
}

void applyJointContinousMove() //see applyJointWaypointMove() for explanation
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();
	//utilities.LogArray("Current position", feedbackPosition);
	//utilities.LogArray("Current velocity", feedbackVelocity);

	array<array<double, 3>, 3> inputs = trajectory.calculateContinousMove();
	array<double, 3> desiredAngles, desiredSpeeds, desiredAccelerations;
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		desiredSpeeds[i] = inputs[i][1];
		desiredAccelerations[i] = inputs[i][2];
	}
	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
	//utilities.LogArray("Torques", torques);
	//utilities.LogArray("Angles", desiredAngles);
	//utilities.LogArray("speeds", desiredSpeeds);
	//utilities.LogArray("accelerations", desiredAccelerations);
	servoHelper.SendTorquesAllInOne(torques);
	//Serial.println("=================");
}

void applyCartesianWaypointMove()//see applyJointWaypointMove() for explanation			Not used in final product
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

void applyCartesianContinousMove()//see applyJointWaypointMove() for explanation
{
	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();
	//utilities.LogArray("Current position", feedbackPosition);
	//utilities.LogArray("Current velocity", feedbackVelocity);

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
	//Serial.println("=================");
}

//void debug()
//{
//	array<double, 3> feedbackPosition = servoHelper.ReadPositionRadArray();
//	array<double, 3> feedbackVelocity = servoHelper.ReadVelocityRadArray();
//	utilities.LogArray("Current position", feedbackPosition);
//	utilities.LogArray("Current velocity", feedbackVelocity);
//	array<double, 3> desiredSpeeds = { 0,0,0 };
//	array<double, 3> desiredAngles = { 0,0,0 };
//	if (debugPhase == 1)
//	{
//		desiredAngles = { 0,0,0 };
//	}
//	else
//	{
//		desiredAngles = { 0,1.57, -1.57 };
//	}
//
//	array<double, 3> desiredAccelerations = { 0,0,0 };
//	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, desiredSpeeds, desiredAccelerations, feedbackPosition, feedbackVelocity);
//	utilities.LogArray("Angles", desiredAngles);
//	utilities.LogArray("Speeds", desiredSpeeds);
//	utilities.LogArray("Accelerations", desiredAccelerations);
//	utilities.LogArray("Torques", torques);
//	servoHelper.SendTorquesAllInOne(torques);
//	Serial.println("=================");
//}

void printEEPROMvalues() // prints values stored in EEPROM (prints the local copies)
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
        dxl.setOperatingMode(4,MODE_POSITION);
        dxl.setOperatingMode(5,MODE_POSITION);
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
        dxl.setOperatingMode(4,MODE_PWM);
        dxl.setOperatingMode(5,MODE_PWM);
        dxl.torqueEnable(4,1);
        dxl.torqueEnable(5,1);
		dxl.setGoalPwm(4, -150);
		dxl.setGoalPwm(5, -150);
		gripperState = 1;
    }
}

void enableTorqueForAll() //puts servos in pwm mode and enables torque
{
	dxl.torqueEnable(1, 0);
	dxl.torqueEnable(2, 0);
	dxl.torqueEnable(3, 0);
	dxl.setOperatingMode(1, MODE_PWM);
	dxl.setOperatingMode(2, MODE_PWM);
	dxl.setOperatingMode(3, MODE_PWM);
	dxl.torqueEnable(1, 1);
	dxl.torqueEnable(2, 1);
	dxl.torqueEnable(3, 1);
	Serial.println("[INFO] Enabled torque for all servos");
	servoHelper.LEDsOff();
}

void commandDecoder() //decodes commands incoming via serial
{
	if (Serial.available() > 0)
	{
		switch (Serial.read())
		{
		case 'a': //pose input from myoband
			pose = Serial.parseInt();
			break;
		case 'z': //update control mode joint/cartesian CAN ONLY BE USED AFTER CONTROL LOOP IS RUNNING
			if (Serial.read() == 'c')
			{
				controlMode = CARTESIAN;
				enableJointContinousLoop = false;
				enableCartesianContinousLoop = true;
				currentControlAxis = 1;
				Serial.println("Control is now CARTESIAN");
			}
			else
			{
				controlMode = JOINT;
				enableCartesianContinousLoop = false;
				enableJointContinousLoop = true;
				currentControlAxis = 2;
				Serial.println("Control is now JOINT");
			}
			break;
		//misc. debug commands below
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
		case 'n': //update Kp eg: n2 56
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
		case 'm': //update Kv eg: m2 56
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
		case 'c': //update Ki eg: c2 56
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
			dxl.setOperatingMode(1, MODE_PWM);
			dxl.setOperatingMode(2, MODE_PWM);
			dxl.setOperatingMode(3, MODE_PWM);
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
			enableJointWaypointLoop = false;
			enableJointContinousLoop = false;
			enableCartesianWaypointLoop = false;
			enableCartesianContinousLoop = false;
			enableDebug = 0;
			break;
		case 'l': //loosen
			dxl.torqueEnable(1, 0);
			dxl.torqueEnable(2, 0);
			dxl.torqueEnable(3, 0);
			servoHelper.LEDsOff();
			break;
		case 'q': //debug
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
	case DOUBLE_TAP: //advance current control axis/joint
		currentControlAxis += 1;
		if (currentControlAxis == 4) currentControlAxis = 1;
		pose = POSE_NONE;
		utilities.Log("Current axis", currentControlAxis);
		break;
	case FIST://close gripper in cart mode or goes to saved waypoint in joint mode
	{
		if (controlMode == CARTESIAN)
		{
			gripper(1);
		}
		else
		{
			auto currentPosition = servoHelper.ReadPositionRadArray();
			if (trajectory.goalReachedFlag)
			{
				enableJointContinousLoop = false;
				enableJointWaypointLoop = true;
				trajectory.setNewGoal(currentPosition, userWaypoint, { 3,3,3 }, 4000);
			}
			else
			{
				Serial.println("[WARNING] Main: Movement in progress, ignoring command");
			}
		}

		pose = POSE_NONE;
		break;
	}
	case FINGER_SPREAD: //opens gripper in cart mode saves current position as waypoint in joint mode
		if (controlMode == CARTESIAN)
		{
			gripper(0);
		}
		else
		{
			userWaypoint = servoHelper.ReadPositionRadArray();
			Serial.println("Saved new waypoint");
			utilities.LogArray("waypoint", userWaypoint);
		}
		pose = POSE_NONE;
		break;
	case REST: //stops movement
		trajectory.stopContinousMove();
		trajectory.stopContinousCartesianMove();
		break;
	case WAVE_IN: //moves in the positive direction in the current mode (joint/cart)
		if (enableFlag == false) //enables torque on first run
		{
			enableTorqueForAll();
			enableJointContinousLoop = true;
			enableFlag = true;
		}
		pose = POSE_NONE;
		switch (currentControlAxis)
		{
		case 1:
			continousSpeeds = { 0.3,0,0 };
			continousAccelerations = { 2,0,0 };
			cartesianContinousSpeeds = { 30, 0, 0 };
			break;
		case 2:
			continousSpeeds = { 0,0.3,0 };
			continousAccelerations = { 0,2,0 };
			cartesianContinousSpeeds = {0,30,0};
			break;
		case 3:
			continousSpeeds = { 0,0,0.3 };
			continousAccelerations = { 0,0,2 };
			cartesianContinousSpeeds = {0,0,30};
			break;
		}
		if (trajectory.goalReachedFlag)
		{
			trajectory.startContinousMove(servoHelper.ReadPositionRadArray(), continousAccelerations, continousSpeeds);
			trajectory.startContinousCartesianMove(cartesianContinousSpeeds);
		}
		else
		{
			Serial.println("[WARNING] Main: Movement in progress, ignoring command");
		}
		break;
	case WAVE_OUT: //moves in the negative direction in the current mode (joint/cart)
		if (enableFlag == false)//enables torque on first run
		{
			enableTorqueForAll();
			enableJointContinousLoop = true;
			enableFlag = true;
		}
		pose = POSE_NONE;
		switch (currentControlAxis)
		{
		case 1:
			continousSpeeds = { -0.3,0,0 };
			continousAccelerations = { -2,0,0 };
			cartesianContinousSpeeds = {-30, 0,0};
			break;
		case 2:
			continousSpeeds = { 0,-0.3,0 };
			continousAccelerations = { 0,-2,0 };
			cartesianContinousSpeeds = {0,-30,0};
			break;
		case 3:
			continousSpeeds = { 0,0,-0.3 };
			continousAccelerations = { 0,0,-2 };
			cartesianContinousSpeeds = {0,0,-30};
			break;
		}
		if (trajectory.goalReachedFlag)
		{
			trajectory.startContinousMove(servoHelper.ReadPositionRadArray(), continousAccelerations, continousSpeeds);
			trajectory.startContinousCartesianMove(cartesianContinousSpeeds);
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

void waypointIDdecoder() //not used in final product
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
    Serial.begin(115200); //USB serial
    Serial.setTimeout(100);

    Serial1.begin(3000000); //servo serial
    Serial1.transmitterEnable(2);

    EEPROM.get(100,cycleFrequency); //read from EEPROM
    cycleTime = 1000000/cycleFrequency; //in microsec (converts cycleFreq to cycle time)
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
    dxl.setOperatingMode(4, MODE_PWM); //gripper init
    dxl.setOperatingMode(5, MODE_PWM);
    dxl.torqueEnable(4, 1);
    dxl.torqueEnable(5, 1);
	servoHelper.SoftEstop(); //lock position
	enableCartesianContinousLoop = 0;
	enableCartesianWaypointLoop = 0;
	enableDebug = 0;
	enableJointContinousLoop = 0;
	enableJointWaypointLoop = 0;
	trajectory.goalReachedFlag = true; //later checks won't allow setting new goal if this is false

	Serial.println("INITIALIZED");
}

//##############################################################################################################################################################
//MAIN LOOP
void loop() 
{
	commandDecoder(); //decodes incoming serial commands
	poseDecoder(); //performes actions based on incoming pose
	
    if(micros()-timer > cycleTime)
    {
        float rateError = micros()-timer-cycleTime; //cycletime error checking
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
		if (enableJointWaypointLoop && trajectory.goalReachedFlag == false)
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

		//if (enableDebug) debug();
    }
}
