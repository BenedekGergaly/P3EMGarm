/*
 * Copyright (c) 2018 Jens Dalsgaard Nielsen
 * Copyright (c) 2018 Karl Damkjær Hansen
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//CORRECT WIRE COLORS: black, green, yellow, blue, red to 3.3v

#include <array>
#include <EEPROM.h>
#include <Arduino.h>
#include "servo.h"
#include "CrustCrawler/ArmKinematics.h"
#include "CrustCrawler/ArmControl.h"
#include "CrustCrawler/ArmTrajectory.h"
#include "CrustCrawler/ArmDynamics.h"

using namespace std;

int pose = 0; //rest, waveIn, waveOut, fist, fingerSpread, doubleTap
int pid[3][3];
int previousError[3];
int integral[3];
int currentControlAxis = 1; //1, 2, 3 = x, y, z mode
bool gripperState = 0;

int cycleFrequency;
int cycleTime;
unsigned long timer = 0;
double x, y, z;
bool runControlLoop = false;
bool testDynamicsFlag = 0;

servo dxl;
ArmKinematics kinematics;
ArmControl control;
ArmTrajectory trajectory;
ArmDynamics dynamics;

void debug()
{
	array<double, 3> startAngles = { 0,0,-0.34 };
	array<double, 3> goalAngles = { 0,0,0.8 };
	array<double, 3> goalAccelerations = { 0,0,1};
	array<double, 3> goalVelocities = { 0,0,2 };
	int desiredTime = 3000;
	ArmTrajectory trajectory;
	trajectory.time = 2500;

	//trajectory.startContinousMove(startAngles, goalAccelerations, goalVelocities);
	trajectory.setNewGoal(startAngles, goalAngles, goalAccelerations, desiredTime);

	while (trajectory.goalReachedFlag == 0)
	{
		trajectory.time += 100;
		//trajectory.calculateContinousMove();
		trajectory.calculate();
		//Serial.print(trajectory.output[2][0]);
		//Serial.print("   ");
		//Serial.print(trajectory.output[2][1]);
		//Serial.print("   ");
		//Serial.println(trajectory.output[2][2]);
		//if (trajectory.time > 5000) trajectory.stopContinousMove();
	}
}

void applyTorquesForPosition(double x, double y, double z)
{
	Serial.print("XYZ: ");
	Serial.print(x);
	Serial.print(", ");
	Serial.print(y);
	Serial.print(", ");
	Serial.println(z);

	auto point = Point3D<double>(x, y, z);
	auto angles = kinematics.InverseKinematics(point).SolutionOne;

	array<double, 3> feedbackPosition = control.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = control.ReadVelocityRadArray();
	control.LogArray("kinematic solution", angles);
	control.LogArray("Current position", feedbackPosition);
	control.LogArray("Current velocity", feedbackVelocity);

	array<array<double, 3>, 3> inputs = trajectory.calculate();
	array<double, 3> desiredAngles = {0, 1.57, -1.57};
	array<double, 3> speeds = {0,0,0};
	array<double, 3> accs = {0,20,0};
	for (int i = 0; i < 3; i++)
	{
		desiredAngles[i] = inputs[i][0];
		speeds[i] = inputs[i][1];
		accs[i] = inputs[i][2];
	}

	array<double, 3> torques = control.ComputeControlTorque(desiredAngles, speeds, accs, feedbackPosition, feedbackVelocity);
	control.LogArray("Torques", torques);

	control.SendTorquesAllInOne(torques);
	//double current1 = control.ComputeOutputCurrent(torques[0], ServoType::MX106);
	//double current2 = control.ComputeOutputCurrent(torques[1], ServoType::MX106);
	//double current3 = control.ComputeOutputCurrent(torques[2], ServoType::MX64);
	//Serial.print("Currents: ");
	//Serial.print(current1);
	//Serial.print(", ");
	//Serial.print(current2);
	//Serial.print(", ");
	//Serial.println(current3);
	//int16_t signal1 = control.ConvertCurrentToSignalValue(current1);
	//int16_t signal2 = control.ConvertCurrentToSignalValue(current2);
	//int16_t signal3 = control.ConvertCurrentToSignalValue(current3);
	//dxl.setGoalCurrent(1, signal1);
	//dxl.setGoalCurrent(2, signal2);
	//dxl.setGoalCurrent(3, signal3);
	//Serial.print("Applying current signals: ");
	//Serial.print(signal1);
	//Serial.print(", ");
	//Serial.print(signal2);
	//Serial.print(", ");
	//Serial.println(signal3);
	Serial.println("=================");
}

void testDynamics(double joint1, double joint2, double joint3)
{
	array<double, 3> feedbackPosition = control.ReadPositionRadArray();
	array<double, 3> feedbackVelocity = control.ReadVelocityRadArray();
	array<double, 3> desiredAngles = { joint1, joint2, joint3 };
	array<double, 3> speeds = { 0, 1, -2 };
	array<double, 3> accs = { 0, 10, -20 };
	auto outputTorque = dynamics.ComputeOutputTorque(accs, feedbackPosition, feedbackVelocity);
	control.SendTorquesAllInOne(outputTorque);
	Serial.print("output torques: ");
	Serial.print(outputTorque[0]);
	Serial.print("   ");
	Serial.print(outputTorque[1]);
	Serial.print("   ");
	Serial.println(outputTorque[2]);
}

void updatePIDvalue(int joint, char letter, int value)
{
    int j;
    switch(letter)
    {
        case 'p':
            j=0;
            break;
        case 'i':
            j=1;
            break;
        case 'd':
            j=2;
            break;
        default:
            Serial.println("ERROR: Invalid parameters");
            return;
    }
    pid[joint][j] = value;
    EEPROM.put(joint*30+j*10, value);
}

void printPIDvalues()
{
    Serial.println("Current PID values:");
    for(int i=0;i<=2;i++)
    {        
        Serial.print("Joint ");
        Serial.print(i+1);
        Serial.print(": ");
        Serial.print(pid[i][0]);
        Serial.print(", ");
        Serial.print(pid[i][1]);
        Serial.print(", ");
        Serial.print(pid[i][2]);
        Serial.print("\n\r");
    }
    Serial.print("Frequency: ");
    Serial.println(cycleFrequency);
	Serial.print("kp, kv: ");
	Serial.print(control.kpTemp);
	Serial.print("   ");
	Serial.println(control.kvTemp);
}

int pidCalculate(int joint, int desired, int actual)
{
    int error = desired - actual;
    integral[joint] = integral[joint] + (error * cycleTime);
    int derivative = (error - previousError[joint]) / cycleTime;
    previousError[joint] = error;
    
    return ((pid[joint][0]*error) + (pid[joint][1]*integral[joint]) + (pid[joint][2]*derivative));
}

void gripper(bool b)
{
	if (b == 0 && gripperState == 1)
    {

        dxl.torqueEnable(4,0);
        dxl.torqueEnable(5,0);
        dxl.setOperatingMode(4,3);
        dxl.setOperatingMode(5,3);
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
        dxl.setOperatingMode(4,16);
        dxl.setOperatingMode(5,16);
        dxl.torqueEnable(4,1);
        dxl.torqueEnable(5,1);
		dxl.setGoalPwm(4, -150);
		dxl.setGoalPwm(5, -150);
		gripperState = 1;
    }
}

void setup()
{    
    Serial.begin(115200);
    Serial.setTimeout(100);

    Serial1.begin(1000000);
    Serial1.transmitterEnable(2);

    int eeaddress = 0;
    for (int i=0;i<=2;i++)
    {
        for (int j=0;j<=2;j++)
        {
            EEPROM.get(eeaddress, pid[i][j]);
            eeaddress += 10;
        }
    }

    EEPROM.get(100,cycleFrequency);
    cycleTime = 1000000/cycleFrequency; //in microsec
	EEPROM.get(200, control.kpTemp);
	EEPROM.get(300, control.kvTemp);

    for (int i=0;i<5;i++)
    {
        if(!dxl.doPing(i+1, 1000))
        {
            Serial.print("ERROR: No response from servo #");
            Serial.println(i+1);
        }
    }
    dxl.setOperatingMode(4, 16);
    dxl.setOperatingMode(5, 16);
    dxl.torqueEnable(4, 1);
    dxl.torqueEnable(5, 1);
	Serial.println("INITIALIZED");
}

void loop() {
    if (Serial.available() > 0)
    {
        switch (Serial.read())
        {
            case 'a': //pose input
                pose = Serial.parseInt();
                break;
            case 'j': //update pid value j2p123
                {
                int joint = Serial.parseInt();
                char pid = Serial.read();
                int value = Serial.parseInt();
                updatePIDvalue(joint-1, pid, value);
                break;
                }
            case 'r': //retrieve pid values
                printPIDvalues();
                break;
            case 'f': //change frequency
                cycleFrequency = Serial.parseInt();
                EEPROM.put(100,cycleFrequency);
                cycleTime = 1000000/cycleFrequency;
                break;
            case 't': //torque enable/disable t3,1  joint3 enable
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
                int32_t position;
                dxl.readPosition(id, position);
                Serial.print("Position of servo #");
                Serial.print(id);
                Serial.print(" : ");
                Serial.println(position);
                break;
                }
            case 'c': //set goal current
                {
                int id = Serial.parseInt();
                Serial.read();
                int current = Serial.parseInt();
                Serial.println(dxl.setGoalCurrent(id, current));
                Serial.println(current);
                break;
                }
            case 'o': //set operating mode 0=current, 3=position
                {
                int id = Serial.parseInt();
                Serial.read();
                int mode = Serial.parseInt();
                dxl.setOperatingMode(id, mode);
                break;
                }
            case 'g': //goal position
                {
                int id = Serial.parseInt();
                Serial.read();
                int32_t pos = Serial.parseInt();
                dxl.setGoalPosition(id, pos);
                break;
                }
            case 'w': //goal pwm
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
            case 'n':
			{
				double temp = Serial.parseFloat();
				Serial.print("new kp is: ");
				Serial.println(temp);
				EEPROM.put(200, temp);
				control.kpTemp = temp;
				break;
			}
            case 'm':
			{
				double temp = Serial.parseFloat();
				Serial.print("new kv is: ");
				Serial.println(temp);
				EEPROM.put(300, temp);
				control.kvTemp = temp;
				break;
			}
			case 'k': //Kinematic position
			{
				//x = Serial.parseFloat();
				x = -505;
				//Serial.read();
				//y = Serial.parseFloat();
				y = 0;
				//Serial.read();
				//z = Serial.parseFloat();
				z = 240;
				dxl.torqueEnable(1, 0);
				dxl.torqueEnable(2, 0);
				dxl.torqueEnable(3, 0);
				dxl.setOperatingMode(1, 0);
				dxl.setOperatingMode(2, 0);
				dxl.setOperatingMode(3, 0);
				//dxl.torqueEnable(1, 1);
				dxl.torqueEnable(2, 1);
				dxl.torqueEnable(3, 1);
				runControlLoop = true;
				array<double, 3> desiredAngles = {0,-1.57,1.57};
				array<double, 3> desiredAccelerations = {0,1,1};
				array<double, 3> currentAngles = control.ReadPositionRadArray();
				Serial.println(currentAngles[2]);
				trajectory.setNewGoal(currentAngles, desiredAngles, desiredAccelerations, 3000);
				break;
			}
			case 'h':
				dxl.torqueEnable(1, 0);
				dxl.torqueEnable(2, 0);
				dxl.torqueEnable(3, 0);
				dxl.setOperatingMode(1, 0);
				dxl.setOperatingMode(2, 0);
				dxl.setOperatingMode(3, 0);
				//dxl.torqueEnable(1, 1);
				dxl.torqueEnable(2, 1);
				dxl.torqueEnable(3, 1);
				testDynamicsFlag = 1;
				break;
			case 'd':
				debug();
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

	switch (pose)
	{
	case 5:
		currentControlAxis += 1;
		if (currentControlAxis == 4) currentControlAxis = 1;
		pose = 0;
		break;
	case 3:
		gripper(1);
		pose = 0;
		break;
	case 4:
		gripper(0);
		pose = 0;
		break;
	default:
		break;
	}

    if(micros()-timer > cycleTime)
    {
        float rateError = micros()-timer-cycleTime;
        unsigned int actualFrequency = 1000000/(micros()-timer);
        if(rateError > cycleTime * 0.1)
        {
            Serial.print("WARNING: Missed target rate, actual frequency: ");
            Serial.print(actualFrequency);
            Serial.println(" Hz");
        }
        timer = micros();
		if (runControlLoop && trajectory.goalReachedFlag == 0)
		{
			applyTorquesForPosition(x, y, z);
		}
		if (runControlLoop && trajectory.goalReachedFlag == 1)
		{
			control.SoftEstop();
			runControlLoop = 0;
		}
		if (testDynamicsFlag)
		{
			testDynamics(0, 0, 0);
		}
		//control.CheckOverspeed(1.2);
        //pidCalculate(1,1,1);
    }
}
