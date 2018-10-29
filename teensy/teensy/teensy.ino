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

using namespace std;

int pose = 0;
int pid[3][3];
int previousError[3];
int integral[3];

int cycleFrequency;
int cycleTime;
unsigned long timer = 0;
double x, y, z;
bool runControlLoop = false;

servo dxl = servo();
ArmKinematics kinematics;
ArmControl control;

void applyTorquesForPosition(double x, double y, double z)
{
	auto point = Point3D<double>(x, y, z);
	auto angles = kinematics.InverseKinematics(point).SolutionTwo;
	angles[1] = 0.0;
	int32_t pos1, pos2, pos3, vel1, vel2, vel3;
	dxl.readPosition(1, pos1);
	dxl.readPosition(2, pos2);
	dxl.readPosition(3, pos3);
	pos1 = control.ConvertPositionSignal(pos1);
	pos2 = control.ConvertPositionSignal(pos2);
	pos3 = control.ConvertPositionSignal(pos3);
	array<double, 3> feedbackPosition = { pos1, pos2, pos3 };
	dxl.readVelocity(1, vel1);
	dxl.readVelocity(2, vel2);
	dxl.readVelocity(3, vel3);
	vel1 = control.ConvertVelocitySignal(vel1);
	vel2 = control.ConvertVelocitySignal(vel2);
	vel3 = control.ConvertVelocitySignal(vel3);
	array<double, 3> feedbackVelocity = { vel1, vel2, vel3 };
	Serial.print("Angles ");
	Serial.print(angles[0]);
	Serial.print(" ");
	Serial.print(angles[1]);
	Serial.print(" ");
	Serial.println(angles[2]);
	Serial.print("Feedback pos ");
	Serial.print(feedbackPosition[0]);
	Serial.print(" ");
	Serial.print(feedbackPosition[1]);
	Serial.print(" ");
	Serial.println(feedbackPosition[2]);
	Serial.print("Feedback vel ");
	Serial.print(feedbackVelocity[0]);
	Serial.print(" ");
	Serial.print(feedbackVelocity[1]);
	Serial.print(" ");
	Serial.println(feedbackVelocity[2]);
	array<double, 3> torques = control.ComputeControlTorque(angles, feedbackPosition, feedbackVelocity);
	Serial.print("Torques ");
	Serial.print(torques[0]);
	Serial.print(" ");
	Serial.print(torques[1]);
	Serial.print(" ");
	Serial.println(torques[2]);
	double current1 = control.ComputeOutputCurrent(torques[0], ServoType::MX106);
	double current2 = control.ComputeOutputCurrent(torques[1], ServoType::MX106);
	double current3 = control.ComputeOutputCurrent(torques[2], ServoType::MX64);
	Serial.print("Currents ");
	Serial.print(current1);
	Serial.print(" ");
	Serial.print(current2);
	Serial.print(" ");
	Serial.println(current3);
	int16_t signal1 = control.ConvertCurrentToSignalValue(current1);
	int16_t signal2 = control.ConvertCurrentToSignalValue(current2);
	int16_t signal3 = control.ConvertCurrentToSignalValue(current3);
	dxl.setGoalCurrent(1, signal1);
	dxl.setGoalCurrent(2, signal2);
	dxl.setGoalCurrent(3, signal3);
	Serial.print("Applying current signals ");
	Serial.print(signal1);
	Serial.print(" ");
	Serial.print(signal1);
	Serial.print(" ");
	Serial.println(signal1);
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
    if(b == 0)
    {

        dxl.torqueEnable(4,0);
        dxl.torqueEnable(5,0);
        dxl.setOperatingMode(4,3);
        dxl.setOperatingMode(5,3);
        dxl.torqueEnable(4,1);
        dxl.torqueEnable(5,1);
        dxl.setGoalPosition(4,2600);
        dxl.setGoalPosition(5,3200);

    }
    else
    {
        dxl.torqueEnable(4,0);
        dxl.torqueEnable(5,0);
        dxl.setOperatingMode(4,16);
        dxl.setOperatingMode(5,16);
        dxl.torqueEnable(4,1);
        dxl.torqueEnable(5,1);
        dxl.setGoalPwm(4, -300);
        dxl.setGoalPwm(5, -300);
    }
}

void setup()
{    
    Serial.begin(115200);
    Serial.setTimeout(100);

    Serial1.begin(115200);
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
            case 'n': //RX dump
                Serial.print("RX dump: ");
                dxl.dumpPackage(dxl.rx_buffer);
                break;
            case 'm': //TX dump
                Serial.print("TX dump: ");
                dxl.dumpPackage(dxl.tx_buffer);
                break;
			case 'k': //Kinematic position
				x = Serial.parseFloat();
				Serial.read();
				y = Serial.parseFloat();
				Serial.read();
				z = Serial.parseFloat();
				dxl.torqueEnable(1, 0);
				dxl.torqueEnable(2, 0);
				dxl.torqueEnable(3, 0);
				dxl.setOperatingMode(1, 0);
				dxl.setOperatingMode(2, 0);
				dxl.setOperatingMode(3, 0);
				dxl.torqueEnable(1, 1);
				dxl.torqueEnable(2, 1);
				dxl.torqueEnable(3, 1);
				runControlLoop = true;
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
		if (runControlLoop)
		{
			applyTorquesForPosition(x, y, z);
		}
        pidCalculate(1,1,1);
    }
}
