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

#include <EEPROM.h>
#include <Arduino.h>
#include "servo.h"

int pose = 0;
int pid[3][3];
int previousError[3];
int integral[3];

int cycleFrequency;
int cycleTime;
unsigned long timer = 0;

servo dxl = servo();

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
            case 'n': //RX dump
                Serial.print("RX dump: ");
                dxl.dumpPackage(dxl.rx_buffer);
                break;
            case 'm': //TX dump
                Serial.print("TX dump: ");
                dxl.dumpPackage(dxl.tx_buffer);
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
        pidCalculate(1,1,1);
    }
}
