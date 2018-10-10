// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

// This sample illustrates how to use EMG data. EMG streaming is only supported for one Myo at a time.

#include <array>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <stdio.h>

#include <myo/myo.hpp>

#include <SerialPort.h>


class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : emgSamples()
    {
    }

    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        emgSamples.fill(0);
    }

    // onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
    {
        for (int i = 0; i < 8; i++) {
            emgSamples[i] = emg[i];
        }
    }

	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose input)
	{
		pose = input.toString();
		myo->unlock(myo::Myo::unlockHold);
		std::cout << pose << std::endl;
	}

    void print()
    {
        // Clear the current line
        std::cout << '\r';

        // Print out the EMG data.
        /*for (size_t i = 0; i < emgSamples.size(); i++) {
            std::ostringstream oss;
            oss << static_cast<int>(emgSamples[i]);
            std::string emgString = oss.str();

            std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
        }*/

        std::cout << std::flush;
    }

	std::array<int8_t, 8> emgSamples;
	std::string pose;
};


int main(int argc, char** argv)
{
    try {

	std::ifstream configFile;
	std::string port;
	configFile.open("../config.txt");
	if (!configFile.is_open())
	{
		std::cout << "ERROR: Can't open config file! Trying COM3" << std::endl;
		port = "COM3";
	}
	else
	{
		std::getline(configFile, port);
		std::cout << "Using " << port << std::endl;
	}
	configFile.close();

	char serialOutput[64];
	char serialInput[64];
	std::string temp = "\\\\.\\" + port;
	const char* temp2 = temp.c_str();
	char* portName = (char*)temp2;

	SerialPort teensy(portName);


    myo::Hub hub("com.example.emg-data-sample");

    std::cout << "Attempting to find a Myo..." << std::endl;
    myo::Myo* myo = hub.waitForMyo(10000);
    if (!myo) {
        throw std::runtime_error("Unable to find a Myo!");
    }

    std::cout << "Connected to a Myo armband!" << std::endl << std::endl;

    myo->setStreamEmg(myo::Myo::streamEmgEnabled);

    DataCollector collector;

    // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
    // Hub::run() to send events to all registered device listeners.
    hub.addListener(&collector);

    // Finally we enter our main loop.
    while (1) {
        // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
        // In this case, we wish to update our display 50 times a second, so we run for 1000/20 milliseconds.
        hub.run(1000/20);
        collector.print();

		if (collector.pose != "")
		{
			std::string command;
			if (collector.pose == "rest") command = "a0,";
			if (collector.pose == "waveIn") command = "a1,";
			if (collector.pose == "waveOut") command = "a2,";
			if (collector.pose == "fist") command = "a3,";
			if (collector.pose == "fingersSpread") command = "a4,";
			if (collector.pose == "doubleTap") command = "a5,";

			teensy.writeSerialPort(command);
			std::cout << "sent: " << command << std::endl;
			collector.pose = "";
		}
    }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore();
        return 1;
    }
}
