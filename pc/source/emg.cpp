// Copyright (C) 2013-2014 Thalmic Labs Inc.
// Distributed under the Myo SDK license agreement. See LICENSE.txt for details.

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <stdio.h>
#include <string>

#include <myo/myo.hpp>

#include <SerialPort.h>

class DataCollector : public myo::DeviceListener
{
public:
	DataCollector() {}

	void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose input)
	{
		pose = input.toString(); // pose to string
		myo->unlock(myo::Myo::unlockHold); // stays unlocked forever
		std::cout << pose << std::endl; // log pose
	}
	std::string pose;
};

int main(int argc, char** argv)
{
	try
	{
		// open config file containing port name
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
		std::string temp = "\\\\.\\" + port;
		char* portName = (char*)temp.c_str(); // convert string to char*

		SerialPort teensy(portName);
		DataCollector collector;
		myo::Hub hub("com.example.emg-data-sample");

		std::cout << "Attempting to find a Myo..." << std::endl;
		myo::Myo* myo = hub.waitForMyo(10000);
		if (!myo)
		{
			throw std::runtime_error("Unable to find a Myo!");
		}
		std::cout << "Connected to a Myo armband!" << std::endl
			<< std::endl;

		// Hub::addListener() takes the address of any object whose class inherits
		// from DeviceListener, and will cause Hub::run() to send events to all
		// registered device listeners.
		hub.addListener(&collector);

		while (1)
		{
			hub.run(1000 / 20); // run at 50Hz

			if (collector.pose != "")
			{
				std::string command;
				if (collector.pose == "rest")
					command = "a0";
				if (collector.pose == "waveIn")
					command = "a1";
				if (collector.pose == "waveOut")
					command = "a2";
				if (collector.pose == "fist")
					command = "a3";
				if (collector.pose == "fingersSpread")
					command = "a4";
				if (collector.pose == "doubleTap")
					command = "a5";

				teensy.writeSerialPort(command);
				std::cout << "sent: " << command << std::endl;
				collector.pose = "";
			}
		}

	}
	catch (const std::exception& e)
	{
		std::cerr << "Error: " << e.what() << std::endl;
		std::cerr << "Press enter to continue.";
		std::cin.ignore();
		return 1;
	}
}
