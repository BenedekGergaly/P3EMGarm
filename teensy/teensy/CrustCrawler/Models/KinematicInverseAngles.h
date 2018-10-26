// KinematicInverseAngles.h

#ifndef _KINEMATICINVERSEANGLES_h
#define _KINEMATICINVERSEANGLES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <array>

class KinematicInverseAngles
{
public:
	std::array<double, 3> SolutionOne;
	std::array<double, 3> SolutionTwo;
};

#endif

