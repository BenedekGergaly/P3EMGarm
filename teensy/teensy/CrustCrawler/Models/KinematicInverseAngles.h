// KinematicInverseAngles.h

#ifndef _KINEMATICINVERSEANGLES_h
#define _KINEMATICINVERSEANGLES_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class KinematicInverseAngles
{
public:
	double SolutionOne[3];
	double SolutionTwo[3];
};

#endif

