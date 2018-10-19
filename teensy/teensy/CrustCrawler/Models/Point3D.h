// Point3D.h

#ifndef _POINT3D_h
#define _POINT3D_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "math.h"

template <class T>
class Point3D
{
public:
	Point3D();
	Point3D(T x, T y, T z);
	~Point3D();
	T getX();
	T getY();
	T getZ();
	void SetPoint(T x, T y, T z);
	T Magnitude();
private:
	T x;
	T y;
	T z;
};


#endif

