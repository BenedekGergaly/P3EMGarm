// Point3D.h

#ifndef _POINT3D_h
#define _POINT3D_h

#include "math.h"
#include <array>

using namespace std;


//Template class used for representing a 3D point in Cartesian space
template <typename T>
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
	array<T, 3> getArray();
private:
	T x;
	T y;
	T z;
};

template <typename T>
Point3D<T>::Point3D()
{
	x = 0;
	y = 0;
	z = 0;
}

template<typename T>
Point3D<T>::Point3D(T x, T y, T z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

template <typename T>
Point3D<T>::~Point3D()
{

}

template<typename T>
T Point3D<T>::getX()
{
	return x;
}

template<typename T>
T Point3D<T>::getY()
{
	return y;
}

template<typename T>
T Point3D<T>::getZ()
{
	return z;
}

template<typename T>
T Point3D<T>::Magnitude()
{
	return (T)sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

template<typename T>
inline array<T, 3> Point3D<T>::getArray()
{
	array<T, 3> o = { x,y,z };
	return o;
}

template<typename T>
void Point3D<T>::SetPoint(T x, T y, T z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}


#endif

