#include "Point3D.h"

template <class T>
Point3D<T>::Point3D()
{
	x = 0;
	y = 0;
	z = 0;
}

template<class T>
Point3D<T>::Point3D(T x, T y, T z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

template <class T>
Point3D<T>::~Point3D()
{

}

template<class T>
T Point3D<T>::getX()
{
	return x;
}

template<class T>
T Point3D<T>::getY()
{
	return y;
}

template<class T>
T Point3D<T>::getZ()
{
	return z;
}

template<class T>
T Point3D<T>::Magnitude()
{
	return (T)sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
}

template<class T>
void Point3D<T>::SetPoint(T x, T y, T z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}
