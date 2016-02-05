#include "Vector3D.h"

#include <cmath>


Vector3D::Vector3D(double xCoord, double yCoord, double zCoord) : x(xCoord), y(yCoord), z(zCoord)
{
}


Vector3D Vector3D::operator + (const Vector3D& other) const
{
	return Vector3D(this->x + other.x, this->y + other.y, this->z + other.z);
}


Vector3D Vector3D::operator - (const Vector3D& other) const
{
	return Vector3D(this->x - other.x, this->y - other.y, this->z - other.z);
}


Vector3D Vector3D::operator * (const Vector3D& other) const
{
	return Vector3D(this->y * other.z - this->z * other.y, this->z * other.x - this->z * other.z,
			this->x * other.y - this->y * other.x);
}


double Vector3D::Length()const
{
	return sqrt(this->x * this->x + this->y * this->y + this->z * this->z);
}


Vector3D Vector3D::Normalized() const
{
	return this->ScaledBy(1.f / this->Length());
}


void Vector3D::Normalize()
{
	this->ScaleBy(1.0 / this->Length());
}


Vector3D Vector3D::ScaledBy(double scalar)const
{
	return Vector3D(this->x * scalar, this->y * scalar, this->z * scalar);
}

void Vector3D::ScaleBy(double scalar)
{
	this->x *= scalar;
	this->y *= scalar;
	this->z *= scalar;
}


double Vector3D::ScalarProduct(const Vector3D& other) const
{
	return this->x * other.x + this->y * other.y + this->z * other.z;
}
