#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include <cmath>


struct Vector3D {
	double x, y, z;

	Vector3D(double xCoord = 0.f, double yCoord = 0.f, double zCoord = 0.f);
	~Vector3D(){};

	Vector3D operator + (const Vector3D& other) const
	{
		return Vector3D(x + other.x, y + other.y, z + other.z);
	}

	Vector3D operator - (const Vector3D& other) const
	{
		return Vector3D(x - other.x, y - other.y, z - other.z);
	}

	Vector3D operator * (const Vector3D& other) const // vector product
	{
		return Vector3D(y * other.z - z * other.y, z * other.x - z * other.z, x * other.y - y * other.x);
	}

	double Length()const
	{
		return sqrt(x * x + y * y + z * z);
	}

	Vector3D Normalized() const
	{
		return ScaledBy(1.f / Length());
	}

	void Normalize()
	{
		ScaleBy(1.0 / Length());
	}

	Vector3D ScaledBy(double scalar)const
	{
		return Vector3D(x * scalar, y * scalar, z * scalar);
	}

	void ScaleBy(double scalar)
	{
		this->x *= scalar;
		this->y *= scalar;
		this->z *= scalar;
	}

	double ScalarProduct(const Vector3D& other) const
	{
		return x * other.x + y * other.y + z * other.z;
	}

};

#endif /* VECTOR3D_H_ */
