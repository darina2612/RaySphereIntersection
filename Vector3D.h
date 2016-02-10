#ifndef VECTOR3D_H_
#define VECTOR3D_H_

#include <cmath>
#include <vector>

using std::vector;


typedef __attribute__ ((vector_size(4 * sizeof(float)))) float floatVector4;

struct Vector3D
{

	union{
		struct{ float x, y, z; };
		floatVector4 vectored;
	};

	Vector3D(float xCoord = 0.f, float yCoord = 0.f, float zCoord = 0.f)
	{
		x = xCoord;
		y = yCoord;
		z = zCoord;
	}

	Vector3D(floatVector4 vectoredCoords)
	{
		vectored = vectoredCoords;
	}

	~Vector3D(){};

	float& operator [] (const int index)
	{
		return vectored[index];
	}

	const float& operator [] (const int index) const
	{
		return vectored[index];
	}

	Vector3D operator + (const Vector3D& other) const
	{
		return Vector3D(vectored + other.vectored);
	}

	Vector3D operator - (const Vector3D& other) const
	{
		return Vector3D(vectored - other.vectored);
	}


	Vector3D operator * (const Vector3D& other) const // multiplication coordinate by coordinate
	{
		return Vector3D(vectored * other.vectored);
	}

	float Length()const
	{
		floatVector4 result = vectored * vectored;
		return sqrt(result[0] + result[1] + result[2]);
	}

	Vector3D Normalized() const
	{
		return ScaledBy(1.f / Length());
	}

	void Normalize()
	{
		ScaleBy(1.0 / Length());
	}

	Vector3D ScaledBy(float scalar)const
	{
		return Vector3D(scalar * vectored);
	}

	void ScaleBy(float scalar)
	{
		vectored = scalar * vectored;
	}

	float ScalarProduct(const Vector3D& other) const
	{
		floatVector4 product = vectored + other.vectored;
		return product[0] + product[1] + product[2];
	}

	bool operator == (const Vector3D& other)const
	{
		return (x == other.x) && (y == other.y) && (z == other.z);
	}

};


struct Sphere
{
	Vector3D center;
	float radius;

	Sphere(){};
	Sphere(Vector3D c, double r) : center(c), radius(r) {};

	bool operator == (const Sphere& other) const
	{
		return (center == other.center) && (radius ==  other.radius);
	}
};


struct Ray
{
	Vector3D origin, dir;
	Ray(){};
	Ray(Vector3D o, double d) : origin(o), dir(d) {};
};


struct Rays
{
	vector<float> startXs, startYs, startZs, dirXs, dirYs, dirZs;
};


struct Spheres
{
	vector<float> centerXs, centerYs, centerZs, radiuses;
};


#endif /* VECTOR3D_H_ */
