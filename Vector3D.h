#ifndef VECTOR3D_H_
#define VECTOR3D_H_

class Vector3D {
public:
	Vector3D(double xCoord = 0.f, double yCoord = 0.f, double zCoord = 0.f);
	virtual ~Vector3D(){};

	Vector3D operator + (const Vector3D& other) const;
	Vector3D operator - (const Vector3D& other) const;
	Vector3D operator * (const Vector3D& other) const; // vector product

	double Length()const;
	Vector3D Normalized() const;
	void Normalize();
	Vector3D ScaledBy(double scalar)const;
	void ScaleBy(double scalar);
	double ScalarProduct(const Vector3D& other) const;

private:
	double x, y, z;
};

#endif /* VECTOR3D_H_ */
