#include "Vector3D.h"

#include <iostream>


int main()
{
	Vector3D v(1.f, 2.f, 3.f);
	v.ScaleBy(3.f);

	std::cout << v.x << " " << v.y << " " << v.z << std::endl;

	std::cout << v.Length();
	return 0;
}

