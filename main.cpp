#include "KDTree.h"
#include "Vector3D.h"

#include <iostream>


int main()
{
	int n = 100000;
	vector<Sphere> spheres(n);

	for(int i = 0; i < n; ++i)
	{
		spheres[i].center = Vector3D(i, i * 2, i * 3);
		spheres[i].radius = 5 * (i % 10) + 1;
	}

	KDTree t;
	t.BuildTree(spheres);

	return 0;
}

