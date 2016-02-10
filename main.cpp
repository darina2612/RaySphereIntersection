#include "IntersectionInterface.h"

#include <iostream>


int main()
{
	int n = 100000;
	int rc = 3000000;
	vector<Sphere> spheres(n);
	vector<Ray> rays(rc);
	vector<char> doesIntersect(rc);
	vector<Vector3D> points(rc);

	for(int i = 0; i < n; ++i)
	{
		spheres[i].center = Vector3D(i, i, i);
		spheres[i].radius = 5;// * (i % 10) + 1;

	}

	for(int i = 0; i < rc; ++i)
	{
		rays[i].origin = {100, 100, 100};
		rays[i].dir = {0.5, 0.5, 0.5};
	}

	IntersectRaysWithSpheres(rays, spheres, points, doesIntersect);
	int intersectionsCount = 0;
	for(int i = 0; i < rc; ++i)
	{
		if(doesIntersect[i]) ++intersectionsCount;
	}

	std::cout << intersectionsCount << std::endl;

	//std::cout << t.NodesNumber() << ' ' << t.LeavesNumber();
	return 0;

}

