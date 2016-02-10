#include "KDTree.h"

#include <iostream>


int main()
{
	int n = 10000000;
	vector<Sphere> spheres(n);

	for(int i = 0; i < n; ++i)
	{
		spheres[i].center = Vector3D(i, i, i);
		spheres[i].radius = 5;// * (i % 10) + 1;
	}

	KDTree t;
	t.BuildTree(spheres);
	Vector3D p;
	Ray r;
	r.origin = {0, 0, 0};
	r.dir = {0.5, 0.5, 0.5};

	if(t.Intersect(r, p))
	{
		std::cout << p.x << " " << p.y << " " << p.z << std::endl;
	}
	//std::cout << t.NodesNumber() << ' ' << t.LeavesNumber();
	return 0;

}

