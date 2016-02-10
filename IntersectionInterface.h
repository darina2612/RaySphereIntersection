#ifndef INTERSECTIONINTERFACE_H_
#define INTERSECTIONINTERFACE_H_

#include "KDTree.h"

#include <thread>

const int raysPerThread = 1000000;

KDTree tree;

bool IntersectRayWithSpheres(const Ray& ray, const vector<Sphere>& spheres, Vector3D& intersectionPoint)
{
	tree.BuildTree(spheres);

	return tree.Intersect(ray, intersectionPoint);
}


void RaysIntersection(const vector<Ray>& rays, const vector<Sphere>& spheres,
		vector<Vector3D>& intersectionPoints, vector<char>& doesIntersect, int start, int end)
{
	for(unsigned i = start; i < end; ++i)
	{
		doesIntersect[i] = tree.Intersect(rays[i], intersectionPoints[i]);
	}
}

using std::thread;

void IntersectRaysWithSpheres(const vector<Ray>& rays, const vector<Sphere>& spheres,
		vector<Vector3D>& intersectionPoints, vector<char>& doesIntersect)
{
	if(spheres != tree.GetSpheres())
	{
		tree.BuildTree(spheres);
	}

	int numThreads = rays.size() / raysPerThread;

	vector<thread> threads;
	if(numThreads > 1)
	{
		threads.resize(numThreads);
		for(int i = 1; i <= numThreads; ++i)
		{
			threads[i - 1] = thread(&RaysIntersection, std::cref(rays), std::cref(spheres),
						 std::ref(intersectionPoints), std::ref(doesIntersect),
						 (i - 1) * raysPerThread, i * raysPerThread);
		}

	}

	RaysIntersection(rays, spheres, intersectionPoints, doesIntersect,raysPerThread * numThreads, rays.size());

	if(numThreads > 1)
	{
		for(int i = 0; i < numThreads; ++i)
		{
			threads[i].join();
		}
	}

}

#endif /* INTERSECTIONINTERFACE_H_ */
