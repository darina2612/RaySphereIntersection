#include "KDTree.h"
#include <stack>
#include <limits>
#include <thread>
#include <iostream>
#include <algorithm>

using std::min;
using std::max;
using std::stack;


bool KDTree::Intersect(const Ray& ray, Vector3D& intersectionPoint) const
{
	// used for help here http://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
	float tmin, tmax;
	Vector3D inversedDir;
	inversedDir.vectored = 1.f / ray.dir.vectored;

	Vector3D minPlanesIntersections, maxPlanesIntersections;
	minPlanesIntersections = (this->sceneBox.minPoint - ray.origin) * inversedDir;
	maxPlanesIntersections = (this->sceneBox.maxPoint - ray.origin) * inversedDir;


	tmin = max(max(min(minPlanesIntersections[0], maxPlanesIntersections[0]),
			min(minPlanesIntersections[1], maxPlanesIntersections[1])),
			min(minPlanesIntersections[2], maxPlanesIntersections[2]));
	tmax = min(min(max(minPlanesIntersections[0], maxPlanesIntersections[0]),
			max(minPlanesIntersections[1], maxPlanesIntersections[1])),
			max(minPlanesIntersections[2], maxPlanesIntersections[2]));

	// if tmax < 0, the scene box is "behing" the ray
	if(tmax < 0) return false;

	// if tmin > tmax, ray doesn't intersect scene bounding box
	if(tmin > tmax) return false;

	float closestIntersectionPointT;

	if(this->Traverse(ray, tmin, tmax, closestIntersectionPointT))
	{
		intersectionPoint = ray.origin + ray.dir.ScaledBy(closestIntersectionPointT);
		return true;
	}

	return false;
}


bool KDTree::Traverse(const Ray& ray, float tnear, float tfar, float& intersectionPointT) const
{
	struct StackNode
	{
		int index;
		float tmin, tmax;

		StackNode(int i, float min, float max) : index(i), tmin(min), tmax(max){}
	};

	stack<StackNode> s;

	float d;
	Axis axis;
	unsigned currentIndex = 0;
	while(true)
	{
		while(!isLeaf(this->tree[currentIndex]))
		{
			axis = splitAxis(this->tree[currentIndex]);
			d = (this->tree[currentIndex].inner.splitCoordinate - ray.origin[axis]) / ray.dir[axis];

			unsigned leftChild = currentIndex + (firstChildOffset(this->tree[currentIndex]) / sizeof(Node));
			unsigned rightChild = leftChild + 1;
			if(d <= tnear)
			{
				currentIndex = rightChild;
			}
			else if(d >= tfar)
			{
				currentIndex = leftChild;
			}
			else
			{
				s.push(StackNode(rightChild, d, tfar));
				currentIndex = leftChild;
				tfar = d;
			}
		}

		if(this->ClosestIntersectionPointInLeaf(currentIndex, ray, intersectionPointT)) return true;

		if(tfar >= intersectionPointT)
			return false;
		if (s.empty())
			return false;

		StackNode n = s.top();
		s.pop();
		currentIndex = n.index;
		tnear = n.tmin;
		tfar = n.tmax;
	}
}



//mainly using help from http://www.vis.uky.edu/~ryang/teaching/cs535-2012spr/Lectures/13-RayTracing-II.pdf
bool KDTree::ClosestIntersectionPointInLeaf(unsigned leafIndex, const Ray& ray, float& closestPointT) const
{
	unsigned spheresIndexesIndex = this->tree[leafIndex].leaf.flagDimentionOffset;
	spheresIndexesIndex &= (unsigned)~(1 << 31);

	int numSpheres = this->leafSpheresIndexes[spheresIndexesIndex].size();

	floatVector4 centerXs, centerYs, centerZs, radiuses, Bs, Cs, Ds, DsSqrted, zeroVector, t0s, t1s;

	float maxFloat = std::numeric_limits<float>::max();
	zeroVector = _mm_set1_ps(0.f);
	closestPointT =  maxFloat;
	float currentClosestT;

	for(int i = 0; i < numSpheres; i += 4)
	{
		for(int j = 0; j < 4; ++j)
		{
			if(i + j < numSpheres)
			{
				centerXs[j] = this->spheres[this->leafSpheresIndexes[spheresIndexesIndex][i + j]].center.x;
				centerYs[j] = this->spheres[this->leafSpheresIndexes[spheresIndexesIndex][i + j]].center.y;
				centerZs[j] = this->spheres[this->leafSpheresIndexes[spheresIndexesIndex][i + j]].center.z;
				radiuses[j] = this->spheres[this->leafSpheresIndexes[spheresIndexesIndex][i + j]].radius;
			}
			else
			{
				centerXs[j] = 0.f;
				centerYs[j] = 0.f;
				centerZs[j] = 0.f;
				radiuses[j] = 0.f;
			}
		}

		//A = 1 when ray.dir is normal
		Vector3D normalizedDir = ray.dir.Normalized();
		//B = 2 * (Xd * (X0 - Xc) + Yd * (Y0 - Yc) + Zd * (Z0 - Zc))
		Bs = 2.f * (normalizedDir.x * (ray.origin.x - centerXs) + normalizedDir.y * (ray.origin.y - centerYs)
			 + normalizedDir.z * (ray.origin.z - centerZs));

		//D = (X0 - Xc)^2 + (Y0 - Yc)^2 + (Z0 - Zc)^2 - Sr^2
		Cs = (ray.origin.x - centerXs) * (ray.origin.x - centerXs) +
			 (ray.origin.y - centerYs) * (ray.origin.y - centerYs) +
			 (ray.origin.z - centerZs) * (ray.origin.z - centerZs)- radiuses * radiuses;

		Ds = Bs * Bs - 4.f * Cs;

		//using http://felix.abecassis.me/2012/08/sse-vectorizing-conditional-code/:
		floatVector4 nonNegativesMask = _mm_cmpge_ps(Ds, zeroVector);

		DsSqrted = _mm_sqrt_ps(Ds);
		DsSqrted = _mm_and_ps(nonNegativesMask, DsSqrted);

		t0s = 0.5f * (-Bs - DsSqrted);
		t1s = 0.5f * (-Bs + DsSqrted);

		t0s = _mm_and_ps(nonNegativesMask, t0s);
		t1s = _mm_and_ps(nonNegativesMask, t1s);

		currentClosestT = maxFloat;

		for(int j = 0; j < 4; ++j)
		{
			if(t0s[j] > 0.f && t0s[j] < currentClosestT) currentClosestT = t0s[j];
			if(t1s[j] > 0.f && t1s[j] < currentClosestT) currentClosestT = t1s[j];
		}

		if(currentClosestT < closestPointT) closestPointT = currentClosestT;
	}

	 return (closestPointT < maxFloat);
}


void KDTree::MinAndMaxCoordinateByAxis(float& min, float& max, Axis axis)
{
	min = this->spheres[0].center[axis] - this->spheres[0].radius;
	max = this->spheres[0].center[axis] + this->spheres[0].radius;

	float currentMin, currentMax;

	for(const auto& sphere : this->spheres)
	{
		currentMin = sphere.center[axis] - sphere.radius;
		currentMax = sphere.center[axis] + sphere.radius;
		if(currentMin < min) min = currentMin;
		if(currentMax > max) max =  currentMax;
	}
}


void KDTree::BuildTree(const vector<Sphere>& spheres)
{
	this->spheres = spheres;

	this->tree.resize(0);
	this->tree.push_back(Node());

	std::thread byX(&KDTree::MinAndMaxCoordinateByAxis, this, std::ref(this->sceneBox.minPoint.x),
					std::ref(this->sceneBox.maxPoint.x), X);
	std::thread byY(&KDTree::MinAndMaxCoordinateByAxis, this, std::ref(this->sceneBox.minPoint.y),
					std::ref(this->sceneBox.maxPoint.y), Y);
	std::thread byZ(&KDTree::MinAndMaxCoordinateByAxis, this, std::ref(this->sceneBox.minPoint.z),
						std::ref(this->sceneBox.maxPoint.z), Z);

	//this->MinAndMaxCoordinateByAxis(this->sceneBox.minPoint.z,this->sceneBox.maxPoint.z, Z);

	int n = this->spheres.size();
	vector<unsigned> spheresIndexes(n);
	for(int i = 0; i < n; ++i)
	{
		spheresIndexes[i] = i;
	}

	byX.join();
	byY.join();
	byZ.join();

	this->BuildTree(0, this->sceneBox, spheresIndexes);
}


void KDTree::BuildTree(unsigned nodeIndex, BoundingBox box, const vector<unsigned>& spheresIndexes)
{
	if(spheresIndexes.size() <= maxSpheresInLeaf)
	{
		this->leafSpheresIndexes.push_back(spheresIndexes);
		this->tree[nodeIndex].leaf.flagDimentionOffset = 0;
		this->tree[nodeIndex].leaf.flagDimentionOffset |= this->leafSpheresIndexes.size() - 1;
		this->tree[nodeIndex].leaf.flagDimentionOffset |= (unsigned int)(1<<31);//is leaf, flag set to 1
		return;
	}

	SplitPlane plane = this->BestSplitPlane(box, spheresIndexes);

	BoundingBox leftBox, rightBox;

	box.Split(plane.axis, plane.coordinate, leftBox, rightBox);

	vector<unsigned> leftSpheresIndexes, rightSpheresIndexes;

	for(unsigned i : spheresIndexes)
	{
		if(this->spheres[i].center[plane.axis] <= plane.coordinate)
		{
			leftSpheresIndexes.push_back(i);
		}
		else rightSpheresIndexes.push_back(i);
	}

	Node leftChild, rightChild;
	this->tree.push_back(leftChild);
	this->tree.push_back(rightChild);
	unsigned leftChildIndex = tree.size() - 2;
	unsigned rightChildIndex= leftChildIndex + 1;
	this->tree[nodeIndex].inner.flagDimentionOffset = 0;
	this->tree[nodeIndex].inner.flagDimentionOffset |= (leftChildIndex - nodeIndex) * sizeof(Node);//the offset of the first child *8, do the first 3 bits are 0's
	this->tree[nodeIndex].inner.flagDimentionOffset |= (unsigned)plane.axis;
	this->tree[nodeIndex].inner.splitCoordinate = plane.coordinate;
	this->BuildTree(leftChildIndex, leftBox, leftSpheresIndexes);
	this->BuildTree(rightChildIndex, rightBox, rightSpheresIndexes);
}


//Cost(cell) = C_trav + Prob(hit L) * Cost(L) + Prob(hit R) * Cost(R)
//= C_trav + SA(L) * TriCount(L) + SA(R) * TriCount(R)
float KDTree::HeuristicEstimation(BoundingBox box,  const vector<unsigned>& spheresIndexes,
								 Axis axis, float splitCoordinate) const
{
	float al, bl, cl, ar, br, cr; //boxes' dimensions lengths
	if(axis == X)
	{
		al = splitCoordinate - box.minPoint.x;
		ar = box.maxPoint.x - splitCoordinate;

		bl = box.maxPoint.y - box.minPoint.y;
		br = box.maxPoint.y - box.minPoint.y;

		cl = box.maxPoint.z - box.minPoint.z;
		cr = box.maxPoint.z - box.minPoint.z;
	}

	if(axis == Y)
	{
		al = box.maxPoint.x - box.minPoint.x;
		ar = box.maxPoint.x - box.minPoint.x;

		bl = splitCoordinate - box.minPoint.y;
		br = box.maxPoint.y - splitCoordinate;

		cl = box.maxPoint.z - box.minPoint.z;
		cr = box.maxPoint.z - box.minPoint.z;
	}

	if(axis == Z)
	{

		al = box.maxPoint.x - box.minPoint.x;
		ar = box.maxPoint.x - box.minPoint.x;


		bl = box.maxPoint.y - box.minPoint.y;
		br = box.maxPoint.y - box.minPoint.y;

		cl = splitCoordinate - box.minPoint.z;
		cr = box.maxPoint.z - splitCoordinate;
	}

	float sl = al * bl + bl * cl + al * cl;// multiplying the exact formula by 1/2
	float sr = ar * br + br * cr + ar * cr;//it won't change the estimation

	unsigned rightSpheresCount = 0, leftSpheresCount = 0;

	for(int i : spheresIndexes)
	{
		if((this->spheres[i].center[axis] + this->spheres[i].radius) <= splitCoordinate) ++leftSpheresCount;
		else if((this->spheres[i].center[axis] - this->spheres[i].radius) >= splitCoordinate) ++rightSpheresCount;
	}

	return sl * leftSpheresCount + sr * rightSpheresCount;
}


void KDTree::BestSplitPlaneByAxis(const BoundingBox& box,  const vector<unsigned>& spheresIndexes, Axis axis,
								  SplitPlane& bestPlane) const
{
	SplitPlane currentPlane;
	bestPlane.coordinate = currentPlane.coordinate =  axis;

	bestPlane.splitEstimation = std::numeric_limits<float>::max();
	float coordinate;
	for(auto i : spheresIndexes)
	{
		coordinate = this->spheres[i].center[axis] - this->spheres[i].radius;
		if(coordinate >= box.minPoint[axis])
		{
			currentPlane.coordinate = coordinate;
			currentPlane.splitEstimation = this->HeuristicEstimation(box, spheresIndexes, axis, coordinate);
			if(currentPlane < bestPlane) bestPlane = currentPlane;
		}

		coordinate = this->spheres[i].center[axis] - this->spheres[i].radius;
		if(coordinate <= box.minPoint[axis])
		{
			currentPlane.coordinate = coordinate;
			currentPlane.splitEstimation = this->HeuristicEstimation(box, spheresIndexes, axis, coordinate);
			if(currentPlane < bestPlane) bestPlane = currentPlane;
		}
	}

}


SplitPlane KDTree::BestSplitPlane(const BoundingBox& box,  const vector<unsigned>& spheresIndexes) const
{
	SplitPlane bestByX, bestByY, bestByZ;

	bestByX.axis = X;
	bestByY.axis = Y;
	bestByZ.axis = Z;

	bestByX.coordinate = (box.minPoint.x + box.maxPoint.x ) * 0.5;
	bestByY.coordinate = (box.minPoint.y + box.maxPoint.y ) * 0.5;
	bestByZ.coordinate = (box.minPoint.z + box.maxPoint.z ) * 0.5;

	bestByX.splitEstimation = this->HeuristicEstimation(box, spheresIndexes, X, bestByX.coordinate);
	bestByY.splitEstimation = this->HeuristicEstimation(box, spheresIndexes, Y, bestByY.coordinate);
	bestByZ.splitEstimation = this->HeuristicEstimation(box, spheresIndexes, Z,bestByZ.coordinate);

//	std::thread xThread(&KDTree::BestSplitPlaneByAxis, this, std::cref(box),
//						std::cref(spheresIndexes), X, std::ref(bestByX));
//	std::thread yThread(&KDTree::BestSplitPlaneByAxis, this, std::cref(box),
//						std::cref(spheresIndexes), Y, std::ref(bestByY));
//	this->BestSplitPlaneByAxis(box, spheresIndexes, Z, bestByZ);
//
//	xThread.join();
//	yThread.join();

	vector<SplitPlane> bests = {bestByX, bestByY, bestByZ};
	std::sort(bests.begin(), bests.end());

	return bests[0];
}
