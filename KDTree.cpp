/*
 * KDTree.cpp
 *
 *  Created on: Feb 7, 2016
 *      Author: darina
 */

#include "KDTree.h"
#include <limits>
#include <thread>
#include <algorithm>


KDTree::KDTree()
{
	// TODO Auto-generated constructor stub

}

KDTree::~KDTree()
{
	// TODO Auto-generated destructor stub
}


void KDTree::MinAndMaxCoordinateByAxis(float& min, float& max, Axis axis)
{
	min = this->spheres[0].center[axis] - this->spheres[0].radius;
	max = this->spheres[0].center[axis] + this->spheres[0].radius;

	float currentMin, currentMax;

	for(auto sphere : this->spheres)
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

	BoundingBox theWholeBox;
	this->tree.push_back(Node());

	std::thread byX(&KDTree::MinAndMaxCoordinateByAxis, this, std::ref(theWholeBox.minPoint.x),
					std::ref(theWholeBox.maxPoint.x), X);
	std::thread byY(&KDTree::MinAndMaxCoordinateByAxis, this, std::ref(theWholeBox.minPoint.y),
					std::ref(theWholeBox.maxPoint.y), Y);

	this->MinAndMaxCoordinateByAxis(theWholeBox.minPoint.z, theWholeBox.maxPoint.z, Z);

	int n = this->spheres.size();
	vector<int> spheresIndexes(n);
	for(int i = 0; i < n; ++i)
	{
		spheresIndexes[i] = i;
	}

	byX.join();
	byY.join();

	this->BuildTree(0, theWholeBox, spheresIndexes);
}


void KDTree::BuildTree(int nodeIndex, BoundingBox box, const vector<int>& spheresIndexes)
{
	if(spheres.size() <= minSpheresInLeaf)
	{
		this->tree[nodeIndex].leaf.flagDimentionOffset |= (unsigned int)(1<<31);//is leaf, flag set to 1
		//!!!!!!!!!!TODO:set where objects are!!!!
		return;
	}

	SplitPlane plane = this->BestSplitPlane(box, spheresIndexes);

	BoundingBox leftBox, rightBox;

	box.Split(plane.axis, plane.coordinate, leftBox, rightBox);

	vector<int> leftSpheresIndexes, rightSpheresIndexes;

	for(auto i : spheresIndexes)
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
	this->tree[nodeIndex].inner.flagDimentionOffset = (this->tree.size()-2 - nodeIndex) * 8;//the offset of the first child *8, do the first 3 bits are 0's
	this->tree[nodeIndex].inner.flagDimentionOffset |= (unsigned)(plane.axis << 30);
	this->BuildTree(this->tree.size() - 2, leftBox, leftSpheresIndexes);
	this->BuildTree(this->tree.size() - 1, rightBox, rightSpheresIndexes);
}


//Cost(cell) = C_trav + Prob(hit L) * Cost(L) + Prob(hit R) * Cost(R)
//= C_trav + SA(L) * TriCount(L) + SA(R) * TriCount(R)
float KDTree::HeuristicEstimation(BoundingBox box,  const vector<int>& spheresIndexes,
								 Axis axis, float splitCoordinate) const
{
	float al, bl, cl, ar, br, cr; //boxes' dimensions lengths
	if(axis == X)
	{
		al = splitCoordinate - box.minPoint.x;
		ar = box.maxPoint.x - splitCoordinate;

		bl = br = box.maxPoint.y - box.minPoint.y;

		cl = cr = box.maxPoint.z - box.minPoint.z;
	}

	if(axis == Y)
	{
		al = ar = box.maxPoint.x - box.minPoint.x;

		bl = splitCoordinate - box.minPoint.y;
		br = box.maxPoint.y - splitCoordinate;

		cl = cr = box.maxPoint.z - box.minPoint.z;
	}

	if(axis == X)
	{

		al = ar = box.maxPoint.x - box.minPoint.x;

		bl = br = box.maxPoint.y - box.minPoint.y;

		cl = splitCoordinate - box.minPoint.z;
		cr = box.maxPoint.z - splitCoordinate;
	}

	float sl = al * bl + bl * cl + al * cl;// multiplying the exact formula by 1/2
	float sr = ar * br + br * cr + ar * cr;//it won't change the estimation

	int rightSpheresCount = 0, leftSpheresCount = 0;

	for(auto i : spheresIndexes)
	{
		if((this->spheres[i].center[axis] + this->spheres[i].radius) <= splitCoordinate) ++leftSpheresCount;
		else if((this->spheres[i].center[axis] - this->spheres[i].radius) >= splitCoordinate) ++rightSpheresCount;
	}

	return travesingCost + sl * leftSpheresCount + sr * rightSpheresCount;
}


void KDTree::BestSplitPlaneByAxis(const BoundingBox& box,  const vector<int>& spheresIndexes, Axis axis,
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


SplitPlane KDTree::BestSplitPlane(const BoundingBox& box,  const vector<int>& spheresIndexes) const
{
	SplitPlane bestByX, bestByY, bestByZ;

	std::thread xThread(&KDTree::BestSplitPlaneByAxis, this, std::cref(box),
						std::cref(spheresIndexes), X, std::ref(bestByX));
	std::thread yThread(&KDTree::BestSplitPlaneByAxis, this, std::cref(box),
						std::cref(spheresIndexes), Y, std::ref(bestByY));
	this->BestSplitPlaneByAxis(box, spheresIndexes, Z, bestByZ);

	xThread.join();
	yThread.join();

	vector<SplitPlane> bests = {bestByX, bestByY, bestByZ};
	std::sort(bests.begin(), bests.end());

	return bests[0];
}
