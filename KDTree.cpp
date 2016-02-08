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


void KDTree::BuildTree(Node* node, BoundingBox box, const vector<Sphere>& spheres)
{
	if(spheres.size() <= minSpheresInLeaf)
	{
		node->leaf.flagDimentionOffset |= (unsigned int)(1<<31);//is leaf, flag set to 1
		//!!!!!!!!!!TODO:set where objects are!!!!
		return;
	}

	SplitPlane plane = this->BestSplitPlane(box, spheres);

	BoundingBox leftBox, rightBox;

	box.Split(plane.axis, plane.coordinate, leftBox, rightBox);

	vector<Sphere> leftSpheres, rightSpheres;

	for(auto sphere : spheres)
	{
		if(sphere.center[plane.axis] <= plane.coordinate)
		{
			leftSpheres.push_back(sphere);
		}
		else rightSpheres.push_back(sphere);
	}

	Node leftChild, rightChild;
	this->tree.push_back(leftChild);
	this->tree.push_back(rightChild);
	node->inner.flagDimentionOffset = (this->tree.size()-2) * 8;//the offset of the first child *8, do the first 3 bits are 0's
	node->inner.flagDimentionOffset |= (unsigned)(plane.axis << 30);
	this->BuildTree(&leftChild, leftBox, leftSpheres);
	this->BuildTree(&rightChild, rightBox, rightSpheres);
}


//Cost(cell) = C_trav + Prob(hit L) * Cost(L) + Prob(hit R) * Cost(R)
//= C_trav + SA(L) * TriCount(L) + SA(R) * TriCount(R)
float KDTree::HeuristicEstimation(BoundingBox box, const vector<Sphere>& spheres,
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

	for(auto sphere : spheres)
	{
		if((sphere.center[axis] + sphere.radius) <= splitCoordinate) ++leftSpheresCount;
		else if((sphere.center[axis] - sphere.radius) >= splitCoordinate) ++rightSpheresCount;
	}

	return travesingCost + sl * leftSpheresCount + sr * rightSpheresCount;
}


void KDTree::BestSplitPlaneByAxis(const BoundingBox& box, const vector<Sphere> & spheres, Axis axis,
								  SplitPlane& bestPlane) const
{
	SplitPlane currentPlane;
	bestPlane.coordinate = currentPlane.coordinate =  axis;

	bestPlane.splitEstimation = std::numeric_limits<float>::max();
	float coordinate;
	for(auto sphere : spheres)
	{
		coordinate = sphere.center[axis] - sphere.radius;
		if(coordinate >= box.minPoint[axis])
		{
			currentPlane.coordinate = coordinate;
			currentPlane.splitEstimation = this->HeuristicEstimation(box, spheres, axis, coordinate);
			if(currentPlane < bestPlane) bestPlane = currentPlane;
		}

		coordinate = sphere.center[axis] + sphere.radius;
		if(coordinate <= box.minPoint[axis])
		{
			currentPlane.coordinate = coordinate;
			currentPlane.splitEstimation = this->HeuristicEstimation(box, spheres, axis, coordinate);
			if(currentPlane < bestPlane) bestPlane = currentPlane;
		}
	}

}


SplitPlane KDTree::BestSplitPlane(const BoundingBox& box, const vector<Sphere> & spheres) const
{
	SplitPlane bestByX, bestByY, bestByZ;

	std::thread xThread(&KDTree::BestSplitPlaneByAxis, this, std::cref(box),
						std::cref(spheres), X, std::ref(bestByX));
	std::thread yThread(&KDTree::BestSplitPlaneByAxis, this, std::cref(box),
						std::cref(spheres), Y, std::ref(bestByY));
	this->BestSplitPlaneByAxis(box, spheres, Z, bestByZ);

	xThread.join();
	yThread.join();

	vector<SplitPlane> bests = {bestByX, bestByY, bestByZ};
	std::sort(bests.begin(), bests.end());

	return bests[0];
}
