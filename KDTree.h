#ifndef KDTREE_H_
#define KDTREE_H_

#include "Vector3D.h"

#include <vector>

using std::vector;

//const float travesingCost = 1.f;
const int maxSpheresInLeaf = 32;


enum Axis{X, Y, Z};


struct BoundingBox
{
	Vector3D minPoint, maxPoint;

	void Split(Axis axis, float splitCoordinate, BoundingBox& leftBox, BoundingBox& rightBox) const
	{
		leftBox = rightBox = *this;
		leftBox.maxPoint[axis] = rightBox.minPoint[axis] = splitCoordinate;
	}
};


struct LeafNode {
	//flag -1b(isLeaf) | offset - 31b(index of spheres indexes in leafSpheresIndexes)
	unsigned int flagDimentionOffset;
};


struct InnerNode {
	unsigned int flagDimentionOffset;
	// bits 0..30: offset to first son
	// bit 31 (sign) : flat whether node is a leaf
	float splitCoordinate;
};


union Node
{
	LeafNode leaf;
	InnerNode inner;
};


inline bool isLeaf(const Node& n)
{
	return n.inner.flagDimentionOffset & (unsigned int)(1<<31);
}


inline Axis splitAxis(const Node& n)
{
	return static_cast<Axis>(n.inner.flagDimentionOffset & 0x3);
}


inline int firstChildOffset(const Node& n)
{
	return n.inner.flagDimentionOffset & (0x7FFFFFFC);
}


struct SplitPlane
{
	Axis axis;
	float coordinate;
	float splitEstimation;

	bool operator < (const SplitPlane& other) const
	{
		return splitEstimation < other.splitEstimation;
	}
};


class KDTree {
public:
	KDTree(){leaves = 0;};

	KDTree(const KDTree& other) = delete;
	KDTree& operator = (const KDTree& other) = delete;

	~KDTree();

	void BuildTree(const vector<Sphere>& spheres);

	bool Intersect(const Ray& ray, Vector3D& intersectionPoint) const;

	const vector<Sphere>& GetSpheres() const {return this->spheres;}

	int NodesNumber() const {return tree.size();}
	int LeavesNumber()const {return leaves;}

private:
	bool Traverse(const Ray& ray, float tnear, float tfar, float& intersectionPointT) const;

	void MinAndMaxCoordinateByAxis(float& min, float& max, Axis axis);
	void BuildTree(int nodeIndex, BoundingBox box,  const vector<int>& spheresIndexes);

	float HeuristicEstimation(BoundingBox box,  const vector<int>& spheresIndexes, Axis axis,
							  float splitCoordinate) const;

	SplitPlane BestSplitPlane(const BoundingBox& box, const vector<int>& spheresIndexes) const;
	void BestSplitPlaneByAxis(const BoundingBox& box, const vector<int>& spheresIndexes, Axis axis,
							  SplitPlane& bestPlane) const;

	bool ClosestIntersectionPointInLeaf(int leafIndex, const Ray& ray, float& closestPointT) const;

	std::vector<Node> tree;
	std::vector<Sphere> spheres;
	std::vector< std::vector<int> > leafSpheresIndexes;
	BoundingBox sceneBox;
	int leaves;
};

#endif /* KDTREE_H_ */
