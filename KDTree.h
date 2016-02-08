#ifndef KDTREE_H_
#define KDTREE_H_

#include "Vector3D.h"

#include <vector>

using std::vector;

#define ISLEAF(n) (n.flagDimentionOffset & (unsigned int)(1<<31))
#define AXIS(n) (n.flagDimentionOffset & 0x3)
#define FIRST_CHILD_OFFSET(n) (n.flagDimentionOffset & (0x7FFFFFFC))

const float travesingCost = 1.f;
const int minSpheresInLeaf = 256;


enum Axis{X, Y, Z};


struct BoundingBox
{
	Vector3D minPoint, maxPoint;

	void Split(Axis axis, float splitCoordinate, BoundingBox& leftBox, BoundingBox& rightBox) const
	{
		leftBox = rightBox = *this;
		leftBox.minPoint[axis] = rightBox.maxPoint[axis] = splitCoordinate;
	}
};


struct LeafNode {
	unsigned int flagDimentionOffset;
	// bits 0..1: splitting dimension
	// bits 2..30v: offset bits
	// bit 31 (sign) : flag whether node is a leaf
};


struct InnerNode {
	unsigned int flagDimentionOffset;
	// bits 0..30: offset to first son
	// bit 31 (sign) : flat whether node is a leaf
	float splitCoordinate;
};


typedef union {
	LeafNode leaf;
	InnerNode inner;
} Node;


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
	KDTree();

	KDTree(const KDTree& other) = delete;
	KDTree& operator = (const KDTree& other) = delete;

	~KDTree();

	void BuildTree(const vector<Sphere>& spheres);

private:
	void MinAndMaxCoordinateByAxis(float& min, float& max, Axis axis);
	void BuildTree(int nodeIndex, BoundingBox box,  const vector<int>& spheresIndexes);

	float HeuristicEstimation(BoundingBox box,  const vector<int>& spheresIndexes, Axis axis,
							  float splitCoordinate) const;

	SplitPlane BestSplitPlane(const BoundingBox& box, const vector<int>& spheresIndexes) const;
	void BestSplitPlaneByAxis(const BoundingBox& box, const vector<int>& spheresIndexes, Axis axis,
							  SplitPlane& bestPlane) const;

	std::vector<Node> tree;
	std::vector<Sphere> spheres;
};

#endif /* KDTREE_H_ */
