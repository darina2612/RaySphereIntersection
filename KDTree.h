//!!!Mainly using as guides here the following:
//http://www.sci.utah.edu/~wald/PhD/wald_phd.pdf
//http://www.cescg.org/CESCG-2005/papers/TUBudapest-Toth-Balazs.pdf
//http://dcgi.felk.cvut.cz/home/havran/ARTICLES/ingo06rtKdtree.pdf

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
	unsigned flagDimentionOffset;
};


struct InnerNode {
	//flag -1b(isLeaf) | offset - 28b(offset in bytes to the first child)|splitDimention - 2b(X, Y or Z)
	unsigned flagDimentionOffset;
	float splitCoordinate;
};


union Node
{
	LeafNode leaf;
	InnerNode inner;
};


inline bool isLeaf(const Node& n)
{
	return (n.inner.flagDimentionOffset) & (unsigned)(1<<31);
}


inline Axis splitAxis(const Node& n)
{
	return static_cast<Axis>(n.inner.flagDimentionOffset & 0x3);
}


inline unsigned firstChildOffset(const Node& n)
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
	KDTree(){};

	KDTree(const KDTree& other) = delete;
	KDTree& operator = (const KDTree& other) = delete;

	~KDTree(){};

	void BuildTree(const vector<Sphere>& spheres);

	bool Intersect(const Ray& ray, Vector3D& intersectionPoint) const;

	const vector<Sphere>& GetSpheres() const {return this->spheres;}

private:
	bool Traverse(const Ray& ray, float tnear, float tfar, float& intersectionPointT) const;

	void MinAndMaxCoordinateByAxis(float& min, float& max, Axis axis);
	void BuildTree(unsigned nodeIndex, BoundingBox box,  const vector<unsigned>& spheresIndexes);

	float HeuristicEstimation(BoundingBox box,  const vector<unsigned>& spheresIndexes, Axis axis,
							  float splitCoordinate) const;

	SplitPlane BestSplitPlane(const BoundingBox& box, const vector<unsigned>& spheresIndexes) const;
	void BestSplitPlaneByAxis(const BoundingBox& box, const vector<unsigned>& spheresIndexes, Axis axis,
							  SplitPlane& bestPlane) const;

	bool ClosestIntersectionPointInLeaf(unsigned leafIndex, const Ray& ray, float& closestPointT) const;

	std::vector<Node> tree;
	std::vector<Sphere> spheres;
	std::vector< std::vector<unsigned> > leafSpheresIndexes;
	BoundingBox sceneBox;
};

#endif /* KDTREE_H_ */
