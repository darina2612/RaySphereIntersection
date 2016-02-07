#ifndef KDTREE_H_
#define KDTREE_H_

#include <vector>


struct LeafNode {
unsigned int flagDimentionOffset;
// bits 0..1: splitting dimension
// bits 2..30v: offset bits
// bit 31 (sign) : flag whether node is a leaf
};
struct InnerNode {
unsigned int flagAndOffset;
// bits 0..30: offset to first son
// bit 31 (sign) : flat whether node is a leaf
float splitCoordinate;
};

typedef union {
LeafNode leaf;
InnerNode inner;
} Node;

class KDTree {
public:
	KDTree();

	KDTree(const KDTree& other) = delete;
	KDTree& operator = (const KDTree& other) = delete;

	~KDTree();

	std::vector<Node> tree;

};

#endif /* KDTREE_H_ */
