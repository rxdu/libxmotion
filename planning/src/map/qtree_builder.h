#ifndef QTREE_BUILDER_
#define QTREE_BUILDER_

#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "quad_tree.h"

namespace srcl_ctrl{

class QTreeBuilder{
public:
	QTreeBuilder(){};
	~QTreeBuilder(){};

private:
	static std::vector<QuadTreeNode*> GetAllLeafNodes(QuadTree *tree);

public:
	static QuadTree* BuildQuadTree(cv::InputArray _src, unsigned int max_depth);
};

}

#endif
