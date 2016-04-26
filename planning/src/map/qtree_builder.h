#ifndef QTREE_BUILDER_
#define QTREE_BUILDER_

#include <vector>
#include <tuple>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "quadtree/quad_tree.h"

namespace srcl_ctrl{

class QTreeBuilder{
public:
	QTreeBuilder(){};
	~QTreeBuilder(){};

private:
	static std::vector<QuadTreeNode*> GetAllLeafNodes(QuadTree *tree);

public:
	static QuadTree* BuildQuadTree(cv::InputArray _src, unsigned int max_depth);
	static std::tuple<QuadTree*, cv::Mat> BuildQuadTreeMap(cv::InputArray _src, unsigned int max_depth);
};

}

#endif
