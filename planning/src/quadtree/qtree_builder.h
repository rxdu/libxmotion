#ifndef QTREE_BUILDER_
#define QTREE_BUILDER_

#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "quad_tree.h"

namespace srcl_ctrl{

class QTreeBuilder{
public:
	QTreeBuilder();
	~QTreeBuilder();

public:
	cv::Mat padded_img_;

private:
	QuadTree* tree_;

private:
	bool PreprocessImage(cv::InputArray _src);
	bool PadGrayscaleImage(cv::InputArray _src);
	OccupancyType CheckAreaOccupancy(BoundingBox area);
	std::vector<TreeNode*> GetAllLeafNodes();

public:
	QuadTree* BuildQuadTree(cv::InputArray _src, unsigned int max_depth);
};

}

#endif
