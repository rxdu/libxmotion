#ifndef QTREE_BUILDER_
#define QTREE_BUILDER_

#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "quad_tree.h"

namespace srcl_ctrl{

enum class TreeVisType
{
	FREE_SPACE,
	OCCU_SPACE,
	ALL_SPACE
};

class QTreeBuilder{
public:
	QTreeBuilder();
	~QTreeBuilder();

private:
	cv::Mat src_img_;
	cv::Mat vis_img_;
	QuadTree* tree_;
	QuadTree* exttree_;

private:
	bool PreprocessImage(cv::InputArray _src);
	bool PadGrayscaleImage(cv::InputArray _src);
	OccupancyType CheckAreaOccupancy(BoundingBox area);
	std::vector<TreeNode*> GetAllLeafNodes();

public:
	void BuildQuadTree(cv::InputArray _src, unsigned int max_depth);
	QuadTree* BuildExtQuadTree(cv::InputArray _src, unsigned int max_depth);
	void VisualizeQuadTree(cv::OutputArray _dst, TreeVisType vis_type);
	void VisualizeExtQuadTree(cv::OutputArray _dst, TreeVisType vis_type);
};

}

#endif
