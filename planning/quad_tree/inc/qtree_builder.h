#ifndef QTREE_BUILDER_
#define QTREE_BUILDER_

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

#include "quad_tree.h"

namespace srcl_ctrl{

class QTreeBuilder{
public:
	QTreeBuilder();
	~QTreeBuilder();

private:
	cv::Mat src_img_;
	QuadTree* tree_;

private:
	void PadGrayscaleImage(cv::InputArray src);
	bool CheckAreaOccupancy(unsigned int x_start, unsigned int x_end, unsigned int y_start, unsigned int y_end);

public:
	void PadGrayscaleImage(cv::InputArray src, cv::OutputArray dst);
	void BuildQuadTree(cv::InputArray _src, unsigned int max_depth);
};

}

#endif
