#ifndef QTREE_BUILDER_
#define QTREE_BUILDER_

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"

namespace srcl_ctrl{

class QTreeBuilder{
public:
	QTreeBuilder();
	~QTreeBuilder();

private:
	cv::Mat src_img_;

public:
	void PadGrayscaleImage(cv::InputArray src, cv::OutputArray dst);
	void BuildQuadTree();
};

}

#endif
