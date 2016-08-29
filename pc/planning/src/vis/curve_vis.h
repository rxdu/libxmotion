/*
 * curve_vis.h
 *
 *  Created on: Aug 28, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_VIS_CURVE_VIS_H_
#define PLANNING_SRC_VIS_CURVE_VIS_H_

#include <vector>

#include "opencv2/opencv.hpp"

namespace srcl_ctrl {

namespace CurveVis {

void VisPolynomialCurve(std::vector<double> coeff, double t0, double t1, double step, cv::OutputArray _dst);

}

}

#endif /* PLANNING_SRC_VIS_CURVE_VIS_H_ */
