/*
 * curve_vis.cpp
 *
 *  Created on: Aug 28, 2016
 *      Author: rdu
 */

#include <cstdint>
#include <cmath>

#include "vis/curve_vis.h"
#include "vis/vis_utils.h"

using namespace srcl_ctrl;
using namespace cv;

void CurveVis::VisPolynomialCurve(std::vector<double> coeff, double t0, double t1, double step, cv::OutputArray _dst)
{
	long img_row = 1024;
	long img_col = 1024;
	_dst.create(Size(img_row, img_col), CV_8UC3);

	Mat dst = _dst.getMat();
	dst = Scalar(255,255,255);

	double actual_size = 1;

	double scale_x = 1024.0;
	double scale_y = 1024.0/(actual_size*2);

	for(double t = t0; t <= t1 - step; t += step)
	{
		cv::Point pt1, pt2;

		double x1, y1, x2, y2;

		x1 = t/(t1 - t0);
		x2 = (t + step)/(t1 - t0);
		y1 = 0;
		y2 = 0;

		uint32_t coeff_size = coeff.size();
		for(int i = 0; i < coeff_size; i++)
		{
			y1 += coeff[i] * std::pow(x1, coeff_size - i - 1);
			y2 += coeff[i] * std::pow(x2, coeff_size - i - 1);
		}

		std::cout << "t1: " << t << " , y1: " << y1 << "; t2: " << t + step << " , y2: " << y2 << std::endl;

		x1 = (x1 - t0) * scale_x;
		y1 = (y1 + actual_size) * scale_y;

		x2 = (x2 - t0) * scale_x;
		y2 = (y2 + actual_size) * scale_y;

		std::cout << "x1: " << x1 << " , y1: " << y1 << "; x2: " << x2 << " , y2: " << y2 << std::endl;

		pt1.x = x1;
		pt1.y = y1;
		pt2.x = x2;
		pt2.y = y2;

		VisUtils::DrawLine(dst, pt1, pt2);
	}
}

