/* 
 * draw_primitives.cpp
 * 
 * Created on: Jan 04, 2019 10:29
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "cvdraw/details/draw_primitives.hpp"

using namespace cv;

namespace librav
{
void CvDraw::DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color, int thick)
{
    int thickness = thick;
    int lineType = 8;
    Point center(pos.x, pos.y);
    circle(img,
           center,
           3,
           color,
           thickness,
           lineType);
}

void CvDraw::DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color, int thick)
{
    int thickness = thick;
    int lineType = 8;

    line(img,
         pt1,
         pt2,
         color,
         thickness,
         lineType);
}

// Source:
//	* https://adishavit.github.io/2015/drawing-arrows-with-opencv/
//	* http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html#arrowedline
void CvDraw::DrawArrow(cv::Mat img, cv::Point base_pt, double length, double angle, const cv::Scalar &color, int thick)
{
    int line_type = 8;
    int thickness = thick;

    double tip_size = length * 0.1;
    if (tip_size < 1)
        tip_size = 1;

    double angleRad = angle * CV_PI / 180.0;                                                              // convert angle to radians
                                                                                                          // cv::Point tip_pt = cv::Point(base_pt.x + length * sin(angleRad), base_pt.y - length * cos(angleRad)); // calculate tip position
    cv::Point tip_pt = cv::Point(base_pt.x + length * sin(angleRad), base_pt.y - length * cos(angleRad)); // calculate tip position

    line(img, base_pt, tip_pt, color, thickness, line_type);

    double angle_l = atan2((double)base_pt.y - tip_pt.y, (double)base_pt.x - tip_pt.x);

    Point p(cvRound(tip_pt.x + tip_size * cos(angle_l + CV_PI / 4)),
            cvRound(tip_pt.y + tip_size * sin(angle_l + CV_PI / 4)));
    line(img, p, tip_pt, color, thickness, line_type);

    p.x = cvRound(tip_pt.x + tip_size * cos(angle_l - CV_PI / 4));
    p.y = cvRound(tip_pt.y + tip_size * sin(angle_l - CV_PI / 4));
    line(img, p, tip_pt, color, thickness, line_type);
    // arrowedLine(img, base_pt, tip_pt, color, thickness, line_type);
}

void CvDraw::DrawArrow(cv::Mat img, cv::Point base_pos, cv::Point tip_pos, const cv::Scalar &color, int thick)
{
    int line_type = 8;
    int thickness = thick;
    cv::arrowedLine(img, base_pos, tip_pos, color, thickness, line_type);
}

void CvDraw::WriteText(cv::Mat img, std::string text, cv::Point pos, const cv::Scalar &color, double scale, int thick)
{
    putText(img, text, pos, FONT_HERSHEY_SIMPLEX, scale, color, thick);
}
} // namespace librav
