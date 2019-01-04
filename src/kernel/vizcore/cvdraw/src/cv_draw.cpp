/* 
 * cv_draw.cpp
 * 
 * Created on: Mar 28, 2018 14:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "cvdraw/cv_draw.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace librav;
using namespace cv;

/******************* Basic Input/Output *******************/
cv::Mat CvDraw::ReadImageFile(std::string img_file)
{
    return imread(img_file);
}

cv::Mat CvDraw::ReadColorImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_COLOR);
}

cv::Mat CvDraw::ReadGrayscaleImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_GRAYSCALE);
}

void CvDraw::ShowImage(cv::Mat img, std::string window_name, bool save_img)
{
    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, img);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    if (save_img)
        imwrite(window_name + ".png", img);

    destroyWindow(window_name);
}

void CvDraw::ShowImageFrame(cv::Mat img, std::string window_name, int32_t frame_period_ms)
{
    imshow(window_name, img); // Show our image inside it.
    waitKey(frame_period_ms); // Wait for a keystroke in the window
}

void CvDraw::ShowImage(std::string file_name, std::string window_name)
{
    cv::Mat img = imread(file_name);
    ShowImage(img, window_name, false);
}

/******************** Draw Primitives *********************/
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