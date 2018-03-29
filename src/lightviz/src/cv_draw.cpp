/* 
 * cv_draw.cpp
 * 
 * Created on: Mar 28, 2018 14:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/cv_draw.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace librav;
using namespace cv;

/********************* Common Colors *********************/
const cv::Scalar LVColors::default_pt_color = Scalar(0, 0, 255);
const cv::Scalar LVColors::default_ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar LVColors::default_area_color = Scalar(0, 255, 255);

const cv::Scalar LVColors::bg_color = Scalar(255, 255, 255);
const cv::Scalar LVColors::ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar LVColors::obs_color = Scalar(Scalar(0, 102, 204));
const cv::Scalar LVColors::aoi_color = Scalar(Scalar(0, 255, 255));
const cv::Scalar LVColors::start_color = Scalar(0, 0, 255);
const cv::Scalar LVColors::intermediate_color = Scalar(0, 0, 255);
const cv::Scalar LVColors::finish_color = Scalar(51, 153, 51); //Scalar(153, 76, 0);

/******************* Basic Input/Output *******************/
cv::Mat LightViz::ReadImageFile(std::string img_file)
{
    return imread(img_file);
}

cv::Mat LightViz::ReadColorImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_COLOR);
}

cv::Mat LightViz::ReadGrayscaleImage(std::string img_file)
{
    return imread(img_file, CV_LOAD_IMAGE_GRAYSCALE);
}

void LightViz::ShowImage(cv::Mat img, std::string window_name, bool save_img)
{
    namedWindow(window_name, WINDOW_NORMAL); // Create a window for display.
    imshow(window_name, img);                // Show our image inside it.

    waitKey(0); // Wait for a keystroke in the window

    if (save_img)
        imwrite(window_name + ".jpg", img);

    destroyWindow(window_name);
}

void LightViz::ShowImage(std::string file_name, std::string window_name)
{
    cv::Mat img = imread(file_name);
    ShowImage(img, window_name, false);
}

/******************** Draw Primitives *********************/
void LightViz::DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color)
{
    int thickness = -1;
    int lineType = 8;
    Point center(pos.x, pos.y);
    circle(img,
           center,
           3,
           color,
           thickness,
           lineType);
}

void LightViz::DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color)
{
    int thickness = 1;
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
void LightViz::DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar &color)
{
    int line_type = 8;
    int thickness = 1;

    double tip_size = length * 0.1;
    if (tip_size < 1)
        tip_size = 1;

    double angleRad = angle * CV_PI / 180.0;                                                                // convert angle to radians
                                                                                                            // cv::Point tip_pt = cv::Point(base_pos.x + length * sin(angleRad), base_pos.y - length * cos(angleRad)); // calculate tip position
    cv::Point tip_pt = cv::Point(base_pos.x + length * sin(angleRad), base_pos.y - length * cos(angleRad)); // calculate tip position

    line(img, base_pos, tip_pt, color, thickness, line_type);

    double angle_l = atan2((double)base_pos.y - tip_pt.y, (double)base_pos.x - tip_pt.x);

    Point p(cvRound(tip_pt.x + tip_size * cos(angle_l + CV_PI / 4)),
            cvRound(tip_pt.y + tip_size * sin(angle_l + CV_PI / 4)));
    line(img, p, tip_pt, color, thickness, line_type);

    p.x = cvRound(tip_pt.x + tip_size * cos(angle_l - CV_PI / 4));
    p.y = cvRound(tip_pt.y + tip_size * sin(angle_l - CV_PI / 4));
    line(img, p, tip_pt, color, thickness, line_type);
}