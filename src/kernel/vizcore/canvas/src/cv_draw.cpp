/* 
 * cv_draw.cpp
 * 
 * Created on: Mar 28, 2018 14:47
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "canvas/cv_draw.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace librav;
using namespace cv;

/********************* Common Colors *********************/
const cv::Scalar CvDrawColors::default_pt_color = Scalar(0, 0, 255);
const cv::Scalar CvDrawColors::default_ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar CvDrawColors::default_area_color = Scalar(0, 255, 255);

const cv::Scalar CvDrawColors::bg_color = Scalar(255, 255, 255);
const cv::Scalar CvDrawColors::ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar CvDrawColors::obs_color = Scalar(Scalar(0, 102, 204));
const cv::Scalar CvDrawColors::aoi_color = Scalar(Scalar(0, 255, 255));
const cv::Scalar CvDrawColors::start_color = Scalar(0, 0, 255);
const cv::Scalar CvDrawColors::intermediate_color = cv::Scalar(255, 153, 51); //Scalar(0, 0, 255);
const cv::Scalar CvDrawColors::finish_color = Scalar(51, 153, 51);            //Scalar(153, 76, 0);
const cv::Scalar CvDrawColors::jet_colormap_lowest = Scalar(128, 0, 0);

// Reference: https://www.rapidtables.com/web/color/RGB_Color.html
const cv::Scalar CvDrawColors::black_color = Scalar(0, 0, 0);
const cv::Scalar CvDrawColors::white_color = Scalar(255, 255, 255);
const cv::Scalar CvDrawColors::red_color = Scalar(0, 0, 255);
const cv::Scalar CvDrawColors::lime_color = Scalar(0, 255, 0);
const cv::Scalar CvDrawColors::blue_color = Scalar(255, 0, 0);
const cv::Scalar CvDrawColors::yellow_color = Scalar(0, 255, 255);
const cv::Scalar CvDrawColors::cyan_color = Scalar(255, 255, 0);
const cv::Scalar CvDrawColors::magenta_color = Scalar(255, 0, 255);
const cv::Scalar CvDrawColors::silver_color = Scalar(192, 192, 192);
const cv::Scalar CvDrawColors::gray_color = Scalar(128, 128, 128);
const cv::Scalar CvDrawColors::maroon_color = Scalar(0, 0, 128);
const cv::Scalar CvDrawColors::olive_color = Scalar(128, 128, 0);
const cv::Scalar CvDrawColors::green_color = Scalar(0, 128, 0);
const cv::Scalar CvDrawColors::purple_color = Scalar(128, 0, 128);
const cv::Scalar CvDrawColors::teal_color = Scalar(128, 128, 0);
const cv::Scalar CvDrawColors::navy_color = Scalar(128, 0, 0);

const cv::Scalar CvDrawColors::orange_color = Scalar(0, 165, 255);
const cv::Scalar CvDrawColors::honeydew_color = Scalar(240, 255, 240);
const cv::Scalar CvDrawColors::palegreen_color = Scalar(152, 251, 152);

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

void CvDraw::ShowImage(std::string file_name, std::string window_name)
{
    cv::Mat img = imread(file_name);
    ShowImage(img, window_name, false);
}

/******************** Transformations *********************/
namespace
{
// Source:
// https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
double JetInterpolate(double val, double y0, double x0, double y1, double x1)
{
    return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

double JetBase(double val)
{
    if (val <= -0.75)
        return 0;
    else if (val <= -0.25)
        return JetInterpolate(val, 0.0, -0.75, 1.0, -0.25);
    else if (val <= 0.25)
        return 1.0;
    else if (val <= 0.75)
        return JetInterpolate(val, 1.0, 0.25, 0.0, 0.75);
    else
        return 0.0;
}

double JetRed(double gray)
{
    return JetBase(gray - 0.5) * 255;
}
double JetGreen(double gray)
{
    return JetBase(gray) * 255;
}
double JetBlue(double gray)
{
    return JetBase(gray + 0.5) * 255;
}
} // namespace

// Input range: 0-1
// Output range: 0-255 (OpenCV color)
cv::Scalar CvDraw::JetPaletteTransform(double val)
{
    return cv::Scalar(JetBlue(val), JetGreen(val), JetRed(val));
}

void CvDraw::JetPaletteTransform(double val, double &r, double &g, double &b)
{
    r = JetRed(val);
    g = JetGreen(val);
    b = JetBlue(val);
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