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
const cv::Scalar LVColors::intermediate_color = cv::Scalar(255, 153, 51); //Scalar(0, 0, 255);
const cv::Scalar LVColors::finish_color = Scalar(51, 153, 51);            //Scalar(153, 76, 0);
const cv::Scalar LVColors::jet_colormap_lowest = Scalar(128, 0, 0);

// Reference: https://www.rapidtables.com/web/color/RGB_Color.html
const cv::Scalar LVColors::black_color = Scalar(0, 0, 0);
const cv::Scalar LVColors::white_color = Scalar(255, 255, 255);
const cv::Scalar LVColors::red_color = Scalar(0, 0, 255);
const cv::Scalar LVColors::lime_color = Scalar(0, 255, 0);
const cv::Scalar LVColors::blue_color = Scalar(255, 0, 0);
const cv::Scalar LVColors::yellow_color = Scalar(0, 255, 255);
const cv::Scalar LVColors::cyan_color = Scalar(255, 255, 0);
const cv::Scalar LVColors::magenta_color = Scalar(255, 0, 255);
const cv::Scalar LVColors::silver_color = Scalar(192, 192, 192);
const cv::Scalar LVColors::gray_color = Scalar(128, 128, 128);
const cv::Scalar LVColors::maroon_color = Scalar(0, 0, 128);
const cv::Scalar LVColors::olive_color = Scalar(128, 128, 0);
const cv::Scalar LVColors::green_color = Scalar(0, 128, 0);
const cv::Scalar LVColors::purple_color = Scalar(128, 0, 128);
const cv::Scalar LVColors::teal_color = Scalar(128, 128, 0);
const cv::Scalar LVColors::navy_color = Scalar(128, 0, 0);

const cv::Scalar LVColors::orange_color = Scalar(0, 165, 255);

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
        imwrite(window_name + ".png", img);

    destroyWindow(window_name);
}

void LightViz::ShowImage(std::string file_name, std::string window_name)
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
cv::Scalar LightViz::JetPaletteTransform(double val)
{
    return cv::Scalar(JetBlue(val), JetGreen(val), JetRed(val));
}

void LightViz::JetPaletteTransform(double val, double &r, double &g, double &b)
{
    r = JetRed(val);
    g = JetGreen(val);
    b = JetBlue(val);
}

/******************** Draw Primitives *********************/
void LightViz::DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color, int thick)
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

void LightViz::DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color, int thick)
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
void LightViz::DrawArrow(cv::Mat img, cv::Point base_pt, double length, double angle, const cv::Scalar &color, int thick)
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

void LightViz::DrawArrow(cv::Mat img, cv::Point base_pos, cv::Point tip_pos, const cv::Scalar &color, int thick)
{
    int line_type = 8;
    int thickness = thick;
    cv::arrowedLine(img, base_pos, tip_pos, color, thickness, line_type);
}

void LightViz::WriteText(cv::Mat img, std::string text, cv::Point pos, const cv::Scalar &color, double scale, int thick)
{
    putText(img, text, pos, FONT_HERSHEY_SIMPLEX, scale, color, thick);
}