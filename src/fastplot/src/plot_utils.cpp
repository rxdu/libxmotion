/* 
 * plot_utils.cpp
 * 
 * Created on: Dec 29, 2017 18:13
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "fastplot/plot_utils.hpp"

using namespace librav;
using namespace cv;

// frequently used colors
const cv::Scalar FastplotColors::default_pt_color = Scalar(0, 0, 255);
const cv::Scalar FastplotColors::default_ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar FastplotColors::default_area_color = Scalar(0, 255, 255);

const cv::Scalar FastplotColors::bg_color = Scalar(255, 255, 255);
const cv::Scalar FastplotColors::ln_color = Scalar(Scalar(0, 0, 0));
const cv::Scalar FastplotColors::obs_color = Scalar(Scalar(0, 102, 204));
const cv::Scalar FastplotColors::aoi_color = Scalar(Scalar(0, 255, 255));
const cv::Scalar FastplotColors::start_color = Scalar(0, 0, 255);
const cv::Scalar FastplotColors::intermediate_color = Scalar(0, 0, 255);
const cv::Scalar FastplotColors::finish_color = Scalar(51, 153, 51);//Scalar(153, 76, 0);

void PlotUtils::DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar &color)
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

void PlotUtils::DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar &color)
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
void PlotUtils::DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar &color)
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

void PlotUtils::DrawSquare(cv::Mat img, BoundingBox<int32_t> bbox, const cv::Scalar &color)
{
    Range rngx(bbox.x.min, bbox.x.max);
    Range rngy(bbox.y.min, bbox.y.max);
    img(rngy, rngx) = color;
}

// Source: http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
HSVColor PlotUtils::RGB2HSV(RGBColor in)
{
    HSVColor out;

    in.r = in.r / 255.0;
    in.g = in.g / 255.0;
    in.b = in.b / 255.0;

    double min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min < in.b ? min : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max > in.b ? max : in.b;

    out.v = max; // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if (max > 0.0)
    {                          // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max); // s
    }
    else
    {
        // if max is 0, then r = g = b = 0
        // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN; // its now undefined
        return out;
    }
    if (in.r >= max)                   // > is bogus, just keeps compilor happy
        out.h = (in.g - in.b) / delta; // between yellow & magenta
    else if (in.g >= max)
        out.h = 2.0 + (in.b - in.r) / delta; // between cyan & yellow
    else
        out.h = 4.0 + (in.r - in.g) / delta; // between magenta & cyan

    out.h *= 60.0; // degrees

    if (out.h < 0.0)
        out.h += 360.0;

    return out;
}

RGBColor PlotUtils::HSV2RGB(HSVColor in)
{
    double hh, p, q, t, ff;
    long i;
    RGBColor out;

    if (in.s <= 0.0)
    { // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;

        out.r = out.r * 255;
        out.g = out.g * 255;
        out.b = out.b * 255;

        return out;
    }

    hh = in.h;
    if (hh >= 360.0)
        hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch (i)
    {
    case 0:
        out.r = in.v;
        out.g = t;
        out.b = p;
        break;
    case 1:
        out.r = q;
        out.g = in.v;
        out.b = p;
        break;
    case 2:
        out.r = p;
        out.g = in.v;
        out.b = t;
        break;

    case 3:
        out.r = p;
        out.g = q;
        out.b = in.v;
        break;
    case 4:
        out.r = t;
        out.g = p;
        out.b = in.v;
        break;
    case 5:
    default:
        out.r = in.v;
        out.g = p;
        out.b = q;
        break;
    }

    out.r = out.r * 255;
    out.g = out.g * 255;
    out.b = out.b * 255;

    return out;
}

// Source: https://codeyarns.com/2015/08/27/depth-and-type-of-matrix-in-opencv/
std::string PlotUtils::GetMatDepth(const cv::Mat &mat)
{
    const int depth = mat.depth();

    switch (depth)
    {
    case CV_8U:
        return "CV_8U";
    case CV_8S:
        return "CV_8S";
    case CV_16U:
        return "CV_16U";
    case CV_16S:
        return "CV_16S";
    case CV_32S:
        return "CV_32S";
    case CV_32F:
        return "CV_32F";
    case CV_64F:
        return "CV_64F";
    default:
        return "Invalid depth type of matrix!";
    }
}

// Source: https://codeyarns.com/2015/08/27/depth-and-type-of-matrix-in-opencv/
std::string PlotUtils::GetMatType(const cv::Mat &mat)
{
    const int mtype = mat.type();

    switch (mtype)
    {
    case CV_8UC1:
        return "CV_8UC1";
    case CV_8UC2:
        return "CV_8UC2";
    case CV_8UC3:
        return "CV_8UC3";
    case CV_8UC4:
        return "CV_8UC4";

    case CV_8SC1:
        return "CV_8SC1";
    case CV_8SC2:
        return "CV_8SC2";
    case CV_8SC3:
        return "CV_8SC3";
    case CV_8SC4:
        return "CV_8SC4";

    case CV_16UC1:
        return "CV_16UC1";
    case CV_16UC2:
        return "CV_16UC2";
    case CV_16UC3:
        return "CV_16UC3";
    case CV_16UC4:
        return "CV_16UC4";

    case CV_16SC1:
        return "CV_16SC1";
    case CV_16SC2:
        return "CV_16SC2";
    case CV_16SC3:
        return "CV_16SC3";
    case CV_16SC4:
        return "CV_16SC4";

    case CV_32SC1:
        return "CV_32SC1";
    case CV_32SC2:
        return "CV_32SC2";
    case CV_32SC3:
        return "CV_32SC3";
    case CV_32SC4:
        return "CV_32SC4";

    case CV_32FC1:
        return "CV_32FC1";
    case CV_32FC2:
        return "CV_32FC2";
    case CV_32FC3:
        return "CV_32FC3";
    case CV_32FC4:
        return "CV_32FC4";

    case CV_64FC1:
        return "CV_64FC1";
    case CV_64FC2:
        return "CV_64FC2";
    case CV_64FC3:
        return "CV_64FC3";
    case CV_64FC4:
        return "CV_64FC4";

    default:
        return "Invalid type of matrix!";
    }
}