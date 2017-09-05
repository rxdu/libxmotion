/*
 * vis_utils.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: rdu
 */

#include "vis/vis_utils.h"

using namespace srcl_ctrl;
using namespace cv;

// frequently used colors
cv::Scalar VisUtils::default_pt_color_ = Scalar( 0, 0, 255 );
cv::Scalar VisUtils::default_ln_color_ = Scalar(Scalar(0,0,0));
cv::Scalar VisUtils::default_area_color_ = Scalar(0,255,255);

cv::Scalar VisUtils::bk_color_ = Scalar(255,255,255);
cv::Scalar VisUtils::ln_color_ = Scalar(Scalar(0,0,0));
cv::Scalar VisUtils::obs_color_ = Scalar(Scalar(0,102,204));
cv::Scalar VisUtils::aoi_color_ = Scalar(Scalar(0,255,255));
cv::Scalar VisUtils::start_color_ = Scalar(0,0,255);
cv::Scalar VisUtils::finish_color_ = Scalar(153,76,0);

void VisUtils::DrawPoint(cv::Mat img, cv::Point pos, const cv::Scalar& color)
{
	int thickness = -1;
	int lineType = 8;
	Point center(pos.x,pos.y);
	circle( img,
			center,
			3,
			Scalar( 0, 0, 255 ),
			thickness,
			lineType);
}

void VisUtils::DrawLine(cv::Mat img, cv::Point pt1, cv::Point pt2, const cv::Scalar& color)
{
	int thickness = 1;
	int lineType = 8;

	line( img,
		  pt1,
	      pt2,
		  Scalar( 237, 149, 100 ),
		  thickness,
		  lineType);
}

// Source:
//	* https://adishavit.github.io/2015/drawing-arrows-with-opencv/
//	* http://docs.opencv.org/2.4/modules/core/doc/drawing_functions.html#arrowedline
void VisUtils::DrawArrow(cv::Mat img, cv::Point base_pos, double length, double angle, const cv::Scalar& color)
{
	int line_type = 8;
	int thickness = 1.5;

	double tip_size = length * 0.1;
	if(tip_size < 1)
		tip_size = 1;

	double angleRad = angle*CV_PI / 180.0;   // convert angle to radians
	cv::Point tip_pt = cv::Point(base_pos.x + length * sin(angleRad), base_pos.y - length * cos(angleRad)); // calculate tip position

	line(img, base_pos, tip_pt, color, thickness, line_type);

	double angle_l = atan2( (double) base_pos.y - tip_pt.y, (double) base_pos.x - tip_pt.x );

	Point p(cvRound(tip_pt.x + tip_size * cos(angle_l + CV_PI / 4)),
			cvRound(tip_pt.y + tip_size * sin(angle_l + CV_PI / 4)));
	line(img, p, tip_pt, color, thickness, line_type);

	p.x = cvRound(tip_pt.x + tip_size * cos(angle_l - CV_PI / 4));
	p.y = cvRound(tip_pt.y + tip_size * sin(angle_l - CV_PI / 4));
	line(img, p, tip_pt, color, thickness, line_type);
}

void VisUtils::FillRectangularArea(cv::Mat img, BoundingBox bbox, const cv::Scalar& color)
{
	Range rngx(bbox.x.min, bbox.x.max);
	Range rngy(bbox.y.min, bbox.y.max);
	img(rngy,rngx) = color;
}

// Source: http://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
HSVColor VisUtils::RGB2HSV(RGBColor in)
{
    HSVColor         out;

    in.r = in.r / 255.0;
    in.g = in.g / 255.0;
    in.b = in.b / 255.0;

    double      min, max, delta;

    min = in.r < in.g ? in.r : in.g;
    min = min  < in.b ? min  : in.b;

    max = in.r > in.g ? in.r : in.g;
    max = max  > in.b ? max  : in.b;

    out.v = max;                                // v
    delta = max - min;
    if (delta < 0.00001)
    {
        out.s = 0;
        out.h = 0; // undefined, maybe nan?
        return out;
    }
    if( max > 0.0 ) { // NOTE: if Max is == 0, this divide would cause a crash
        out.s = (delta / max);                  // s
    } else {
        // if max is 0, then r = g = b = 0
            // s = 0, v is undefined
        out.s = 0.0;
        out.h = NAN;                            // its now undefined
        return out;
    }
    if( in.r >= max )                           // > is bogus, just keeps compilor happy
        out.h = ( in.g - in.b ) / delta;        // between yellow & magenta
    else
    if( in.g >= max )
        out.h = 2.0 + ( in.b - in.r ) / delta;  // between cyan & yellow
    else
        out.h = 4.0 + ( in.r - in.g ) / delta;  // between magenta & cyan

    out.h *= 60.0;                              // degrees

    if( out.h < 0.0 )
        out.h += 360.0;

    return out;
}


RGBColor VisUtils::HSV2RGB(HSVColor in)
{
    double      hh, p, q, t, ff;
    long        i;
    RGBColor         out;

    if(in.s <= 0.0) {       // < is bogus, just shuts up warnings
        out.r = in.v;
        out.g = in.v;
        out.b = in.v;

        out.r = out.r*255;
        out.g = out.g*255;
        out.b = out.b*255;

        return out;
    }

    hh = in.h;
    if(hh >= 360.0) hh = 0.0;
    hh /= 60.0;
    i = (long)hh;
    ff = hh - i;
    p = in.v * (1.0 - in.s);
    q = in.v * (1.0 - (in.s * ff));
    t = in.v * (1.0 - (in.s * (1.0 - ff)));

    switch(i) {
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

    out.r = out.r*255;
    out.g = out.g*255;
    out.b = out.b*255;

    return out;
}
