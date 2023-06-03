/*
 * color_maps.cpp
 *
 * Created on: Jan 04, 2019 09:52
 * Description:
 *
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "cvdraw/color_maps.hpp"

using namespace cv;

namespace xmotion {
namespace {
// Source:
// https://stackoverflow.com/questions/7706339/grayscale-to-red-green-blue-matlab-jet-color-scale?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
double JetInterpolate(double val, double y0, double x0, double y1, double x1) {
  return (val - x0) * (y1 - y0) / (x1 - x0) + y0;
}

double JetBase(double val) {
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

double JetRed(double gray) { return JetBase(gray - 0.5) * 255; }

double JetGreen(double gray) { return JetBase(gray) * 255; }

double JetBlue(double gray) { return JetBase(gray + 0.5) * 255; }
}  // namespace

/**********************************************************************/

cv::Scalar JetColorMap::Transform(double val) {
  return cv::Scalar(JetBlue(val), JetGreen(val), JetRed(val));
}

void JetColorMap::Transform(double val, double &r, double &g, double &b) {
  r = JetRed(val);
  g = JetGreen(val);
  b = JetBlue(val);
}
}  // namespace xmotion