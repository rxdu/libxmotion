/* 
 * polygon_viz.cpp
 * 
 * Created on: Aug 10, 2018 10:17
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lightviz/polygon_viz.hpp"

#include "lightviz/details/geometry_draw.hpp"

using namespace librav;
using namespace LightViz;

void LightViz::ShowPolygon(const Polygon &polygon, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    double xspan = polygon.GetMaxX() - polygon.GetMinX();
    double yspan = polygon.GetMaxY() - polygon.GetMinY();

    double xmin = polygon.GetMinX() - xspan * 0.2;
    double xmax = polygon.GetMaxX() + xspan * 0.2;
    double ymin = polygon.GetMinY() - yspan * 0.2;
    double ymax = polygon.GetMaxY() + yspan * 0.2;

    cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);
    canvas = gdraw.DrawPolygon(canvas, polygon);

    ShowImage(canvas, window_name, save_img);
}
