/* 
 * field_viz.cpp
 * 
 * Created on: Aug 12, 2018 23:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_viz/field_viz_ext.hpp"

#include <cmath>

#include "lightviz/details/geometry_draw.hpp"

using namespace librav;

void LightViz::ShowTrafficParticipant(std::shared_ptr<TrafficParticipant> participant, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(0, 80, 0, 60, CvDrawColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, participant->position_x, participant->position_y, 20, 20, std::function<double(double, double)>(participant->threat_func));

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowCollisionField(std::shared_ptr<CollisionField> cfield, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    cv::Mat canvas = gdraw.CreateCanvas(cfield->xmin_, cfield->xmax_, cfield->ymin_, cfield->ymax_, CvDrawColors::jet_colormap_lowest);

    std::cout << "params: " << cfield->GetMeanX() << " , " << cfield->GetMeanY() << " , " << cfield->GetSpanX() << " , " << cfield->GetSpanY() << std::endl;

    canvas = gdraw.DrawDistribution(canvas, cfield->GetMeanX(), cfield->GetMeanY(), cfield->GetSpanX(), cfield->GetSpanY(), *cfield.get());

    ShowImage(canvas, window_name, save_img);
}

void LightViz::ShowTrafficParticipantThreat(std::shared_ptr<TrafficParticipant> participant, const Polygon &polygon, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    GeometryDraw gdraw(pixel_per_unit);

    // double xspan = polygon.GetMaxX() - polygon.GetMinX();
    // double yspan = polygon.GetMaxY() - polygon.GetMinY();

    // double xmin = polygon.GetMinX() - xspan * 0.2;
    // double xmax = polygon.GetMaxX() + xspan * 0.2;
    // double ymin = polygon.GetMinY() - yspan * 0.2;
    // double ymax = polygon.GetMaxY() + yspan * 0.2;

    // cv::Mat canvas = gdraw.CreateCanvas(xmin, xmax, ymin, ymax);
    cv::Mat canvas = gdraw.CreateCanvas(-12, 12, -15, 15, CvDrawColors::jet_colormap_lowest);

    canvas = gdraw.DrawDistribution(canvas, participant->position_x, participant->position_y, 20, 20, std::function<double(double, double)>(participant->threat_func));
    canvas = gdraw.DrawPolygon(canvas, polygon, false, CvDrawColors::black_color, 2);

    ShowImage(canvas, window_name, save_img);
}
