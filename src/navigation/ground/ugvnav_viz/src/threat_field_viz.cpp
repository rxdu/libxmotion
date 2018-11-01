/* 
 * threat_field_viz.cpp
 * 
 * Created on: Aug 12, 2018 23:57
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/threat_field_viz.hpp"

#include <cmath>

#include "lightviz/details/cartesian_canvas.hpp"
#include "lightviz/details/geometry_draw.hpp"

using namespace librav;
using namespace CvDraw;

void LightViz::ShowTrafficParticipant(std::shared_ptr<TrafficParticipant> participant, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    GeometryDraw gdraw(canvas);

    canvas.SetupCanvas(0, 80, 0, 60, CvDrawColors::jet_colormap_lowest);

    gdraw.DrawDistribution(participant->position_x, participant->position_y, 20, 20, std::function<double(double, double)>(participant->threat_func));

    ShowImage(canvas.paint_area, window_name, save_img);
}

void LightViz::ShowCollisionField(std::shared_ptr<CollisionField> cfield, bool show_wp, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    GeometryDraw gdraw(canvas);

    canvas.SetupCanvas(cfield->xmin_, cfield->xmax_, cfield->ymin_, cfield->ymax_, CvDrawColors::jet_colormap_lowest);

    std::cout << "params: " << cfield->GetMeanX() << " , " << cfield->GetMeanY() << " , " << cfield->GetSpanX() << " , " << cfield->GetSpanY() << std::endl;

    gdraw.DrawDistribution(cfield->GetMeanX(), cfield->GetMeanY(), cfield->GetSpanX(), cfield->GetSpanY(), *cfield.get());

    ShowImage(canvas.paint_area, window_name, save_img);
}

void LightViz::ShowTrafficParticipantThreat(std::shared_ptr<TrafficParticipant> participant, const Polygon &polygon, int32_t pixel_per_unit, std::string window_name, bool save_img)
{
    CartesianCanvas canvas(pixel_per_unit);
    GeometryDraw gdraw(canvas);

    canvas.SetupCanvas(-12, 12, -15, 15, CvDrawColors::jet_colormap_lowest);

    gdraw.DrawDistribution(participant->position_x, participant->position_y, 20, 20, std::function<double(double, double)>(participant->threat_func));
    gdraw.DrawPolygon(polygon, false, CvDrawColors::black_color, 2);

    ShowImage(canvas.paint_area, window_name, save_img);
}
