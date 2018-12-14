#include <iostream>
#include <cstdint>
#include <cmath>

#include "canvas/cv_draw.hpp"
#include "lightviz/lightviz.hpp"
#include "lightviz/details/geometry_draw.hpp"
#include "lightviz/polygon_viz.hpp"
#include "geometry/polygon.hpp"

#include "field_decomp/threat_map.hpp"

using namespace librav;

int main()
{
    ThreatMap moving_tmap(20.0, -2.0);
    ThreatMap static_tmap(-20.0, 0.0);

    ///////////////////////////////////////////////////////////
    // setup canvas
    CartesianCanvas canvas(10);
    canvas.SetupCanvas(-50, 50, -17, 13, CvDrawColors::jet_colormap_lowest);
    GeometryDraw gdraw(canvas);

    // draw distribution
    gdraw.DrawDistribution(20, 0, 50, 50, std::bind(moving_tmap, std::placeholders::_1, std::placeholders::_2, 15, 15));
    gdraw.DrawDistribution(-20, 0, 50, 50, std::bind(static_tmap, std::placeholders::_1, std::placeholders::_2, 0, 0));

    CvDraw::ShowImage(canvas.paint_area, "threat_map", true);

    return 0;
}