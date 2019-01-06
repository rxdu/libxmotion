#include <iostream>
#include <cstdint>
#include <cmath>

#include "coreviz/geometry_draw.hpp"
#include "geometry/polygon.hpp"
#include "field_decomp/threat_map.hpp"

using namespace librav;

int main()
{
    ThreatMap moving_tmap(20.0, -2.0);
    ThreatMap static_tmap(-20.0, 0.0);

    ///////////////////////////////////////////////////////////
    // setup canvas
    CvCanvas canvas(10, CvColors::jet_colormap_lowest);
    canvas.Resize(-50, 50, -17, 13);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    // draw distribution
    GeometryViz::DrawDistribution(canvas, 20, 0, 50, 50, std::bind(moving_tmap, std::placeholders::_1, std::placeholders::_2, 15, 15));
    GeometryViz::DrawDistribution(canvas, -20, 0, 50, 50, std::bind(static_tmap, std::placeholders::_1, std::placeholders::_2, 0, 0));

    CvIO::ShowImage(canvas.GetPaintArea(), "threat_map", true);

    return 0;
}