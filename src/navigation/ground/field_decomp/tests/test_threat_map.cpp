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
    ThreatMap tmap(0.0, 0.0);

    ///////////////////////////////////////////////////////////
    // setup canvas
    CartesianCanvas canvas(10);
    canvas.SetupCanvas(-50, 50, -50, 50, CvDrawColors::jet_colormap_lowest);
    GeometryDraw gdraw(canvas);

    // draw distribution
    gdraw.DrawDistribution(0, 0, 50, 50, std::bind(tmap, std::placeholders::_1, std::placeholders::_2, -5, -20));
    CvDraw::ShowImage(canvas.paint_area, "test field viz");

    return 0;
}