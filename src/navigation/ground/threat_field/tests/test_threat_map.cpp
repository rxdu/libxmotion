#include <iostream>
#include <cstdint>
#include <cmath>

#include "geometry/polygon.hpp"
#include "threat_field/threat_map.hpp"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/navviz.hpp"
#include "coreviz/geometry_draw.hpp"
#endif

using namespace librav;

int main()
{
    ThreatMap tmap(50.0, 50.0);

    ///////////////////////////////////////////////////////////
#ifdef ENABLE_VIZ
    // setup canvas
    CvCanvas canvas(10, CvColors::jet_colormap_lowest);
    canvas.Resize(0, 100, 0, 100);

    // draw distribution
    GeometryViz::DrawDistribution(canvas, 50, 50, 100, 100, std::bind(tmap, std::placeholders::_1, std::placeholders::_2, 15, 15));
    CvIO::ShowImage(canvas.GetPaintArea(), "skewed_quadratic2",true);
#endif

    return 0;
}