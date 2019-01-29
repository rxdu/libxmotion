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
    ThreatMap basis(0.0, 0.0);

    ///////////////////////////////////////////////////////////
#ifdef ENABLE_VIZ
    // setup canvas
    CvCanvas canvas(10, CvColors::jet_colormap_lowest);
    canvas.Resize(-50, 50, -50, 50);

    // draw distribution
    GeometryViz::DrawDistribution(canvas, 0, 0, 50, 50, std::bind(basis, std::placeholders::_1, std::placeholders::_2, 10, 10));
    CvIO::ShowImage(canvas.GetPaintArea(), "test field viz");
#endif

    return 0;
}