#include "lightviz/lightviz.hpp"
#include "lightviz/details/geometry_draw.hpp"
#include "lightviz/polygon_viz.hpp"
#include "polygon/polygon.hpp"

using namespace librav;

int main()
{
    // GeometryDraw gdraw(50);
    // cv::Mat canvas = gdraw.CreateCanvas(-8, 8, -5, 5);
    // LightViz::ShowImage(canvas, "test geometry viz");

    Polygon polygon;
    polygon.AddPoint(0, 0);
    polygon.AddPoint(1, 0);
    polygon.AddPoint(1, 0.6);
    polygon.AddPoint(0, 0.6);
    
    // auto trans1 = polygon.TransformRT(1,1,M_PI/4.0);
    // trans1.PrintInfo();

    // auto trans2 = polygon.TransformTR(1,1,M_PI/4.0);
    // trans2.PrintInfo();

    LightViz::ShowPolygon(polygon, 100);

    return 0;
}