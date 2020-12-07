#include "geometry/polygon_viz.hpp"
#include "geometry/polygon.hpp"

using namespace robotnav;

int main()
{
    // GeometryViz gdraw(50);
    // cv::Mat canvas = gdraw.CreateCanvas(-8, 8, -5, 5);
    // LightViz::ShowImage(canvas, "test geometry viz");

    Polygon polygon;
    polygon.AddPoint(0, 0);
    polygon.AddPoint(1, 0);
    polygon.AddPoint(1, 0.6);
    polygon.AddPoint(0, 0.6);

    LightViz::ShowPolygon(polygon, 100);

    auto trans1 = polygon.TransformRT(1, 1, M_PI / 4.0);
    trans1.PrintInfo();

    auto trans2 = polygon.TransformTR(1, 1, M_PI / 4.0);
    trans2.PrintInfo();

    std::vector<Polygon> polys;
    polys.push_back(polygon);
    polys.push_back(trans1);
    polys.push_back(trans2);
    LightViz::ShowPolygon(polys, 200);

    return 0;
}