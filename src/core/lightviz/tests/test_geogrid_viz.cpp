#include "decomp/curvilinear_grid.hpp"
#include "lightviz/details/geometric_draw.hpp"

using namespace librav;

int main()
{
    // generate a parametric curve
    Polyline polyline;
    polyline.AddPoint(0, 0);
    polyline.AddPoint(0.5, 0.25);
    polyline.AddPoint(1.0, 1.0);
    polyline.AddPoint(1.5, 1.75);
    polyline.AddPoint(2.0, 2);

    auto curve = CurveFitting::FitApproximateLengthCurve(polyline);

    // create curvilinear grid
    // CurvilinearGrid grid(curve, 0.1, 0.5, 3);
    CurvilinearGrid grid(curve, 0.41, 0.2, 5);

    GeometryDraw gdraw(200);

    cv::Mat canvas = gdraw.CreateCanvas(-1, 3, -1, 3);
    canvas = gdraw.DrawCurvilinearGrid(canvas, grid, 0.1, true);

    LightViz::ShowImage(canvas, "curvilinear_grid");

    return 0;
}