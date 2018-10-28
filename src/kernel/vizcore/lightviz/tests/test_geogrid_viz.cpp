#include "decomp/curvilinear_grid.hpp"
#include "lightviz/details/geometry_draw.hpp"

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

    CartesianCanvas canvas(200);
    canvas.SetupCanvas(-1, 3, -1, 3);

    GeometryDraw gdraw(canvas);
    gdraw.DrawCurvilinearGrid(grid, 0.1, true);

    CvDraw::ShowImage(canvas.paint_area, "curvilinear_grid");

    return 0;
}