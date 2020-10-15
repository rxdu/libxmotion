#include "decomp/curvilinear_grid.hpp"
#include "decomp/curvilinear_grid_draw.hpp"

using namespace ivnav;

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

    std::cout << "curve length: " << curve.GetLength() << std::endl;

    // create curvilinear grid
    // CurvilinearGrid grid(curve, 0.1, 0.5, 3);
    CurvilinearGrid grid(curve, 0.41, 0.2, 5, 0.2);

    std::cout << "curvilinear grid created" << std::endl;

    CvCanvas canvas(100);
    canvas.Resize(-1, 3, -1, 3);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    CurvilinearGridViz::FillCurvilinearGrid(canvas, grid);
    CurvilinearGridViz::DrawCurvilinearGrid(canvas, grid, CvColors::black_color);

    canvas.Show();

    return 0;
}