#include <iostream>
#include <iomanip>

#include "decomp/curvilinear_grid.hpp"

using namespace robotnav;

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
    CurvilinearGrid grid(curve, 0.5, 0.5, 2);
    // grid.SetOriginCoordinate(5, 6);

    grid.PrintInfo();

    // grid.SetTileAtGridCoordinate(-2, -1, -1.0);
    // grid.SetTileAtGridCoordinate(0, 0, 1.0);
    // grid.SetTileAtGridCoordinate(1, 2, 1.2);
    // grid.GetTileRefAtGridCoordinate(1, 2) = 1.3;

    // grid.PrintGrid();

    // std::cout << "after resize:\n"
    //           << std::endl;

    // // grid.ResizeGrid(8, 9);   
    // grid.ResizeGrid(12, 15);

    // grid.PrintGrid();

    CvCanvas canvas(100);
    canvas.Resize(-1, 3, -1, 3);
    canvas.SetMode(CvCanvas::DrawMode::GeometryInvertedY);

    FillCurvilinearGrid(canvas, grid);
    DrawCurvilinearGrid(canvas, grid, CvColors::black_color);

    canvas.Show();

    return 0;
}