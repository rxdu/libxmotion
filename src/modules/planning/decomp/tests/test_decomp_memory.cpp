#include <iostream>
#include <iomanip>

#include "decomp/curvilinear_grid.hpp"
#include "decomp/dense_grid.hpp"
#include "decomp/square_grid.hpp"

using namespace autodrive;

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

    /////////////////////////////////////////////////////////

    CurvilinearGrid cgrid(curve, 0.5, 0.5, 2);
    CurvilinearGrid cgrid2 = cgrid;
    CurvilinearGrid cgrid3(CurvilinearGrid(curve, 0.5, 0.5, 2));
    CurvilinearGrid cgrid4 = CurvilinearGrid(curve, 0.5, 0.5, 2);

    /////////////////////////////////////////////////////////

    const int size_x = 500;
    const int size_y = 600;

    DenseGrid dgrid(size_x, size_y);
    dgrid.SetValueAtCoordinate(0, 0, 1.0);
    dgrid.SetValueAtCoordinate(1, 2, 1.2);
    dgrid.SetValueAtCoordinate(4, 5, 2.2);

    DenseGrid dgrid2 = dgrid;
    DenseGrid dgrid3;
    dgrid3 = dgrid;
    DenseGrid dgrid4(DenseGrid(size_x, size_y));
    DenseGrid dgrid5;
    dgrid5 = DenseGrid(size_x, size_y);

    /////////////////////////////////////////////////////////

    SquareGrid sgrid(10, 5);
    for (int i = 0; i < 5; ++i)
        sgrid.SetCellLabel(i, 0, SquareCellLabel::OCCUPIED);

    SquareGrid sgrid2 = sgrid;
    SquareGrid sgrid3(SquareGrid(10, 5));
    SquareGrid sgrid4 = SquareGrid(10, 5);

    return 0;
}