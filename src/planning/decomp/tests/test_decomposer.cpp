#include <iostream>
#include <iomanip>

#include "decomp/dense_grid.hpp"
#include "decomp/grid_decomposer.hpp"

#include "lightviz/matrix_viz.hpp"
#include "lightviz/grid_viz.hpp"

using namespace librav;

int main()
{
    // const int size_x = 500;
    // const int size_y = 600;
    const int size_x = 502;
    const int size_y = 601;

    DenseGrid grid(size_x, size_y);
    // grid.SetOriginCoordinate(6, 6);

    // grid.SetTileAtGridCoordinate(-2, -1, -1.0);
    // grid.SetValueAtCoordinate(0, 0, 1.0);
    // grid.SetValueAtCoordinate(1, 2, 1.2);
    // grid.SetValueAtCoordinate(4, 5, 2.2);

    for (int i = 20; i < 100; ++i)
        for (int j = 50; j < 200; ++j)
            grid.SetValueAtCoordinate(i, j, 4.2);

    // LightViz::ShowMatrixAsImage(grid.GetGridMatrix(true)*128);
    LightViz::ShowMatrixAsColorMap(grid.GetGridMatrix(true), "matrix", true);

    auto sgrid = GridDecomposer::CreateSquareGridFromMatrix(grid.GetGridMatrix(false), 10);
    LightViz::ShowSquareGrid(sgrid.get(), 100, "Square Grid", true);


    return 0;
}