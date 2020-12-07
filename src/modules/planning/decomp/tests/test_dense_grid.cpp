#include <iostream>
#include <iomanip>

#define ENABLE_VIZ

#include "decomp/dense_grid.hpp"

#ifdef ENABLE_VIZ
#include "lightviz/matrix_viz.hpp"
#endif 

using namespace robotnav;

int main()
{
    const int size_x = 500;
    const int size_y = 600;

    DenseGrid grid(size_x, size_y);
    // grid.SetOriginCoordinate(6, 6);

    // grid.SetTileAtGridCoordinate(-2, -1, -1.0);
    grid.SetValueAtCoordinate(0, 0, 1.0);
    grid.SetValueAtCoordinate(1, 2, 1.2);
    grid.SetValueAtCoordinate(4, 5, 2.2);

    for (int i = 20; i < 100; ++i)
        for (int j = 50; j < 200; ++j)
            grid.SetValueAtCoordinate(i, j, 4.2);

    // grid.PrintGrid();

    // std::cout << "after resize:\n"
    //           << std::endl;
    // // grid.ResizeGrid(8, 9);
    // grid.ResizeGrid(8, 8);
    // grid.ResizeGrid(8, 5);
    // grid.ResizeGrid(3, 3);
    // grid.PrintGrid();

#ifdef ENABLE_VIZ
    LightViz::ShowMatrixAsImage(grid.GetGridMatrix(true)*128);
    LightViz::ShowMatrixAsColorMap(grid.GetGridMatrix(true));
#endif 

    // std::cout << "value " << grid.GetValueAtCoordinate(0,1)  << " , " << grid.GetValueAtCoordinate(1,2) << std::endl;

    return 0;
}