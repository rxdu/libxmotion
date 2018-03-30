#include <iostream>
#include <iomanip>

#include "decomp/dense_grid.hpp"
#include "lightviz/matrix_viz.hpp"

using namespace librav;

int main()
{
    const int size_x = 5;
    const int size_y = 6;

    DenseGrid grid(size_x, size_y);
    // grid.SetOriginCoordinate(6, 6);

    // grid.SetTileAtGridCoordinate(-2, -1, -1.0);
    grid.SetValueAtCoordinate(0, 0, 1.0);
    grid.SetValueAtCoordinate(1, 2, 1.2);
    grid.SetValueAtCoordinate(4, 5, 2.2);

    grid.PrintGrid();

    std::cout << "after resize:\n"
              << std::endl;

    // // grid.ResizeGrid(8, 9);
    // grid.ResizeGrid(8, 8);
    // grid.ResizeGrid(8, 5);
    grid.ResizeGrid(3, 3);

    grid.PrintGrid();

    // std::cout << "value " << grid.GetValueAtCoordinate(0,1)  << " , " << grid.GetValueAtCoordinate(1,2) << std::endl;

    return 0;
}