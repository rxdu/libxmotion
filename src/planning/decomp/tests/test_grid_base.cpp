#include <iostream>
#include <iomanip>

#include "decomp/grid_base.hpp"

using namespace librav;

int main()
{
    const int size_x = 10;
    const int size_y = 12;

    GridBase<double> grid(size_x, size_y);
    // grid.SetOriginCoordinate(5, 6);

    // grid.SetTileAtGridCoordinate(-2, -1, -1.0);
    // grid.SetTileAtGridCoordinate(0, 0, 1.0);
    // grid.SetTileAtGridCoordinate(1, 2, 1.2);
    // grid.GetTileRefAtGridCoordinate(1, 2) = 1.3;

    grid.PrintGrid();

    std::cout << "after resize:\n"
              << std::endl;

    // grid.ResizeGrid(8, 9);   
    grid.ResizeGrid(12, 15);

    grid.PrintGrid();

    return 0;
}