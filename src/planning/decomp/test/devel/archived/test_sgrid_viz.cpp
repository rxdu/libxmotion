#include "decomp/square_grid.hpp"
#include "decomp/square_grid_viz.hpp"

using namespace xmotion;

int main()
{
    SquareGrid grid(30, 20);

    // for (int i = 0; i < 30; ++i)
    //     grid.SetCellLabel(i, 0, SquareCellLabel::OCCUPIED);

    for (int i = 0; i < 15; ++i)
        grid.SetCellLabel(i, 5, SquareCellLabel::OCCUPIED);

    for (int i = 0; i < 15; ++i)
        grid.SetCellLabel(i, 19, SquareCellLabel::OCCUPIED);

    std::vector<SquareCell *> path;
    path.push_back(grid.GetCell(15));
    path.push_back(grid.GetCell(16));
    path.push_back(grid.GetCell(17));
    path.push_back(grid.GetCell(18));
    path.push_back(grid.GetCell(48));

    LightViz::ShowSquareGrid(&grid, 200);
    LightViz::ShowSquareGridPath(&grid, path, 200);

    return 0;
}