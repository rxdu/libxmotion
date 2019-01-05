#include "decomp/square_grid.hpp"
// #include "lightviz/square_grid_viz.hpp"

using namespace librav;

int main()
{
    SquareGrid grid(10, 5);

    // for (int i = 0; i < 30; ++i)
    //     grid.SetCellLabel(i, 0, SquareCellLabel::OCCUPIED);

    // for (int i = 0; i < 15; ++i)
    //     grid.SetCellLabel(i, 5, SquareCellLabel::OCCUPIED);

    // for (int i = 0; i < 15; ++i)
    //     grid.SetCellLabel(i, 19, SquareCellLabel::OCCUPIED);

    // std::vector<SquareCell *> path;
    // path.push_back(grid.GetCell(15));
    // path.push_back(grid.GetCell(16));
    // path.push_back(grid.GetCell(17));
    // path.push_back(grid.GetCell(18));
    // path.push_back(grid.GetCell(48));

    // LightViz::ShowSquareGrid(&grid, 500, "square_grid.png");
    // LightViz::ShowSquareGridPath(&grid, path, 100);

    return 0;
}