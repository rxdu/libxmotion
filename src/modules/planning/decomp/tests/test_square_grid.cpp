#include "decomp/square_grid.hpp"

#define ENABLE_VIZ

#ifdef ENABLE_VIZ
#include "lightviz/square_grid_viz.hpp"
#endif

using namespace autodrive;

int main()
{
    SquareGrid grid(30, 20);

    for (int i = 0; i < 30; ++i)
        grid.SetCellLabel(i, 0, SquareCellLabel::OCCUPIED);

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

#ifdef ENABLE_VIZ
    // LightViz::ShowSquareGrid(&grid, 500, "square_grid.png");
    LightViz::ShowSquareGridPath(&grid, path, 100);
#endif

    return 0;
}