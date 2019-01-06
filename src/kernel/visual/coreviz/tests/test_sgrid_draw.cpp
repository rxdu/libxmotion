#include "decomp/square_grid.hpp"
#include "coreviz/coreviz.hpp"

using namespace librav;

int main()
{
    SquareGrid grid(30, 20);

    for (int i = 0; i < 30; ++i)
        grid.SetCellLabel(i, 0, SquareCellLabel::OCCUPIED);

    for (int i = 15; i < 19; ++i)
        grid.SetCellLabel(i, 3, SquareCellLabel::OCCUPIED);

    for (int i = 0; i < 20; ++i)
        grid.SetCellLabel(20, i, SquareCellLabel::OCCUPIED);

    for (int i = 0; i < 15; ++i)
        grid.SetCellLabel(i, 19, SquareCellLabel::OCCUPIED);

    std::vector<SquareCell *> path;
    path.push_back(grid.GetCell(15));
    path.push_back(grid.GetCell(16));
    path.push_back(grid.GetCell(17));
    path.push_back(grid.GetCell(18));
    path.push_back(grid.GetCell(48));

    auto canvas = SquareGridDraw::CreateCanvas(&grid);
    SquareGridDraw::DrawGridCell(canvas, &grid);
    SquareGridDraw::DrawGridPathStartGoal(canvas, path);
    SquareGridDraw::DrawGridNet(canvas, &grid);
    SquareGridDraw::DrawGridPath(canvas, path);
    canvas.Show();

    return 0;
}