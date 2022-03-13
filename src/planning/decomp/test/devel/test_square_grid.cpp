#include "decomp/square_grid.hpp"

using namespace robosw;

int main() {
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

  CvCanvas canvas = CreateCanvas(grid, 200);

  DrawGridCell(canvas, grid);
  DrawGridCost(canvas, grid);
  DrawGridNet(canvas, grid);

  canvas.Show();

  return 0;
}