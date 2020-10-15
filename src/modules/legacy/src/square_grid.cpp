/*
 * square_grid.cpp
 *
 * Created on: Jan 28, 2016
 * Description:
 *
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "map/square_grid.hpp"

#include <iostream>

using namespace ivnav;

// Note: Assertions are for debugging purpose. Since assert() will be expanded
// as void if "NDEBUG" is defined, you should not rely on it to do online
// condition checking in Release code.
#define ASSERT_COORDINATE_RANGE(col, row) \
  assert(col >= 0 && row >= 0 && col < col_size_ && row < row_size_)

#define ASSERT_ID_RANGE(id) assert(id >= 0 && id < col_size_ * row_size_)

////////////////////////////////////////////////////////////////////////////////

SquareCell::SquareCell(int64_t id, int32_t col, int32_t row,
                       OccupancyType occupancy)
    : id_(id), occupancy_(occupancy)
{
  assert((col >= 0 && row >= 0));

  coordinate_.x = col;
  coordinate_.y = row;
}

void SquareCell::UpdateMapInfo(int32_t row_size, int32_t col_size, double side_size, int32_t pixel_per_meter)
{
  int32_t vis_side_size = side_size * pixel_per_meter;

  bounding_box_.x.min = coordinate_.x * vis_side_size;
  bounding_box_.x.max = bounding_box_.x.min + vis_side_size - 1;
  bounding_box_.y.min = (row_size - 1 - coordinate_.y) * vis_side_size;
  bounding_box_.y.max = bounding_box_.y.min + vis_side_size - 1;

  position_.x = coordinate_.x * side_size + side_size/2;
	position_.y = coordinate_.y * side_size + side_size/2;
}

void SquareCell::PrintInfo()
{
  std::string occu;
  switch (occupancy_)
  {
  case OccupancyType::FREE:
    occu = "free";
    break;
  case OccupancyType::OCCUPIED:
    occu = "occupied";
    break;
  case OccupancyType::UNKONWN:
  default:
    occu = "unknown";
    break;
  }
  std::cout << "Cell - id: " << id_ << ", coordinate: (" << coordinate_.x << ","
            << coordinate_.y << ")"
            << ", position: (" << position_.x << ","
            << position_.y << ")"
            << ", occupancy: " << occu << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

SquareGrid::SquareGrid(int32_t row_num, int32_t col_num, double cell_size, int32_t pixel_per_meter)
    : row_size_(row_num), col_size_(col_num), cell_size_(cell_size), pixel_per_meter_(pixel_per_meter)
{
  assert((row_num > 0 && col_num > 0));

  grid_cells_.resize(col_num);
  for (auto &grid_col : grid_cells_)
    grid_col.resize(row_num);

  for (int32_t y = 0; y < row_num; y++)
    for (int32_t x = 0; x < col_num; x++)
    {
      int64_t new_id = y * col_num + x;
      SquareCell *new_cell = new SquareCell(new_id, x, y, OccupancyType::FREE);
      grid_cells_[x][y] = new_cell;
      grid_cells_[x][y]->UpdateMapInfo(row_size_, col_size_, cell_size_, pixel_per_meter_);
    }
}

// This should be the only place to delete SquareGrid objects
SquareGrid::~SquareGrid()
{
  for (auto &grid_col : grid_cells_)
    for (auto &cell : grid_col)
      delete cell;
}

void SquareGrid::SetCellOccupancy(int32_t x_col, int32_t y_row, OccupancyType occ)
{
  ASSERT_COORDINATE_RANGE(x_col, y_row);

  grid_cells_[x_col][y_row]->occupancy_ = occ;
}

void SquareGrid::SetCellOccupancy(int64_t id, OccupancyType occ)
{
  ASSERT_ID_RANGE(id);

  Position2Di coord = GetCoordinateFromID(id);
  SetCellOccupancy(coord.x, coord.y, occ);
}

Position2Di SquareGrid::GetCoordinateFromID(int64_t id)
{
  ASSERT_ID_RANGE(id);
  int32_t y = id / col_size_;
  int32_t x = id % col_size_;

  return Position2Di(x, y);
}

int64_t SquareGrid::GetIDFromCoordinate(int32_t x_col, int32_t y_row)
{
  ASSERT_COORDINATE_RANGE(x_col, y_row);

  return y_row * col_size_ + x_col;
}

SquareCell *SquareGrid::GetCellFromID(int64_t id)
{
  ASSERT_ID_RANGE(id);
  Position2Di coord = GetCoordinateFromID(id);
  return grid_cells_[coord.x][coord.y];
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(int64_t id, bool allow_diag)
{
  ASSERT_ID_RANGE(id);

  auto cell = GetCellFromID(id);

  return GetNeighbours(cell->coordinate_.x, cell->coordinate_.y, allow_diag);
}

std::vector<SquareCell *> SquareGrid::GetNeighbours(int32_t x_col, int32_t y_row, bool allow_diag)
{
  ASSERT_COORDINATE_RANGE(x_col, y_row);

  std::vector<SquareCell *> neighbours;

  // not consider diagonal cells
  if (allow_diag)
  {
    for (int32_t x = x_col - 1; x <= x_col + 1; ++x)
      for (int32_t y = y_row - 1; y <= y_row + 1; ++y)
      {
        if (x == x_col && y == y_row)
          continue;

        if (x >= 0 && x < col_size_ && y >= 0 && y < row_size_)
          neighbours.push_back(grid_cells_[x][y]);
      }
  }
  else
  {
    Position2Di pos[4];

    pos[0].x = x_col;
    pos[0].y = y_row + 1;

    pos[1].x = x_col;
    pos[1].y = y_row - 1;

    pos[2].x = x_col + 1;
    pos[2].y = y_row;

    pos[3].x = x_col - 1;
    pos[3].y = y_row;

    for (int i = 0; i < 4; i++)
    {
      if (pos[i].x >= 0 && pos[i].x < col_size_ &&
          pos[i].y >= 0 && pos[i].y < row_size_)
        neighbours.push_back(grid_cells_[pos[i].x][pos[i].y]);
    }
  }

  return neighbours;
}
