/* 
 * grid_base.hpp
 * 
 * Created on: Mar 28, 2018 23:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GRID_BASE_HPP
#define GRID_BASE_HPP

#include <cstdint>
#include <vector>
#include <tuple>
#include <iomanip>
#include <iostream>
#include <type_traits>

#include <Eigen/Dense>

#include "details/grid_base_tiles.hpp"

namespace librav
{

/*
 * 
 *  Grid Coordinate System:
 * 
 *              |
 *              |
 *              |
 *              |
 *              |
 *   ---------- o ---------> x
 *              |
 *              |
 *              |
 *              | 
 *              v
 *              y
 *                           
 * 
 *  Internal Coordinate System:
 *		
 *		o ------------------> x  
 *		|         |
 *		|         |
 *		|         |
 *		| --------o' (origin offset)
 *		|         
 *		|         
 *		|                 
 *		v
 *		y
 * 
 */

////////////////////////////////////////////////////////////////////

class GridCoordinate
{
public:
  GridCoordinate(int64_t x = 0, int64_t y = 0)
  {
    coordinate_x_ = x;
    coordinate_y_ = y;
  }
  ~GridCoordinate() = default;

  inline int64_t GetX() const { return coordinate_x_; };
  inline int64_t GetY() const { return coordinate_y_; };
  inline void SetX(int64_t x) { coordinate_x_ = x; };
  inline void SetY(int64_t y) { coordinate_y_ = y; };
  inline void SetXY(int64_t x, int64_t y)
  {
    coordinate_x_ = x;
    coordinate_y_ = y;
  };

private:
  int64_t coordinate_x_;
  int64_t coordinate_y_;
};

////////////////////////////////////////////////////////////////////

template <typename TileType>
class GridBase : public GridTiles<TileType>
{
public:
  GridBase(int64_t size_x = 0, int64_t size_y = 0) : size_x_(size_x),
                                                     size_y_(size_y)
  {
    GridTiles<TileType>::ResizeGrid(size_x, size_y);
  }

  int64_t SizeX() const
  {
    return size_x_;
  };
  int64_t SizeY() const { return size_y_; };

  // Note: resizing the grid may invalidate the reference to grid elements
  //    acquired from GetTileRefAtRawCoordinate() and GetTileRefAtGridCoordinate()
  void ResizeGrid(int64_t x, int64_t y)
  {
    if (x == size_x_ && y == size_y_)
      return;

    assert(x > origin_offset_x_ && y > origin_offset_y_);

    GridTiles<TileType>::ResizeGrid(x, y);
    size_x_ = x;
    size_y_ = y;
  }

  void SetOriginCoordinate(int64_t origin_x, int64_t origin_y)
  {
    origin_offset_x_ = origin_x;
    origin_offset_y_ = origin_y;
  }

  // operations WRT coordinate of internal data structure directly
  void SetTileAtRawCoordinate(int64_t x, int64_t y, TileType tile)
  {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    GridTiles<TileType>::SetTileAtCoordinate(x, y, tile);
  }

  TileType GetTileAtRawCoordinate(int64_t x, int64_t y) const
  {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    return GridTiles<TileType>::GetTileAtCoordinate(x, y);
  }

  TileType &GetTileRefAtRawCoordinate(int64_t x, int64_t y)
  {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    return GridTiles<TileType>::GetTileRefAtCoordinate(x, y);
  }

  // operations WRT coordinate of field
  void SetTileAtGridCoordinate(int64_t x, int64_t y, TileType tile)
  {
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    assert((internal_coordinate.GetX() >= 0) && (internal_coordinate.GetX() < size_x_) && (internal_coordinate.GetY() >= 0) && (internal_coordinate.GetY() < size_y_));

    SetTileAtRawCoordinate(internal_coordinate.GetX(), internal_coordinate.GetY(), tile);
  }

  TileType GetTileAtGridCoordinate(int64_t x, int64_t y) const
  {
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    assert((internal_coordinate.GetX() >= 0) && (internal_coordinate.GetX() < size_x_) && (internal_coordinate.GetY() >= 0) && (internal_coordinate.GetY() < size_y_));

    return GetTileAtRawCoordinate(internal_coordinate.GetX(), internal_coordinate.GetY());
  }

  TileType &GetTileRefAtGridCoordinate(int64_t x, int64_t y)
  {
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    assert((internal_coordinate.GetX() >= 0) && (internal_coordinate.GetX() < size_x_) && (internal_coordinate.GetY() >= 0) && (internal_coordinate.GetY() < size_y_));

    return GetTileRefAtRawCoordinate(internal_coordinate.GetX(), internal_coordinate.GetY());
  }

  // convertion between two coordinates
  GridCoordinate ConvertToRawCoordinate(int64_t x, int64_t y) const
  {
    assert((x > -size_x_) && (x < size_x_) && (y > -size_y_) && (y < size_y_));

    return GridCoordinate(x + origin_offset_x_, y + origin_offset_y_);
  }

  GridCoordinate ConvertToGridCoordinate(int64_t x, int64_t y) const
  {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    return GridCoordinate(x - origin_offset_x_, y - origin_offset_y_);
  }

  template <typename T = TileType, typename std::enable_if<std::is_floating_point<T>::value || std::is_integral<T>::value>::type * = nullptr>
  void PrintGrid() const
  {
    for (int64_t y = 0; y < size_y_; ++y)
    {
      for (int64_t x = 0; x < size_x_; ++x)
        std::cout << std::setw(6) << GridTiles<TileType>::GetTileAtCoordinate(x, y);
      std::cout << std::endl;
    }
  }

protected:
  // internal data structure for a 2D field
  int64_t size_x_;
  int64_t size_y_;

  // origin offset
  int64_t origin_offset_x_ = 0;
  int64_t origin_offset_y_ = 0;
};
}

#endif /* GRID_BASE_HPP */
