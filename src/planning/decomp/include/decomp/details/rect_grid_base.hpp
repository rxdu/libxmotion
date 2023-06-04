/*
 * rect_grid_base.hpp
 *
 * Created on: Mar 28, 2018 23:34
 * Description: Base class for rectangular grid.
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RECT_GRID_BASE_HPP
#define RECT_GRID_BASE_HPP

#include <cstdint>
#include <vector>
#include <tuple>
#include <iomanip>
#include <iostream>
#include <type_traits>

#include <Eigen/Dense>

#include "decomp/details/rect_grid_base_tiles.hpp"

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

namespace xmotion {
class RectGridIndex {
 public:
  RectGridIndex(int64_t x = 0, int64_t y = 0) : index_x_(x), index_y_(y) {}

  inline int64_t GetX() const { return index_x_; };
  inline int64_t GetY() const { return index_y_; };
  inline void SetX(int64_t x) { index_x_ = x; };
  inline void SetY(int64_t y) { index_y_ = y; };
  inline void SetXY(int64_t x, int64_t y) {
    index_x_ = x;
    index_y_ = y;
  };

 private:
  int64_t index_x_ = 0;
  int64_t index_y_ = 0;
};

////////////////////////////////////////////////////////////////////

template <typename TileType>
class RectGridBase : public GridTiles<TileType> {
 public:
  RectGridBase(int64_t size_x, int64_t size_y)
      : GridTiles<TileType>(size_x, size_y), size_x_(size_x), size_y_(size_y) {}
  virtual ~RectGridBase() = default;

  inline int64_t SizeX() const { return size_x_; };
  inline int64_t SizeY() const { return size_y_; };

  // Note: resizing the grid may invalidate the reference to grid elements
  //    acquired from GetTileRefAtRawCoordinate() and
  //    GetTileRefAtGridCoordinate()
  inline void ResizeGrid(int64_t x, int64_t y) {
    if (x == size_x_ && y == size_y_) return;

    assert(x > origin_offset_x_ && y > origin_offset_y_);

    GridTiles<TileType>::ResizeGridTiles(x, y);
    size_x_ = x;
    size_y_ = y;
  }

  inline void SetOriginCoordinate(int64_t origin_x, int64_t origin_y) {
    origin_offset_x_ = origin_x;
    origin_offset_y_ = origin_y;
  }

  template <
      typename T = TileType,
      typename std::enable_if<std::is_floating_point<T>::value ||
                              std::is_integral<T>::value>::type * = nullptr>
  void PrintGrid() const {
    for (int64_t y = 0; y < size_y_; ++y) {
      for (int64_t x = 0; x < size_x_; ++x)
        std::cout << std::setw(6)
                  << GridTiles<TileType>::GetTileAtCoordinate(x, y);
      std::cout << std::endl;
    }
  }

 protected:
  // operations WRT coordinate of internal data structure directly
  inline void SetTileAtRawCoordinate(int64_t x, int64_t y, TileType tile) {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));
    GridTiles<TileType>::SetTileAtCoordinate(x, y, tile);
  }

  inline TileType GetTileAtRawCoordinate(int64_t x, int64_t y) const {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));
    return GridTiles<TileType>::GetTileAtCoordinate(x, y);
  }

  inline TileType &GetTileRefAtRawCoordinate(int64_t x, int64_t y) {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));
    return GridTiles<TileType>::GetTileRefAtCoordinate(x, y);
  }

  // operations WRT coordinate of field
  inline void SetTileAtGridCoordinate(int64_t x, int64_t y, TileType tile) {
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    SetTileAtRawCoordinate(internal_coordinate.GetX(),
                           internal_coordinate.GetY(), tile);
  }

  inline TileType GetTileAtGridCoordinate(int64_t x, int64_t y) const {
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    return GetTileAtRawCoordinate(internal_coordinate.GetX(),
                                  internal_coordinate.GetY());
  }

  inline TileType &GetTileRefAtGridCoordinate(int64_t x, int64_t y) {
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    return GetTileRefAtRawCoordinate(internal_coordinate.GetX(),
                                     internal_coordinate.GetY());
  }

  // convertion between two coordinates
  inline RectGridIndex ConvertToRawCoordinate(int64_t x, int64_t y) const {
    return RectGridIndex(x + origin_offset_x_, y + origin_offset_y_);
  }

  inline RectGridIndex ConvertToGridCoordinate(int64_t x, int64_t y) const {
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));
    return RectGridIndex(x - origin_offset_x_, y - origin_offset_y_);
  }

 protected:
  // internal data structure for a 2D field
  int64_t size_x_ = 0;
  int64_t size_y_ = 0;

  // origin offset
  int64_t origin_offset_x_ = 0;
  int64_t origin_offset_y_ = 0;
};
}  // namespace xmotion

#endif /* RECT_GRID_BASE_HPP */
