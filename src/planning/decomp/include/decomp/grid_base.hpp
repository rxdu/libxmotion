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
#include <type_traits>

#include <Eigen/Dense>

namespace librav
{

/*
 * 
 *  Grid Coordinate System:
 *              y
 *              ^
 *              |
 *		          |
 *  	          |
 *		          |
 *   ---------- o ---------> x
 *		          |
 *  	          |
 *              |
 *              | 
 *                           
 * 
 *  Internal Coordinate System:
 *		
 *		y 
 *		^         
 *		|         
 *    |         
 *  	| --------o' (origin offset)
 *		|         |
 *		|         |
 *    |         |         
 *    o------------------> x 
 * 
 */

// Reference: https://stackoverflow.com/questions/25492589/can-i-use-sfinae-to-selectively-define-a-member-variable-in-a-template-class?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
template <typename T, typename IsScalar = void>
class GridTiles;

// my favourite type :D
template <typename T>
class GridTiles<T, std::enable_if_t<std::is_floating_point<T>::value>>
{
public:
  int some_variable;
};

// not my favourite type :(
template <typename T>
class GridTiles<T, std::enable_if_t<!std::is_floating_point<T>::value>>
{
public:
  // no variable
};

template <typename T>
class derived_class : public base_class<T>
{
public:
  // do stuff
};

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

template <typename TileType, typename TileContainer>
class GridBase
{
public:
  GridBase(int64_t size_x = 0, int64_t size_y = 0);

  int64_t SizeX() const { return size_x_; };
  int64_t SizeY() const { return size_y_; };

  void ResizeGrid(int64_t x, int64_t y);
  void SetOriginCoordinate(int64_t origin_x, int64_t origin_y);

  void PrintGrid() const;

protected:
  // internal data structure for a 2D field
  int64_t size_x_;
  int64_t size_y_;
  std::vector<std::vector<TileType>> field_tiles_;

  // origin offset
  int64_t origin_offset_x_ = 0;
  int64_t origin_offset_y_ = 0;

  // operations WRT coordinate of field
  void SetTileAtGridCoordinate(int64_t x, int64_t y, TileType tile);
  TileType &GetTileAtGridCoordinate(int64_t x, int64_t y);
  // operations WRT coordinate of internal data structure directly
  void SetTileAtRawCoordinate(int64_t x, int64_t y, TileType tile);
  TileType &GetTileAtRawCoordinate(int64_t x, int64_t y);
  // convertion between two coordinates
  GridCoordinate ConvertToRawCoordinate(int64_t x, int64_t y);
  GridCoordinate ConvertToGridCoordinate(int64_t x, int64_t y);
};
}

#include "details/field_base_impl.hpp"

#endif /* GRID_BASE_HPP */
