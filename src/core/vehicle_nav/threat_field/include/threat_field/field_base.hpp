/* 
 * field_base.hpp
 * 
 * Created on: Nov 17, 2017 13:25
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef FIELD_HPP
#define FIELD_HPP

#include <cstdint>
#include <vector>
#include <tuple>

#include <Eigen/Dense>

namespace librav
{

/*
 * 
 *  Field Coordinate System:
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

class FieldCoordinate
{
public:
  FieldCoordinate(int64_t x = 0, int64_t y = 0)
  {
    coordinate_x_ = x;
    coordinate_y_ = y;
  }
  ~FieldCoordinate() = default;

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

template <typename T>
class FieldBase
{
public:
  FieldBase(int64_t size_x = 0, int64_t size_y = 0);

  int64_t SizeX() const { return size_x_; };
  int64_t SizeY() const { return size_y_; };

  void ResizeField(int64_t x, int64_t y);
  void SetOriginCoordinate(int64_t origin_x, int64_t origin_y);

  void PrintField() const;

protected:
  // internal data structure for a 2D field
  int64_t size_x_;
  int64_t size_y_;
  std::vector<std::vector<T>> field_tiles_;

  // origin offset
  int64_t origin_offset_x_ = 0;
  int64_t origin_offset_y_ = 0;

  // operations WRT coordinate of field
  void SetTileAtFieldCoordinate(int64_t x, int64_t y, T tile);
  T &GetTileAtFieldCoordinate(int64_t x, int64_t y);
  // operations WRT coordinate of internal data structure directly
  void SetTileAtRawCoordinate(int64_t x, int64_t y, T tile);
  T &GetTileAtRawCoordinate(int64_t x, int64_t y);
  // convertion between two coordinates
  FieldCoordinate ConvertToRawCoordinate(int64_t x, int64_t y);
  FieldCoordinate ConvertToFieldCoordinate(int64_t x, int64_t y);
};
}

#include "details/field_base_impl.hpp"

#endif /* FIELD_HPP */
