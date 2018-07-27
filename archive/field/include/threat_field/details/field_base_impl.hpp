/* 
 * field_impl.hpp
 * 
 * Created on: Nov 17, 2017 15:12
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef FIELD_IMPL_HPP
#define FIELD_IMPL_HPP

#include <cassert>
#include <iostream>

namespace librav
{

template <typename T>
FieldBase<T>::FieldBase(int64_t size_x, int64_t size_y) : size_x_(size_x),
                                                          size_y_(size_y)
{
    field_tiles_.resize(size_x);
    for (auto &tile_array : field_tiles_)
    {
        for (int64_t i = 0; i < size_y; ++i)
            tile_array.push_back(T{});
    }
}

template <typename T>
void FieldBase<T>::SetOriginCoordinate(int64_t origin_x, int64_t origin_y)
{
    origin_offset_x_ = origin_x;
    origin_offset_y_ = origin_y;
}

template <typename T>
void FieldBase<T>::ResizeField(int64_t x, int64_t y)
{
    assert(x > origin_offset_x_ && y > origin_offset_y_);

    field_tiles_.resize(x);
    for (auto &tile_array : field_tiles_)
    {
        tile_array.resize(y);
    }
}

template <typename T>
void FieldBase<T>::SetTileAtFieldCoordinate(int64_t x, int64_t y, T tile)
{
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    assert((internal_coordinate.GetX() >= 0) && (internal_coordinate.GetX() < size_x_) && (internal_coordinate.GetY() >= 0) && (internal_coordinate.GetY() < size_y_));

    field_tiles_[internal_coordinate.GetX()][internal_coordinate.GetY()] = tile;
}

template <typename T>
T &FieldBase<T>::GetTileAtFieldCoordinate(int64_t x, int64_t y)
{
    auto internal_coordinate = ConvertToRawCoordinate(x, y);
    assert((internal_coordinate.GetX() >= 0) && (internal_coordinate.GetX() < size_x_) && (internal_coordinate.GetY() >= 0) && (internal_coordinate.GetY() < size_y_));

    return field_tiles_[internal_coordinate.GetX()][internal_coordinate.GetY()];
}

template <typename T>
void FieldBase<T>::SetTileAtRawCoordinate(int64_t x, int64_t y, T tile)
{
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    field_tiles_[x][y] = tile;
}

template <typename T>
T &FieldBase<T>::GetTileAtRawCoordinate(int64_t x, int64_t y)
{
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    return field_tiles_[x][y];
}

template <typename T>
FieldCoordinate FieldBase<T>::ConvertToRawCoordinate(int64_t x, int64_t y)
{
    assert((x > -size_x_) && (x < size_x_) && (y > -size_y_) && (y < size_y_));

    return FieldCoordinate(x + origin_offset_x_, y + origin_offset_y_);
}

template <typename T>
FieldCoordinate FieldBase<T>::ConvertToFieldCoordinate(int64_t x, int64_t y)
{
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    return FieldCoordinate(x - origin_offset_x_, y - origin_offset_y_);
}
}
#endif /* FIELD_IMPL_HPP */
