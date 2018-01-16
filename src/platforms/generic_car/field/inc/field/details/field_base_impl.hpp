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
FieldBase<T>::FieldBase() : size_x_(0),
                            size_y_(0)
{
}

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
void FieldBase<T>::ResizeField(int64_t x, int64_t y)
{
    field_tiles_.resize(x);
    for (auto &tile_array : field_tiles_)
    {
        tile_array.resize(y);
    }
}

template <typename T>
void FieldBase<T>::SetTileAtLocation(int64_t x, int64_t y, T tile)
{
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));

    field_tiles_[x][y] = tile;
}

template <typename T>
T& FieldBase<T>::GetTileAtLocation(int64_t x, int64_t y)
{
    assert((x >= 0) && (x < size_x_) && (y >= 0) && (y < size_y_));
    
    return field_tiles_[x][y];
}

template <typename T>
void FieldBase<T>::PrintField() const
{
    for (int64_t x = 0; x < field_tiles_.size(); ++x)
        for (int64_t y = 0; y < field_tiles_[x].size(); ++y)
            std::cout << "(" << x << " , " << y << ") : " << field_tiles_[x][y] << std::endl;
}
}

#endif /* FIELD_IMPL_HPP */
