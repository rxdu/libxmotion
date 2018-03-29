/* 
 * grid_base_tiles.hpp
 * 
 * Created on: Mar 29, 2018 11:19
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GRID_BASE_TILES_HPP
#define GRID_BASE_TILES_HPP

#include <cstdint>
#include <vector>

namespace librav
{
// Reference: https://stackoverflow.com/questions/25492589/can-i-use-sfinae-to-selectively-define-a-member-variable-in-a-template-class?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
template <typename T, typename IsScalar = void>
class GridTiles;

// my favourite type :D
template <typename T>
class GridTiles<T, std::enable_if_t<std::is_floating_point<T>::value || std::is_integral<T>::value>>
{
  protected:
    Eigen::MatrixXd grid_tiles_;

    void ResizeGrid(int64_t x, int64_t y)
    {
        grid_tiles_.conservativeResize(y, x);
    }

    T GetTileAtCoordinate(int64_t x, int64_t y) const
    {
        return grid_tiles_(y, x);
    }

    T &GetTileRefAtCoordinate(int64_t x, int64_t y)
    {
        return grid_tiles_(y, x);
    }

    void SetTileAtCoordinate(int64_t x, int64_t y, T tile)
    {
        grid_tiles_(y, x) = tile;
    }
};

// not my favourite type :(
template <typename T>
class GridTiles<T, std::enable_if_t<!std::is_floating_point<T>::value && !std::is_integral<T>::value>>
{
  protected:
    // no variable
    std::vector<std::vector<T>> grid_tiles_;

    void ResizeGrid(int64_t x, int64_t y)
    {
        grid_tiles_.resize(y);
        for (auto &tile_array : grid_tiles_)
            for (int64_t i = 0; i < y; ++i)
                tile_array.resize(x);
    }

    T GetTileAtCoordinate(int64_t x, int64_t y) const
    {
        return grid_tiles_[y][x];
    }

    template<typename T2 = T, typename std::enable_if<!std::is_pointer<T2>::value>::type * = nullptr>
    T &GetTileRefAtCoordinate(int64_t x, int64_t y)
    {
        return grid_tiles_[y][x];
    }

    void SetTileAtCoordinate(int64_t x, int64_t y, T tile)
    {
        grid_tiles_[y][x] = tile;
    }
};
}

#endif /* GRID_BASE_TILES_HPP */
