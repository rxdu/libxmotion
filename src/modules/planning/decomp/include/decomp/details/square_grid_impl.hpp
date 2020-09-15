/* 
 * square_grid_impl.hpp
 * 
 * Created on: Apr 08, 2018 18:01
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SQUARE_GRID_IMPL_HPP
#define SQUARE_GRID_IMPL_HPP

namespace librav
{
template <typename T>
SquareGridBase<T>::SquareGridBase(int32_t size_x, int32_t size_y, double cell_size) : RectGridBase<SquareCellBase<T> *>(size_x, size_y),
                                                                                      cell_size_(cell_size)
{
    assert((size_x > 0 && size_y > 0));

    for (int32_t y = 0; y < size_y; y++)
        for (int32_t x = 0; x < size_x; x++)
        {
            SquareCellBase<T> *new_cell = new SquareCellBase<T>(x, y, IndexToID(x, y));
            new_cell->UpdateGeometry(cell_size);
            RectGridBase<SquareCellBase<T> *>::SetTileAtRawCoordinate(x, y, new_cell);
        }
}

template <typename T>
SquareGridBase<T>::SquareGridBase(const Eigen::MatrixXd &matrix, int32_t side_length, double cell_size) : RectGridBase<SquareCellBase<T> *>(0, 0),
                                                                                                          cell_size_(cell_size)
{
    int32_t grid_size_x, grid_size_y;
    Eigen::MatrixXd occupancy_matrix;

    if (side_length == 1)
    {
        grid_size_x = matrix.cols();
        grid_size_y = matrix.rows();
        occupancy_matrix = matrix;
    }
    else
    {
        bool shrink_x = false;
        bool shrink_y = false;

        // determine size of grid
        int32_t center_x = matrix.cols() / 2;
        int32_t center_y = matrix.rows() / 2;

        if (matrix.cols() % side_length != 0)
            shrink_x = true;
        if (matrix.rows() % side_length != 0)
            shrink_y = true;
        grid_size_x = (center_x / side_length) * 2;
        grid_size_y = (center_y / side_length) * 2;

        if (shrink_x || shrink_y)
        {
            occupancy_matrix = Eigen::MatrixXd::Ones(grid_size_y * side_length, grid_size_x * side_length);

            int32_t x_start = 0, y_start = 0;
            if (shrink_x)
                x_start = center_x % side_length;
            if (shrink_y)
                y_start = center_y % side_length;

            occupancy_matrix = matrix.block(y_start, x_start, occupancy_matrix.rows(), occupancy_matrix.cols());
        }
        else
        {
            occupancy_matrix = matrix;
        }
    }

    // create new grid
    RectGridBase<SquareCellBase<T> *>::ResizeGrid(grid_size_x, grid_size_y);
    for (int32_t y = 0; y < grid_size_y; y++)
        for (int32_t x = 0; x < grid_size_x; x++)
        {
            SquareCellBase<T> *new_cell = new SquareCellBase<T>(x, y, IndexToID(x, y));
            new_cell->UpdateGeometry(cell_size);
            RectGridBase<SquareCellBase<T> *>::SetTileAtRawCoordinate(x, y, new_cell);
        }

    // determine occupancy of grid
    int32_t half_size_x = grid_size_x / 2;
    int32_t half_size_y = grid_size_y / 2;
    for (int64_t x = 0; x < RectGridBase<SquareCellBase<T> *>::SizeX(); ++x)
        for (int64_t y = 0; y < RectGridBase<SquareCellBase<T> *>::SizeY(); ++y)
        {
            bool occupied = false;
            int32_t xmin = x * side_length;
            int32_t xmax = xmin + side_length;
            int32_t ymin = y * side_length;
            int32_t ymax = ymin + side_length;
            for (int i = xmin; i < xmax; ++i)
            {
                for (int j = ymin; j < ymax; ++j)
                {
                    if (occupancy_matrix(j, i) != 0)
                    {
                        SetCellLabel(x, y, SquareCellLabel::OCCUPIED);
                        occupied = true;
                        break;
                    }
                }
                if (occupied)
                    break;
            }
        }
}

template <typename T>
SquareGridBase<T>::~SquareGridBase()
{
    for (int32_t y = 0; y < RectGridBase<SquareCellBase<T> *>::size_y_; y++)
        for (int32_t x = 0; x < RectGridBase<SquareCellBase<T> *>::size_x_; x++)
            delete RectGridBase<SquareCellBase<T> *>::GetTileAtRawCoordinate(x, y);
}

template <typename T>
SquareGridBase<T>::SquareGridBase(const SquareGridBase<T> &other) : RectGridBase<SquareCellBase<T> *>(other.SizeX(), other.SizeY()),
                                                                    cell_size_(other.cell_size_)
{
    for (auto row : other.grid_tiles_)
        for (auto cell : row)
            RectGridBase<SquareCellBase<T> *>::SetTileAtRawCoordinate(cell->x, cell->y, new SquareCellBase<T>(*cell));
}

template <typename T>
SquareGridBase<T> &SquareGridBase<T>::operator=(const SquareGridBase<T> &other)
{
    SquareGridBase<T> tmp(other);
    *this = std::move(tmp);
    return *this;
}

template <typename T>
SquareGridBase<T>::SquareGridBase(SquareGridBase<T> &&other) : RectGridBase<SquareCellBase<T> *>(other.SizeX(), other.SizeY()),
                                                               cell_size_(other.cell_size_)
{
    for (auto row : other.grid_tiles_)
        for (auto cell : row)
            RectGridBase<SquareCellBase<T> *>::SetTileAtRawCoordinate(cell->x, cell->y, cell);
}

template <typename T>
SquareGridBase<T> &SquareGridBase<T>::operator=(SquareGridBase<T> &&other)
{
    RectGridBase<SquareCellBase<T> *>::ResizeGridTiles(other.size_x, other.size_y);
    cell_size_ = std::move(other.cell_size);
    for (auto row : other.grid_tiles_)
        for (auto cell : row)
            RectGridBase<SquareCellBase<T> *>::SetTileAtRawCoordinate(cell->x, cell->y, cell);

    return *this;
}

template <typename T>
SquareCellBase<T> *SquareGridBase<T>::GetCell(int64_t id)
{
    auto coordinate = IDToIndex(id);
    return RectGridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(coordinate.GetX(), coordinate.GetY());
}

template <typename T>
SquareCellBase<T> *SquareGridBase<T>::GetCell(int32_t x, int32_t y)
{
    return RectGridBase<SquareCellBase<T> *>::GetTileAtGridCoordinate(x, y);
}

template <typename T>
std::vector<SquareCellBase<T> *> SquareGridBase<T>::GetNeighbours(int32_t x, int32_t y, bool allow_diag)
{
    std::vector<RectGridIndex> candidates;
    if (allow_diag)
    {
        for (int32_t xi = x - 1; xi <= x + 1; ++xi)
            for (int32_t yi = y - 1; yi <= y + 1; ++yi)
            {
                if (xi == x && yi == y)
                    continue;
                candidates.emplace_back(xi, yi);
            }
    }
    else
    {
        candidates.emplace_back(x, y + 1);
        candidates.emplace_back(x, y - 1);
        candidates.emplace_back(x + 1, y);
        candidates.emplace_back(x - 1, y);
    }

    std::vector<SquareCellBase<T> *> neighbours;
    for (auto &can : candidates)
    {
        int32_t xi = can.GetX();
        int32_t yi = can.GetY();
        if (xi >= 0 && xi < RectGridBase<SquareCellBase<T> *>::size_x_ && yi >= 0 && yi < RectGridBase<SquareCellBase<T> *>::size_y_)
            neighbours.push_back(RectGridBase<SquareCellBase<T> *>::GetTileAtRawCoordinate(xi, yi));
    }

    return neighbours;
}

template <typename T>
std::vector<SquareCellBase<T> *> SquareGridBase<T>::GetNeighbours(int64_t id, bool allow_diag)
{
    auto index = IDToIndex(id);
    return GetNeighbours(index.GetX(), index.GetY(), allow_diag);
}
} // namespace librav

#endif /* SQUARE_GRID_IMPL_HPP */
