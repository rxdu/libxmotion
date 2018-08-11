/* 
 * scalar_field.cpp
 * 
 * Created on: Nov 17, 2017 17:58
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "threat_field/scalar_field.hpp"

#include <iomanip>

using namespace librav;

ScalarField::ScalarField(int64_t size_x, int64_t size_y) : DenseGrid(size_x, size_y)
{
}

ScalarFieldMatrix ScalarField::GenerateFieldMatrix(double x_start, double x_step, double y_start, double y_step, bool normalize_z)
{
    field_matrix_.x.resize(size_x_);
    field_matrix_.y.resize(size_y_);
    field_matrix_.z.resize(size_y_, size_x_);

    for (int64_t i = 0; i < size_x_; ++i)
        field_matrix_.x(i) = x_start + i * x_step;
    for (int64_t j = 0; j < size_y_; ++j)
        field_matrix_.y(j) = y_start + j * y_step;

    for (int64_t j = 0; j < size_y_; ++j)
        for (int64_t i = 0; i < size_x_; ++i)
            field_matrix_.z(j, i) = GetTileAtRawCoordinate(i, j);

    if (normalize_z)
        field_matrix_.z = field_matrix_.z / field_matrix_.z.maxCoeff() * 1.0;

    return field_matrix_;
}

void ScalarField::PrintField(bool pretty) const
{
    if (pretty)
    {
        for (int64_t y = 0; y < size_y_; ++y)
        {
            for (int64_t x = 0; x < size_x_; ++x)
            {
                std::cout << std::setw(6) << GetTileAtRawCoordinate(x,y);
            }
            std::cout << std::endl;
        }
    }
    else
    {
        for (int64_t y = 0; y < size_y_; ++y)
            for (int64_t x = 0; x < size_x_; ++x)
                std::cout << "(" << x << " , " << y << ") : " << GetTileAtRawCoordinate(x,y) << std::endl;
    }
}
