/* 
 * scalar_field.cpp
 * 
 * Created on: Nov 17, 2017 17:58
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "field/scalar_field.hpp"

#include <iomanip>

using namespace librav;

ScalarField::ScalarField(int64_t size_x, int64_t size_y) : FieldBase<double>(size_x, size_y)
{
}

void ScalarField::SetValueAtCoordinate(int64_t x, int64_t y, double val)
{
    SetTileAtFieldCoordinate(x, y, val);
}

double ScalarField::GetValueAtCoordinate(int64_t x, int64_t y)
{
    double val = GetTileAtFieldCoordinate(x, y);
    return val;
}

librav_lcm_msgs::ScalarField_t ScalarField::GenerateScalarFieldMsg()
{
    librav_lcm_msgs::ScalarField_t field_msg;
    field_msg.size_x = size_x_;
    field_msg.size_y = size_y_;
    field_msg.value.resize(size_x_);
    for (int64_t i = 0; i < size_x_; ++i)
    {
        field_msg.value[i].resize(size_y_);
        for (int64_t j = 0; j < size_y_; ++j)
        {
            field_msg.value[i][j] = GetTileAtFieldCoordinate(i, j);
        }
    }

    return field_msg;
}

ScalarFieldMatrix ScalarField::GenerateFieldMatrix(double x_start, double x_step, double y_start, double y_step, bool normalize_z)
{
    field_matrix_.x.resize(size_x_);
    field_matrix_.y.resize(size_y_);
    field_matrix_.z.resize(size_x_, size_y_);

    for (int64_t i = 0; i < size_x_; ++i)
        field_matrix_.x(i) = x_start + i * x_step;
    for (int64_t j = 0; j < size_y_; ++j)
        field_matrix_.y(j) = y_start + j * y_step;

    for (int64_t j = 0; j < size_y_; ++j)
        for (int64_t i = 0; i < size_x_; ++i)
            field_matrix_.z(i, j) = GetTileAtFieldCoordinate(i, j);

    if (normalize_z)
        field_matrix_.z = field_matrix_.z / field_matrix_.z.maxCoeff() * 1.0;

    return field_matrix_;
}

void ScalarField::PrintField(bool pretty) const
{
    if (pretty)
    {
        for (int64_t y = size_y_ - 1; y >= 0; --y)
        {
            for (int64_t x = 0; x < size_x_; ++x)
            {
                std::cout << std::setw(6) << field_tiles_[x][y];
            }
            std::cout << std::endl;
        }
    }
    else
    {
        for (int64_t x = 0; x < field_tiles_.size(); ++x)
            for (int64_t y = 0; y < field_tiles_[x].size(); ++y)
                std::cout << "(" << x << " , " << y << ") : " << field_tiles_[x][y] << std::endl;
    }
}
