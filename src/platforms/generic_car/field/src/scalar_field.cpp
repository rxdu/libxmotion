/* 
 * scalar_field.cpp
 * 
 * Created on: Nov 17, 2017 17:58
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "field/scalar_field.h"

using namespace librav;

ScalarField::ScalarField(int64_t size_x, int64_t size_y) : FieldBase<double>(size_x, size_y)
{
}

void ScalarField::SetValueAtLocation(int64_t x, int64_t y, double val)
{
    SetTileAtLocation(x, y, val);
}

double ScalarField::GetValueAtLocation(int64_t x, int64_t y)
{
    double val = GetTileAtLocation(x, y);
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
            field_msg.value[i][j] = GetTileAtLocation(i, j);
        }
    }

    return field_msg;
}

FieldMatrix ScalarField::GenerateFieldMatrix(double x_start, double x_step, double y_start, double y_step)
{
    FieldMatrix mat;
    mat.x.resize(size_x_);
    mat.y.resize(size_y_);
    mat.z.resize(size_x_, size_y_);

    for (int64_t i = 0; i < size_x_; ++i)
        mat.x(i) = x_start + i * x_step;
    for (int64_t j = 0; j < size_y_; ++j)
        mat.y(j) = y_start + j * y_step;

    for (int64_t j = 0; j < size_y_; ++j)
        for (int64_t i = 0; i < size_x_; ++i)
            mat.z(i, j) = GetTileAtLocation(i, j);

    return mat;
}