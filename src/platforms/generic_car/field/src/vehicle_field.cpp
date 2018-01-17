/* 
 * vehicle_field.cpp
 * 
 * Created on: Nov 17, 2017 17:48
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "field/vehicle_field.hpp"

#include <cmath>

using namespace librav;

VehicleField::VehicleField(int64_t size_x, int64_t size_y) : ScalarField(size_x, size_y)
{
}

void VehicleField::SetCarPosition(int64_t x, int64_t y)
{
    com_pos_.x = x;
    com_pos_.y = y;
}

void VehicleField::UpdateDistribution()
{
    double sig = 15;
    for (int64_t i = 0; i < size_x_; ++i)
        for (int64_t j = 0; j < size_y_; ++j)
        {
            double x_err = i - com_pos_.x;
            double y_err = j - com_pos_.y;

            // double val = 20.0 * std::exp(-(x_err*x_err + y_err*y_err)/(2*sig))/(2*M_PI*sig);
            double val = std::exp(-(x_err*x_err + y_err*y_err)/(2*sig*sig))/2*M_PI*sig*sig;
            SetValueAtLocation(i,j,val);
        }
}