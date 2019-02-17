/* 
 * radial_basis.hpp
 * 
 * Created on: Feb 17, 2019 09:30
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef RADIAL_BASIS_HPP
#define RADIAL_BASIS_HPP

namespace librav
{
struct RadialBasis
{
    virtual double operator()(double x, double y) = 0;
    virtual void PrintInfo() = 0;
};
} // namespace librav

#endif /* RADIAL_BASIS_HPP */
