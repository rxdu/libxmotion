/* 
 * field_basis.hpp
 * 
 * Created on: Dec 14, 2018 00:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef FIELD_BASIS_HPP
#define FIELD_BASIS_HPP

#include <eigen3/Eigen/Core>

namespace librav
{
class FieldBasis
{
  public:
    FieldBasis() = default;
    FieldBasis(double px, double py);

    double operator()(double x, double y);
    void PrintInfo();

    static constexpr double sigma_n = 1;

  private:
    double cx_ = 0.0;
    double cy_ = 0.0;
};
} // namespace librav

#endif /* FIELD_BASIS_HPP */
