/* 
 * polynomial.hpp
 * 
 * Created on: Oct 22, 2018 00:19
 * Description: univariate polynomial wrapper around GSL
 *              API follows the polynomial class in repo [1]
 * Note: a multivariate polynomial wrapper could be created around CGAL
 * 
 * Reference:
 * [1] https://github.com/ethz-asl/mav_trajectory_generation
 * [2] https://www.gnu.org/software/gsl/doc/html/poly.html
 * [3] https://doc.cgal.org/latest/Polynomial/index.html
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP

#include <cstdint>

#include <eigen3/Eigen/Dense>
#include <gsl/gsl_poly.h>

namespace ivnav
{
// (N-1)-degree polynomial with N coefficients
// Polynomial coefficients are stored with increasing powers,
// i.e. P(t) = c_0 + c_1*t ... c_{N-1} * t^{N-1}
//      coeffs = [c0 c1 ... c_{N-1}]
template <int32_t N>
class Polynomial
{
  public:
    Polynomial() : coefficients_(N) { coefficients_.setZero(); }
    Polynomial(const Eigen::VectorXd &coeffs)
        : coefficients_(coeffs)
    {
        assert(coeffs.size() == N);
        SetCoefficients(coefficients_);
    }

    inline bool operator==(const Polynomial &rhs) const
    {
        return coefficients_ == rhs.coefficients_;
    }
    inline bool operator!=(const Polynomial &rhs) const
    {
        return !operator==(rhs);
    }
    inline Polynomial<N> operator+(const Polynomial<N> &rhs) const
    {
        return Polynomial<N>(coefficients_ + rhs.coefficients_);
    }
    inline Polynomial<N> &operator+=(const Polynomial<N> &rhs)
    {
        this->coefficients_ += rhs.coefficients_;
        this->SetCoefficients(this->coefficients_);
        return *this;
    }
    inline Polynomial<N> operator*(const double &rhs) const
    {
        return Polynomial<N>(coefficients_ * rhs);
    }

    int GetCoefficientNum() const { return N; }
    int GetDegreeNum() const { return (N - 1); }

    void SetCoefficients(const Eigen::VectorXd &coeffs)
    {
        assert(N == coeffs.size());

        coefficients_ = coeffs;
        for (int32_t i = 0; i < N; ++i)
            coeff_vals_[i] = coefficients_(i);
    }

    double Evaluate(double t) const
    {
        return gsl_poly_eval(coeff_vals_, N, t);
    }

    double Evaluate(double t, int derivative) const
    {
        if (derivative >= N)
            return 0.0;

        double res[N];
        gsl_poly_eval_derivs(coeff_vals_, N, t, res, N);
        return res[derivative];
    }

  private:
    Eigen::VectorXd coefficients_;
    double coeff_vals_[N] = {0};
};
} // namespace ivnav

#endif /* POLYNOMIAL_HPP */
