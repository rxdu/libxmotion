/* 
 * cspline.hpp
 * 
 * Created on: Oct 13, 2018 11:02
 * Description: cubic spline wrapper around GSL
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CSPLINE_HPP
#define CSPLINE_HPP

#include <cstdint>
#include <vector>

#include <gsl/gsl_errno.h>
#include <gsl/gsl_spline.h>

namespace ivnav
{
class CSpline
{
  public:
    struct Knot
    {
        Knot(double _x = 0, double _y = 0) : x(_x), y(_y) {}
        double x;
        double y;
    };

  public:
    CSpline();
    explicit CSpline(const std::vector<Knot> &knots);
    ~CSpline();

    CSpline(const CSpline &other);
    CSpline &operator=(const CSpline &other);
    CSpline(CSpline &&other);
    CSpline &operator=(CSpline &&other);

    double Evaluate(double x) const;
    double Evaluate(double x, int32_t derivative) const;

    void Interpolate(const std::vector<Knot> &knots);

    std::vector<Knot> GetAllKnots() const { return knots_; }

  private:
    gsl_interp_accel *accel_ = nullptr;
    gsl_spline *spline_ = nullptr;

    std::vector<Knot> knots_;
};
} // namespace ivnav

#endif /* CSPLINE_HPP */
