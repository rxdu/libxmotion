/* 
 * cspline.cpp
 * 
 * Created on: Oct 13, 2018 11:08
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/cspline.hpp"

#include <memory>
#include <algorithm>

using namespace librav;

CSpline::CSpline()
{
    accel_ = gsl_interp_accel_alloc();
}

CSpline::CSpline(const std::vector<Knot> &knots) : knots_(knots)
{
    accel_ = gsl_interp_accel_alloc();
    spline_ = gsl_spline_alloc(gsl_interp_cspline, knots.size());

    double x[knots.size()];
    double y[knots.size()];
    for (int i = 0; i < knots.size(); ++i)
    {
        x[i] = knots[i].x;
        y[i] = knots[i].y;
    }
    gsl_spline_init(spline_, x, y, knots.size());
}

CSpline::~CSpline()
{
    if (spline_ != nullptr)
        gsl_spline_free(spline_);
    if (accel_ != nullptr)
        gsl_interp_accel_free(accel_);
}

CSpline::CSpline(const CSpline &other)
{
    accel_ = gsl_interp_accel_alloc();
    spline_ = gsl_spline_alloc(gsl_interp_cspline, other.knots_.size());

    double x[other.knots_.size()];
    double y[other.knots_.size()];
    for (int i = 0; i < other.knots_.size(); ++i)
    {
        x[i] = other.knots_[i].x;
        y[i] = other.knots_[i].y;
    }
    gsl_spline_init(spline_, x, y, other.knots_.size());
}

CSpline &CSpline::operator=(const CSpline &other)
{
    CSpline temp = other;
    std::swap(*this, temp);
    return *this;
}

CSpline::CSpline(CSpline &&other)
{
    std::swap(this->accel_, other.accel_);
    std::swap(this->spline_, other.spline_);
}

CSpline &CSpline::operator=(CSpline &&other)
{
    std::swap(this->accel_, other.accel_);
    std::swap(this->spline_, other.spline_);
    return *this;
}

double CSpline::Evaluate(double x)
{
    return gsl_spline_eval(spline_, x, accel_);
}

void CSpline::Interpolate(const std::vector<Knot> &knots)
{
    // destroy old spline if one already exists
    if (spline_ != nullptr)
        gsl_spline_free(spline_);

    knots_ = knots;
    spline_ = gsl_spline_alloc(gsl_interp_cspline, knots.size());

    double x[knots.size()];
    double y[knots.size()];
    for (int i = 0; i < knots.size(); ++i)
    {
        x[i] = knots[i].x;
        y[i] = knots[i].y;
    }
    gsl_spline_init(spline_, x, y, knots.size());
}