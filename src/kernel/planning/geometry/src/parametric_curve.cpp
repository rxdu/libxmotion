/* 
 * parametric_curve.cpp
 * 
 * Created on: Oct 20, 2018 08:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "geometry/parametric_curve.hpp"

#include <cassert>

using namespace librav;

ParametricCurve::ParametricCurve(Polyline center_polyline) : polyline_(center_polyline)
{
}

ParametricCurve::ParametricCurve(CSpline xspline, CSpline yspline, double sf) : x_spline_(xspline), y_spline_(yspline), total_length_(sf)
{
}

SimplePoint ParametricCurve::Evaluate(double s) const
{
    return SimplePoint(x_spline_.Evaluate(s), y_spline_.Evaluate(s));
}

SimplePoint ParametricCurve::Evaluate(double s, int32_t derivative) const
{
    return SimplePoint(x_spline_.Evaluate(s, derivative), y_spline_.Evaluate(s, derivative));
}

void ParametricCurve::GetTangentVector(double s, double &x, double &y) const
{
    double vx = x_spline_.Evaluate(s, 1);
    double vy = y_spline_.Evaluate(s, 1);
    double norm = std::hypot(x, y);
    x = vx / norm;
    y = vy / norm;
}

/////////////////////////////////////////////////////////////////////////////////

ParametricCurve CurveFitting::FitApproximateLengthCurve(Polyline polyline)
{
    std::vector<double> distances;
    distances.push_back(0.0);

    double accumulated = 0.0;
    for (int32_t i = 0; i < polyline.GetPointNumer() - 1; ++i)
    {
        double dist = std::hypot(polyline.GetPoint(i).x - polyline.GetPoint(i + 1).x,
                                 polyline.GetPoint(i).y - polyline.GetPoint(i + 1).y);
        accumulated += dist;
        distances.push_back(accumulated);
    }

    std::vector<CSpline::Knot> xknots, yknots;
    for (int32_t i = 0; i < polyline.GetPointNumer(); ++i)
    {
        xknots.emplace_back(distances[i], polyline.GetPoint(i).x);
        yknots.emplace_back(distances[i], polyline.GetPoint(i).y);
    }

    return ParametricCurve(CSpline(xknots), CSpline(yknots), accumulated);
}

ParametricCurve CurveFitting::FitTimedCurve(std::vector<double> x, std::vector<double> y, std::vector<double> t)
{
    assert((t.size() > 2) && (x.size() == t.size()) && (y.size() == t.size()));

    std::vector<CSpline::Knot> xknots, yknots;
    for (int32_t i = 0; i < t.size(); ++i)
    {
        xknots.emplace_back(t[i], x[i]);
        yknots.emplace_back(t[i], y[i]);
    }

    return ParametricCurve(CSpline(xknots), CSpline(yknots), t.back());
}