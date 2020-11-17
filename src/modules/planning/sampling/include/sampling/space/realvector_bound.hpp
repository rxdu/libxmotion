/* 
 * realvector_bound.hpp
 * 
 * Created on: Dec 29, 2018 07:07
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef REALVECTOR_BOUND_HPP
#define REALVECTOR_BOUND_HPP

#include <cassert>

namespace rnav
{
class RealVectorBound
{
  public:
    RealVectorBound() = default;
    RealVectorBound(double min, double max) : lower_bound_(min), upper_bound_(max)
    {
        assert(lower_bound_ < upper_bound_);
    }

    void SetBound(double low, double high)
    {
        lower_bound_ = low;
        upper_bound_ = high;
        assert(lower_bound_ < upper_bound_);
    }

    void SetLow(double value)
    {
        lower_bound_ = value;
        assert(lower_bound_ < upper_bound_);
    };

    void SetHigh(double value)
    {
        upper_bound_ = value;
        assert(lower_bound_ < upper_bound_);
    };

    double GetLow() const { return lower_bound_; }
    double GetHigh() const { return upper_bound_; }
    double GetDifference() const { return upper_bound_ - lower_bound_; };

  private:
    double lower_bound_ = 0.0;
    double upper_bound_ = 1.0;
};
} // namespace rnav

#endif /* REALVECTOR_BOUND_HPP */
