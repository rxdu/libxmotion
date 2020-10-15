/* 
 * rv_polygon_validity_checker.hpp
 * 
 * Created on: Dec 31, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RV_POLYGON_VALIDITY_CHECKER_HPP
#define RV_POLYGON_VALIDITY_CHECKER_HPP

#include <vector>
#include <cstdint>

#include <eigen3/Eigen/Core>

#include "geometry/polygon.hpp"
#include "sampling/space/realvector_space.hpp"

namespace ivnav
{
class RVPolygonValidityChecker
{
  public:
    using SpaceType = RealVectorSpace<2>;
    using StateType = typename SpaceType::StateType;

  public:
    RVPolygonValidityChecker() = default;
    RVPolygonValidityChecker(double step) : step_size_(step) {}

    bool operator()(StateType *state)
    {
        for (auto &poly : collisions_)
        {
            if (poly.CheckInside({state->values_[0], state->values_[1]}))
                return false;
        }
        return true;
    }

    bool operator()(StateType *sstate, StateType *dstate)
    {
        if (!(*this)(sstate) || !(*this)(dstate))
            return false;

        Eigen::Matrix<double, 2, 1> base;
        Eigen::Matrix<double, 2, 1> direction;
        base << (*sstate)[0], (*sstate)[1];
        direction << ((*dstate)[0] - (*sstate)[0]), (*dstate)[1] - (*sstate)[1];
        direction.normalize();
        double distance = std::hypot((*dstate)[0] - (*sstate)[0], (*dstate)[1] - (*sstate)[1]);
        double inc_dist = step_size_;
        while (inc_dist < distance)
        {
            Eigen::Matrix<double, 2, 1> end_state = base + direction * inc_dist;
            for (auto &poly : collisions_)
            {
                if (poly.CheckInside({end_state(0), end_state(1)}))
                    return false;
            }
            inc_dist += step_size_;
        }

        return true;
    }

    void AddCollisionPolygon(const Polygon &polygon)
    {
        collisions_.push_back(polygon);
    }

  private:
    double step_size_ = 0.5;
    std::vector<Polygon> collisions_;
};
} // namespace ivnav

#endif /* RV_POLYGON_VALIDITY_CHECKER_HPP */
