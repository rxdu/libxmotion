/* 
 * rv_straight_steer.hpp
 * 
 * Created on: Dec 31, 2018 02:14
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef RV_STRAIGHT_STEER_HPP
#define RV_STRAIGHT_STEER_HPP

#include "sampling/space/realvector_space.hpp"

#include <eigen3/Eigen/Core>

namespace librav
{
template <int32_t N>
class RVStraightSteer
{
  public:
    using SpaceType = RealVectorSpace<N>;
    using StateType = typename SpaceType::StateType;

  public:
    RVStraightSteer(RealVectorSpace<N> *s, double dist) : space_(s), max_dist_(dist){};

    SpaceType *space_;
    double max_dist_;

    StateType *operator()(StateType *start, StateType *goal)
    {
        double distance = space_->EvaluateDistance(start, goal);
        if (distance < max_dist_)
            return goal;

        Eigen::Matrix<double, N, 1> base;
        Eigen::Matrix<double, N, 1> direction;
        for (int i = 0; i < N; ++i)
        {
            base[i] = (*start)[i];
            direction(i) = (*goal)[i] - (*start)[i];
        }
        direction.normalize();
        Eigen::Matrix<double, N, 1> end_state = base + direction * max_dist_;

        return space_->CreateState(end_state);
    }
};
} // namespace librav

#endif /* RV_STRAIGHT_STEER_HPP */
