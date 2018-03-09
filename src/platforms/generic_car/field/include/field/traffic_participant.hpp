/* 
 * traffic_participant.hpp
 * 
 * Created on: Mar 08, 2018 15:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_PARTICIPANT_HPP
#define TRAFFIC_PARTICIPANT_HPP

#include <functional>

#include "field/scalar_field.hpp"

namespace librav
{
template <typename DistributionT>
class TrafficParticipant : public ScalarField
{
public:
  TrafficParticipant(int64_t size_x = 0, int64_t size_y = 0) : ScalarField(size_x, size_y){};

  void SetPositionVelocity(double posx, double posy, double velx, double vely)
  {
    position_x_ = posx;
    position_y_ = posy;

    velocity_x_ = velx;
    velocity_y_ = vely;

    dist_func_.SetParameters(position_x_, position_y_, velocity_x_, velocity_y_);
    UpdateThreatDistribution();
  }

  double position_x_;
  double position_y_;
  double velocity_x_;
  double velocity_y_;

private:
  DistributionT dist_func_;
  void UpdateThreatDistribution()
  {
    for (int64_t i = 0; i < size_x_; ++i)
      for (int64_t j = 0; j < size_y_; ++j)
        SetValueAtCoordinate(i, j, dist_func_(i, j));
  }
};
}

#endif /* TRAFFIC_PARTICIPANT_HPP */
