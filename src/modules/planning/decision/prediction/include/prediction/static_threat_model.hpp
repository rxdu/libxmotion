/* 
 * static_threat_model.hpp
 * 
 * Created on: Nov 05, 2018 11:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef STATIC_THREAT_MODEL_HPP
#define STATIC_THREAT_MODEL_HPP

#include "navtypes/navtypes.hpp"

namespace rnav
{
class VehicleStaticThreat
{
  public:
    VehicleStaticThreat() = default;
    VehicleStaticThreat(Pose2d _pose, double _probability) : pose(_pose), probability(_probability){};

    // TODO added for compilation, not sure if behavior is correct yet
    VehicleStaticThreat& operator=(const VehicleStaticThreat& other) {
        pose = other.pose;
        probability = other.probability;
        return *this;
    }

    void SetParameters(Pose2d _pose, double _probability)
    {
        pose = _pose;
        probability = _probability;
    }

    double operator()(double x, double y)
    {
        // transform position to align with velocity vector
        double theta = pose.theta;
        double x_err = (x - pose.position.x) * std::cos(theta) + (y - pose.position.y) * std::sin(theta);
        double y_err = -(x - pose.position.x) * std::sin(theta) + (y - pose.position.y) * std::cos(theta);

        return std::exp(-x_err * x_err / sigma_f_hsquare - y_err * y_err / sigma_s_hsquare);
    }

    Pose2d pose;
    double probability;

  private:
    // Note: hard-coded here, could be defined on the fly in
    //      future according to vehicle types
    const double sigma_f = (4.8 / 2) * (4.8 / 2);
    const double sigma_s = (1.8 * 4 / 5) * (1.8 * 4 / 5);

    // intermediate constants for calculation
    const double sigma_f_hsquare = sigma_f * sigma_f / 2.0;
    const double sigma_s_hsquare = sigma_s * sigma_s / 2.0;
};
} // namespace rnav

#endif /* STATIC_THREAT_MODEL_HPP */
