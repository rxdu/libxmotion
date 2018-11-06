/* 
 * static_threat.hpp
 * 
 * Created on: Nov 05, 2018 11:16
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef STATIC_THREAT_HPP
#define STATIC_THREAT_HPP

#include "common/librav_types.hpp"

namespace librav
{
class VehicleStaticThreat
{
  public:
    VehicleStaticThreat() = default;
    VehicleStaticThreat(Pose2d _pose, double _probability) : pose(_pose), probability(_probability)
    {
        footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
    };

    void SetParameters(Pose2d _pose, double _probability)
    {
        pose = _pose;
        probability = _probability;

        footprint.TransformRT(pose.position.x, pose.position.y, pose.theta);
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
    VehicleFP footprint;

  private:
    // Note: hard-coded here, could be defined on the fly in
    //      future according to vehicle types
    static constexpr double sigma_f = (4.8 / 2) * (4.8 / 2);
    static constexpr double sigma_s = (1.8 * 4 / 5) * (1.8 * 4 / 5);

    // intermediate constants for calculation
    static constexpr double sigma_f_hsquare = sigma_f * sigma_f / 2.0;
    static constexpr double sigma_s_hsquare = sigma_s * sigma_s / 2.0;
};
} // namespace librav

#endif /* STATIC_THREAT_HPP */
