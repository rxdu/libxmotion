/*
 * bicycle_velocity_model.hpp
 *
 * Created on: Aug 24, 2023 22:28
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef BICYCLE_MODEL_HPP
#define BICYCLE_MODEL_HPP

#include <cmath>
#include <vector>

namespace xmotion {
// Reference:
//  [1] B. Paden, M. Cap, S. Z. Yong, D. Yershov, and E. Frazzoli, “A Survey of
//      Motion Planning and Control Techniques for Self-driving Urban Vehicles,”
//      Apr. 2016.
//  [2] J. Kong, M. Pfeiffer, G. Schildbach, and F. Borrelli, “Kinematic and
//  dynamic
//      vehicle models for autonomous driving control design,” in 2015 IEEE
//      Intelligent Vehicles Symposium (IV), Jun. 2015, pp. 1094–1099.
//  [3] R. Rajamani, Vehicle Dynamics and Control. Springer Science & Business
//      Media, Dec. 2011.

/*
 * Bicycle kinematics model (rear wheel):
 *  dot_x = v(t) * cos(theta(t))
 *  dot_y = v(t) * sin(theta(t))
 *  dot_theta = v(t)/L * tan(delta(t))
 * State: (x, y, v, theta)
 * Control input: (v, delta) - speed, steering angle of front wheel
 */
class BicycleKinematics {
 public:
  struct Command {
    Command() = default;
    Command(double vel, double ster) : v(vel), delta(ster) {}

    double v = 0.0;
    double delta = 0.0;
  };

  using control_type = Command;
  using state_type = std::vector<double>;

 public:
  BicycleKinematics(control_type u) : u_(u){};

  // x1 = x, x2 = y, x3 = v, x4 = theta
  void operator()(const state_type &x, state_type &xd, double) {
    xd[0] = x[2] * std::cos(x[3]);
    xd[1] = x[2] * std::sin(x[3]);
    xd[3] = x[2] / L * std::tan(u_.delta);
  }

 private:
  control_type u_ = {0.0, 0.0};
  static constexpr double L = 2.4;
};
}  // namespace xmotion

#endif /* BICYCLE_MODEL_HPP */
