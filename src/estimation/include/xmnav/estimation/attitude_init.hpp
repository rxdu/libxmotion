/*
 * @file attitude_init.hpp
 * @brief Attitude initialization from vector observations.
 *
 * Deterministic bootstrap for the MEKF filters: instead of starting at
 * identity and waiting for the covariance to pull the estimate in, seed
 * Params::init_quaternion from the first sensor sample.
 *
 * - TriadAttitude(): Shuster's TRIAD (Shuster & Oh 1981) from an
 *   accelerometer + magnetometer pair — full attitude, the natural
 *   bootstrap for Mekf9. The gravity pair anchors the primary axis (it
 *   is the more accurate observation).
 * - LevelFromAccel(): roll/pitch-only (zero yaw) from the accelerometer
 *   alone — the honest bootstrap for Mekf6, whose yaw is unobservable
 *   anyway.
 * - MagReferenceEnu(): builds Mekf9::Params::mag_reference from the
 *   local declination/inclination (e.g. from the World Magnetic Model
 *   for the deployment site).
 *
 * Frame conventions (matching the filters): the inertial frame is a
 * local tangent frame with z UP; gravity is (0, 0, -g); a level, static
 * accelerometer reads (0, 0, -g). For MagReferenceEnu the inertial
 * frame is specifically ENU (x east, y north, z up); declination is
 * measured from true north toward east, inclination (dip) positive
 * downward. Quaternions are body-to-inertial, w-x-y-z.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_ESTIMATION_ATTITUDE_INIT_HPP
#define XMNAV_ESTIMATION_ATTITUDE_INIT_HPP

#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace xmotion {

// Full attitude from a simultaneous accelerometer + magnetometer sample
// (TRIAD, gravity as the anchored pair). Returns false — leaving *q
// untouched — when any input is non-finite/near-zero or the two
// directions are close to collinear (min_cross_norm on the unit-vector
// cross products), where TRIAD degenerates.
inline bool TriadAttitude(const Eigen::Vector3d &accel,
                          const Eigen::Vector3d &mag,
                          const Eigen::Vector3d &mag_reference,
                          Eigen::Quaterniond *q,
                          double min_cross_norm = 1e-2) {
  if (!accel.allFinite() || !mag.allFinite() || !mag_reference.allFinite()) {
    return false;
  }
  const double an = accel.norm(), mn = mag.norm(), rn = mag_reference.norm();
  if (an < 1e-9 || mn < 1e-9 || rn < 1e-9) return false;

  // reference pair (inertial): gravity direction and the field
  const Eigen::Vector3d r1(0.0, 0.0, -1.0);
  const Eigen::Vector3d r2 = mag_reference / rn;
  // body observations: the accelerometer reads the gravity direction
  const Eigen::Vector3d b1 = accel / an;
  const Eigen::Vector3d b2 = mag / mn;

  Eigen::Vector3d r_cross = r1.cross(r2);
  Eigen::Vector3d b_cross = b1.cross(b2);
  if (r_cross.norm() < min_cross_norm || b_cross.norm() < min_cross_norm) {
    return false;  // collinear pair: heading undetermined
  }
  r_cross.normalize();
  b_cross.normalize();

  Eigen::Matrix3d Ri, Rb;
  Ri.col(0) = r1;
  Ri.col(1) = r_cross;
  Ri.col(2) = r1.cross(r_cross);
  Rb.col(0) = b1;
  Rb.col(1) = b_cross;
  Rb.col(2) = b1.cross(b_cross);

  // r = R_i_from_b b  =>  R = Ri Rb^T
  *q = Eigen::Quaterniond(Ri * Rb.transpose()).normalized();
  return true;
}

// Roll/pitch from the accelerometer alone, yaw fixed at zero (ZYX
// convention). Returns false on non-finite/near-zero input.
inline bool LevelFromAccel(const Eigen::Vector3d &accel,
                           Eigen::Quaterniond *q) {
  if (!accel.allFinite() || accel.norm() < 1e-9) return false;
  const Eigen::Vector3d a = accel.normalized();
  // a = R^T (0,0,-1) with R = Ry(pitch) Rx(roll):
  //   a = (sin(pitch), -sin(roll) cos(pitch), -cos(roll) cos(pitch))
  const double pitch = std::asin(std::clamp(a(0), -1.0, 1.0));
  const double roll = std::atan2(-a(1), -a(2));
  *q = Eigen::Quaterniond(
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
           .normalized();
  return true;
}

// Local magnetic field in the ENU inertial frame from declination d
// (rad, east of true north), inclination i (rad, positive down), and
// field magnitude (the units the magnetometer reports).
inline Eigen::Vector3d MagReferenceEnu(double declination,
                                       double inclination,
                                       double magnitude = 1.0) {
  const double ch = std::cos(inclination);
  return magnitude * Eigen::Vector3d(ch * std::sin(declination),
                                     ch * std::cos(declination),
                                     -std::sin(inclination));
}

}  // namespace xmotion

#endif  // XMNAV_ESTIMATION_ATTITUDE_INIT_HPP
