/*
 * @file sim_log.hpp
 * @brief Preallocated recording of a simulation run + JSONL export.
 *
 * Copyright (c) 2026 Ruixiang Du (rdu)
 */

#ifndef XMNAV_SIM_SIM_LOG_HPP
#define XMNAV_SIM_SIM_LOG_HPP

#include <ostream>

#include <eigen3/Eigen/Dense>

namespace xmotion {

class SimLog {
 public:
  void Reset(int steps, int state_dim, int control_dim) {
    time_.resize(steps);
    states_.resize(steps, state_dim);
    measurements_.resize(steps, state_dim);
    controls_.resize(steps, control_dim);
    count_ = 0;
  }

  template <typename S, typename C>
  void Append(double t, const S &x, const S &z, const C &u) {
    time_(count_) = t;
    states_.row(count_) = x.transpose();
    measurements_.row(count_) = z.transpose();
    controls_.row(count_) = u.transpose();
    ++count_;
  }

  int size() const { return count_; }
  double time(int i) const { return time_(i); }
  Eigen::VectorXd state(int i) const {
    return states_.row(i).transpose();
  }
  Eigen::VectorXd control(int i) const {
    return controls_.row(i).transpose();
  }
  const Eigen::MatrixXd &states() const { return states_; }
  const Eigen::MatrixXd &measurements() const { return measurements_; }
  const Eigen::MatrixXd &controls() const { return controls_; }

  // one JSON object per run
  void WriteJsonl(std::ostream &os) const {
    auto write = [&](const Eigen::MatrixXd &m) {
      os << '[';
      for (int r = 0; r < count_; ++r) {
        if (r != 0) os << ',';
        os << '[';
        for (Eigen::Index c = 0; c < m.cols(); ++c) {
          if (c != 0) os << ',';
          os << m(r, c);
        }
        os << ']';
      }
      os << ']';
    };
    os << "{\"dt\":" << (count_ > 1 ? time_(1) - time_(0) : 0.0)
       << ",\"states\":";
    write(states_);
    os << ",\"measurements\":";
    write(measurements_);
    os << ",\"controls\":";
    write(controls_);
    os << "}\n";
  }

 private:
  Eigen::VectorXd time_;
  Eigen::MatrixXd states_;
  Eigen::MatrixXd measurements_;
  Eigen::MatrixXd controls_;
  int count_ = 0;
};

}  // namespace xmotion

#endif  // XMNAV_SIM_SIM_LOG_HPP
