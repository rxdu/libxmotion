#include <iostream>

#include "estimation/mekf6.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  std::cout << "Hello, World!" << std::endl;

  Mekf6::Params params;
  params.gravity_constant = 9.81;
  params.init_quaternion = Eigen::Quaterniond(1, 0, 0, 0);
  params.init_state = Eigen::VectorXd::Zero(Mekf6::StateDimension);
  params.init_state_cov =
      Eigen::MatrixXd::Identity(Mekf6::StateDimension, Mekf6::StateDimension);
  params.init_observation_noise_cov = Eigen::MatrixXd::Identity(
      Mekf6::ObservationDimension, Mekf6::ObservationDimension);
  params.sigma_omega = Eigen::Vector3d(0.01, 0.01, 0.01);
  params.sigma_f = Eigen::Vector3d(0.01, 0.01, 0.01);
  params.sigma_beta_omega = Eigen::Vector3d(0.01, 0.01, 0.01);
  params.sigma_beta_f = Eigen::Vector3d(0.01, 0.01, 0.01);

  Mekf6 mekf;
  mekf.Initialize(params);

  while (true) {
    Mekf6::ControlInput gyro_tilde;
    gyro_tilde << 0.01, 0.01, 0.01;
    Mekf6::Observation accel_tilde;
    accel_tilde << 0.01, 0.01, 0.01;
    mekf.Update(gyro_tilde, accel_tilde, 0.01);

    std::cout << "q(w,x,y,z): " << mekf.GetQuaternion().w() << ", "
              << mekf.GetQuaternion().x() << ", " << mekf.GetQuaternion().y()
              << ", " << mekf.GetQuaternion().z() << std::endl;
  }

  return 0;
}
