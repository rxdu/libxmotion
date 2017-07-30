/*
 * quad_mixer.h
 *
 *  Created on: Jun 25, 2017
 *      Author: rdu
 */

#ifndef CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUAD_MIXER_H_
#define CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUAD_MIXER_H_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

namespace librav {

class QuadMixer
{
private:
	// internal variables
	Eigen::Matrix<double,4,4> plus_type_trans_;
	Eigen::Matrix<double,4,4> plus_type_trans_inv_;
	Eigen::Matrix<double,4,4> x_type_trans_;
	Eigen::Matrix<double,4,4> x_type_trans_inv_;

	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type);
};

}

#endif /* CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUAD_MIXER_H_ */
