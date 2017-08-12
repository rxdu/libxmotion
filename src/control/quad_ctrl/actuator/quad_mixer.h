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

#include "control/quad_ctrl/state/quad_state.h"

namespace librav {

class QuadMixer
{
public:
	QuadMixer(const QuadState& _rs);

	QuadCmd CalcMotorCmd(float force, float toqure[3], QuadFlightType type);
	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure, QuadFlightType type);
	
private:
	const QuadState& state_;

	// internal variables
	Eigen::Matrix<double,4,4> plus_type_trans_;
	Eigen::Matrix<double,4,4> plus_type_trans_inv_;
	Eigen::Matrix<double,4,4> x_type_trans_;
	Eigen::Matrix<double,4,4> x_type_trans_inv_;	
};

}

#endif /* CONTROL_SRC_QUAD_CTRL_CONTROLLER_QUAD_MIXER_H_ */
