/*
 * att_quat_con.h
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#ifndef CONTROL_ATT_QUAT_CON_H_
#define CONTROL_ATT_QUAT_CON_H_

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "controller/controller_base.h"

namespace srcl_ctrl {

class AttQuatCon: public Controller
{
public:
	AttQuatCon(RobotState *_rs);
	~AttQuatCon();

private:
	float kp_phi;
	float kd_phi;
	float kp_theta;
	float kd_theta;
	float kp_psi;
	float kd_psi;

private:
	Eigen::Matrix<double,4,1> CalcMotorCmd(Eigen::Matrix<float,4,1> force_toqure);

public:
	void Update(ControlInput *input, ControlOutput *cmd);
};

}

#endif /* CONTROL_ATT_QUAT_CON_H_ */
