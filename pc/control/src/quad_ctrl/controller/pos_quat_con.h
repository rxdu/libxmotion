/*
 * pos_quat_con.h
 *
 *  Created on: Mar 3, 2016
 *      Author: rdu
 */

#ifndef CONTROL_POS_QUAT_CON_H_
#define CONTROL_POS_QUAT_CON_H_

#include "quad_ctrl/data_types/quad_state.h"

namespace srcl_ctrl {

typedef struct
{
	// input of position controller
	float pos_d[3];
	float vel_d[3];
	float acc_d[3];
	float jerk_d[3];
	float yaw_d;
	float yaw_rate_d;
}PosQuatConInput;

typedef struct{
	// output of position controller (using Quaternion)
	Eigen::Quaterniond quat_d;
	float rot_rate_d[3];
	float ftotal_d;
}PosQuatConOutput;

class PosQuatCon
{
public:
	PosQuatCon(const QuadState& _rs);
	~PosQuatCon();

private:
	float kp_0;
	float ki_0;
	float kd_0;
	float kp_1;
	float ki_1;
	float kd_1;
	float kp_2;
	float ki_2;
	float kd_2;

private:
	const QuadState& rs_;

	double pos_e_integral[3];
	double zint_uppper_limit;
	double zint_lower_limit;
	double xyint_uppper_limit;
	double xyint_lower_limit;

	double ts_;
	double last_acc_desired_[3];

public:
	void SetControlGains(float _kp_0, float _ki_0, float _kd_0,
			float _kp_1, float _ki_1, float _kd_1,
			float _kp_2, float _ki_2, float _kd_2)
	{
		kp_0 = _kp_0;
		ki_0 = _ki_0;
		kd_0 = _kd_0;
		kp_1 = _kp_1;
		kd_1 = _kd_1;
		ki_1 = _ki_1;
		kp_2 = _kp_2;
		ki_2 = _ki_2;
		kd_2 = _kd_2;
	};

	void Update(const PosQuatConInput& input, PosQuatConOutput& output);
	void SetControlPeriod(double ts) { ts_ = ts; };
};

}



#endif /* CONTROL_POS_QUAT_CON_H_ */
