/*
 * car_sim_process.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#include <iostream>
#include <string>

#include <sim_process/quad_sim_process.h>
#include "library/g3log/g3log.hpp"
#include "control/att_euler_con.h"
#include "control/pos_euler_con.h"

using namespace srcl_ctrl;

QuadSimProcess::QuadSimProcess(int client_id):
	SimProcess(new QuadSimClient(client_id)),
	process_loop_count(0)
{

}

QuadSimProcess::~QuadSimProcess(void)
{

}

void QuadSimProcess::SimLoopUpdate(void)
{
	/********* update robot state *********/
	// Test without state estimator
	rs_.UpdateRobotState(rs_m_);

	//(Extended Kalman Filter)



	/********* update position control *********/
	// invoke position controller update
	// input: (x,y,z,u,v,w)^desired
	// output: (phi, theta, psi, p, q, r)^desired
	ControlInput pos_con_input;
	ControlOutput pos_con_output;
	PosEulerCon pos_con(&rs_);

//	if(process_loop_count < 85)
	pos_con_input.pos_d[0] = 0.8;
//	else
//		pos_con_input.pos_d[0] = 0.2;
	pos_con_input.pos_d[1] = -0.8;
	pos_con_input.pos_d[2] = 2.0;
//	std::cout << "desired height: "<< pos_con_input.pos_d[2] << std::endl;

	pos_con_input.vel_d[0] = 0;
	pos_con_input.vel_d[1] = 0;
	pos_con_input.vel_d[2] = 0;

	pos_con.Update(&pos_con_input, &pos_con_output);

	ControlInput att_con_input;
	ControlOutput att_con_output;

	att_con_input.euler_d[0] = pos_con_output.euler_d[0];
	att_con_input.euler_d[1] = pos_con_output.euler_d[1];
	att_con_input.euler_d[2] = 0;
	att_con_input.rot_rate_d[0] = 0;
	att_con_input.rot_rate_d[1] = 0;
	att_con_input.rot_rate_d[2] = 0;
	att_con_input.delta_w_F = pos_con_output.delta_w_F;

	/*------------- test attitude controller -------------*/
//	att_con_input.euler_d[0] = 0.0/180.0*3.14;
//	att_con_input.euler_d[1] = 0.0/180.0*3.14;
//	att_con_input.euler_d[2] = 0.0/180.0*3.14;
//	att_con_input.delta_w_F = 0;
//
//	att_con_input.rot_rate_d[0] = 0;
//	att_con_input.rot_rate_d[1] = 0;
//	att_con_input.rot_rate_d[2] = 0;
	/*----------------------------------------------------*/

	/********* update attitude control *********/
	// input: (phi, theta, psi, p, q, r)^desired
	// output: motor angular velocities
	AttEulerCon att_con(&rs_);
	att_con.Update(&att_con_input, &att_con_output);

	// set control variables
	// balance force 4800 rpm ~ 6.11e-8*4800*4800*4/9.8 = 0.57458938775 kg
//	cmd_m_.motor_cmd.ang_vel[0] = 4800;
//	cmd_m_.motor_cmd.ang_vel[1] = 4800;
//	cmd_m_.motor_cmd.ang_vel[2] = 4800;
//	cmd_m_.motor_cmd.ang_vel[3] = 4800;
	// velocity for rotation 5200
//	cmd_m_.motor_cmd.ang_vel[0] = 4500;
//	cmd_m_.motor_cmd.ang_vel[1] = 4500;
//	cmd_m_.motor_cmd.ang_vel[2] = 5200;
//	cmd_m_.motor_cmd.ang_vel[3] = 5200;

	cmd_m_.motor_cmd.ang_vel[0] = att_con_output.motor_ang_vel_d[0];
	cmd_m_.motor_cmd.ang_vel[1] = att_con_output.motor_ang_vel_d[1];
	cmd_m_.motor_cmd.ang_vel[2] = att_con_output.motor_ang_vel_d[2];
	cmd_m_.motor_cmd.ang_vel[3] = att_con_output.motor_ang_vel_d[3];

	//	 4761.95 , 4841.48 , 4761.95 , 4841.48
//	double test_value = 4755.00;
//	cmd_m_.motor_cmd.ang_vel[0] = test_value;
//	cmd_m_.motor_cmd.ang_vel[1] = test_value;
//	cmd_m_.motor_cmd.ang_vel[2] = test_value;
//	cmd_m_.motor_cmd.ang_vel[3] = test_value;

	std::cout << "----------------" << std::endl;
	std::cout<< "desired motor vel: ( "<< cmd_m_.motor_cmd.ang_vel[0] << " , "
			<< cmd_m_.motor_cmd.ang_vel[1] << " , "
			<< cmd_m_.motor_cmd.ang_vel[2] << " , "
			<< cmd_m_.motor_cmd.ang_vel[3] << " ) "<<std::endl;
	std::cout << "----------------" << std::endl;

	// code below is used for debugging
	process_loop_count++;

#ifdef ENABLE_LOG
	// log data
	/* data format: image(IMG_RES_X * IMG_RES_Y bytes) +							*/
//	if(process_loop_count == 10)
//	{
		std::string str;
		int i,j;

		for(i = 0; i < IMG_RES_Y; i++)
		{
			for(j = 0; j < IMG_RES_X; j++){
//				str += std::to_string((unsigned int)(rs_m.mono_image[i][j]));
				str += std::to_string((unsigned int)(line_det_.bin_image_[i][j]));
				str += ",";
			}
		}


		LOG(INFO) << str;
//	}
#endif
}


