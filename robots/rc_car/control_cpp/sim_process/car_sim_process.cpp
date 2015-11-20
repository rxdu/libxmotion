/*
 * car_sim_process.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#include <string>

#include "library/g3log/g3log.hpp"

#include "sim_process/car_sim_process.h"

using namespace RobotToolkitRIVeR;

CarSimProcess::CarSimProcess(int client_id):
	SimProcess(new RCCarClient(client_id)),
	process_loop_count(0)
{

}

CarSimProcess::~CarSimProcess(void)
{

}

void CarSimProcess::SimLoopUpdate(void)
{
	// update robot state


	// process image
	line_det_.BinarizeImage(rs_m.mono_image);

	// invoke controller update


	// set control variables
	cmd_m.cmd.driving_vel_lcmd = 15;
	cmd_m.cmd.driving_vel_rcmd = 15;
	cmd_m.cmd.steering_ang_cmd = 0.01;

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


