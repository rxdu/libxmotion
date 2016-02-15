/*
 * car_sim_process.cpp
 *
 *  Created on: Aug 5, 2015
 *      Author: rdu
 */

#include <sim_process/quad_sim_process.h>
#include <string>

#include "library/g3log/g3log.hpp"


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
	// update robot state


	// process image
	line_det_.BinarizeImage(rs_m.mono_image);

	// invoke controller update


	// set control variables
	// balance force 4800 rpm ~ 6.11e-8*4800*4800*4/9.8 = 0.57458938775 kg
	cmd_m.vel_cmd[0] = 4800;
	cmd_m.vel_cmd[0] = 4800;
	cmd_m.vel_cmd[0] = 4800;
	cmd_m.vel_cmd[0] = 4800;


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


