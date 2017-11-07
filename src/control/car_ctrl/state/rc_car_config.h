/*
 * rc_car_config.h
 *
 *  Created on: Aug 10, 2017
 *      Author: rdu
 */

#ifndef CONTROL_RC_CAR_CTRL_RC_CAR_CONFIG_H_
#define CONTROL_RC_CAR_CTRL_RC_CAR_CONFIG_H_

namespace librav
{

  // resolution of the main vision sensor
  #define IMG_RES_X 160
  #define IMG_RES_Y 90

  const double max_steer_angle = 30.0;	// in degree
  const double max_drive_speed = 100.0;	// in round/s
}

#endif /* CONTROL_RC_CAR_CTRL_RC_CAR_CONFIG_H_ */
