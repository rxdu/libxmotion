/*
 * quad_solo_config.h
 *
 *  Created on: Aug 9, 2016
 *      Author: rdu
 */

#ifndef CONTROL_QUAD_CTRL_SOLO_CONFIG_H_
#define CONTROL_QUAD_CTRL_SOLO_CONFIG_H_

namespace librav
{

// RC car model C
#ifdef RC_CAR_MODEL_C

#define IMG_RES_X 160
#define IMG_RES_Y 90

#define LASER_SCAN_RES_X 64
#define LASER_SCAN_RES_y 64

// Default model parameters
#else

#define IMG_RES_X 160
#define IMG_RES_Y 90

#define LASER_SCAN_RES_X 64
#define LASER_SCAN_RES_y 64

#endif
}

#endif /* CONTROL_QUAD_CTRL_SOLO_CONFIG_H_ */
