/*
 * mission_utils.h
 *
 *  Created on: Nov 3, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_MISSION_MISSION_UTILS_H_
#define PLANNING_SRC_MISSION_MISSION_UTILS_H_

#include <vector>
#include "common/planning_types.h"

namespace srcl_ctrl {

namespace MissionUtils {

std::vector<Position3Dd> GetKeyTurningWaypoints(std::vector<Position3Dd>& wps);

}

}

#endif /* PLANNING_SRC_MISSION_MISSION_UTILS_H_ */
