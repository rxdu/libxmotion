/*
 * trajectory_manager.h
 *
 *  Created on: Apr 12, 2016
 *      Author: rdu
 */

#ifndef SRC_CONTROL_MOTION_TRAJECTORY_MANAGER_H_
#define SRC_CONTROL_MOTION_TRAJECTORY_MANAGER_H_

#include <vector>
#include <cstdint>

#include "common/robot_datatypes.h"

namespace srcl_ctrl {

class TrajectoryManager{
public:
	TrajectoryManager();
	~TrajectoryManager();

private:
	std::vector<TrajectoryPoint> traj_;

public:
	void ClearTrajectory();
	TrajectoryPoint GetTrajectoryPoint(uint64_t t);
};


}

#endif /* SRC_CONTROL_MOTION_TRAJECTORY_MANAGER_H_ */
