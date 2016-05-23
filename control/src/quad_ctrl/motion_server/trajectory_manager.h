/*
 * trajectory_manager.h
 *
 *  Created on: Apr 12, 2016
 *      Author: rdu
 */

#ifndef SRC_CONTROL_MOTION_TRAJECTORY_MANAGER_H_
#define SRC_CONTROL_MOTION_TRAJECTORY_MANAGER_H_

#include <common/control_types.h>
#include <vector>
#include <cstdint>


namespace srcl_ctrl {

class TrajectoryManager{
public:
	TrajectoryManager();
	~TrajectoryManager();

private:
	std::vector<UAVTrajectoryPoint> traj_;

public:
	void ClearTrajectory();
	void SetTrajectory(std::vector<UAVTrajectoryPoint>& traj);
	void SetTestTrajectory();
	void SetTestStraigtTrajectory();
	UAVTrajectoryPoint GetTrajectoryPoint(uint64_t t);
};


}

#endif /* SRC_CONTROL_MOTION_TRAJECTORY_MANAGER_H_ */
