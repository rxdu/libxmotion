/*
 * vis_data_transmitter.h
 *
 *  Created on: May 26, 2016
 *      Author: rdu
 */

#ifndef SRC_CONTROL_SRC_APPS_VIS_DATA_TRANSMITTER_H_
#define SRC_CONTROL_SRC_APPS_VIS_DATA_TRANSMITTER_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Geometry"
#include <lcm/lcm-cpp.hpp>

#include "common/control_types.h"
#include "quad_ctrl/estimator/quad_state.h"

namespace srcl_ctrl {

class QuadVisDataTransmitter {
public:
	QuadVisDataTransmitter(std::shared_ptr<lcm::LCM> lcm_ptr);
	~QuadVisDataTransmitter();

private:
	std::shared_ptr<lcm::LCM> lcm_;

public:
	void SendRobotStateDataToROS(const QuadState& rs);
	void SendPoseToROS(Point3f pos, Eigen::Quaterniond quat);
	void SendQuadTransformToROS(Point3f pos, Eigen::Quaterniond quat);
	void SendLaserPointsToROS(const std::vector<Point3f>& pts);
};

}


#endif /* SRC_CONTROL_SRC_APPS_VIS_DATA_TRANSMITTER_H_ */
