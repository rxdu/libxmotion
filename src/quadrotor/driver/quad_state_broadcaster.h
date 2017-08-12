/*
 * quad_state_broadcaster.h
 *
 *  Created on: May 26, 2016
 *      Author: rdu
 */

#ifndef SRC_CONTROL_SRC_QUAD_CTRL_DATA_TRANS_DATA_TRANSMITTER_H_
#define SRC_CONTROL_SRC_QUAD_CTRL_DATA_TRANS_DATA_TRANSMITTER_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Geometry"
#include <lcm/lcm-cpp.hpp>

#include "common/control_types.h"
#include "control/quad_ctrl/data_types/quad_state.h"

namespace librav {

class QuadStateBroadcaster {
public:
	QuadStateBroadcaster(std::shared_ptr<lcm::LCM> lcm_ptr);
	~QuadStateBroadcaster();

public:
	void SendQuadStateData(const QuadState& rs);
	void SendSystemTime(uint64_t sys_t);

private:
	std::shared_ptr<lcm::LCM> lcm_;

	void SendQuadTransform(Point3f pos, Eigen::Quaterniond quat);
	void SendLaserPoints(const std::vector<Point3f>& pts);
	void SendLaserPoints(const std::vector<Point3f>& pts, Point3f pos, Eigen::Quaterniond quat);
};

}


#endif /* SRC_CONTROL_SRC_QUAD_CTRL_DATA_TRANS_DATA_TRANSMITTER_H_ */
