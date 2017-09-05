/*
 * quad_data_transmitter.h
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
#include "quad_ctrl/data_types/quad_state.h"

namespace srcl_ctrl {

class QuadDataTransmitter {
public:
	QuadDataTransmitter(std::shared_ptr<lcm::LCM> lcm_ptr);
	~QuadDataTransmitter();

private:
	std::shared_ptr<lcm::LCM> lcm_;

	void SendQuadTransform(Point3f pos, Eigen::Quaterniond quat);
	void SendLaserPoints(const std::vector<Point3f>& pts);
	void SendLaserPoints(const std::vector<Point3f>& pts, Point3f pos, Eigen::Quaterniond quat);

public:
	void SendQuadStateData(const QuadState& rs);
	void SendSystemTime(uint64_t sys_t);
};

}


#endif /* SRC_CONTROL_SRC_QUAD_CTRL_DATA_TRANS_DATA_TRANSMITTER_H_ */
