/*
 * geo_mark.h
 *
 *  Created on: Sep 11, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PATH_REPAIR_GEO_MARK_H_
#define PLANNING_SRC_PATH_REPAIR_GEO_MARK_H_

#include <cmath>

#include "navtypes/navtypes.hpp"

namespace ivnav {

enum class GeoMarkSource {
	NOT_SPECIFIED,
	LASER_OCTOMAP,
	PLANAR_MAP,
	VIRTUAL_POINT
};

struct GeoMark
{
	GeoMark():
		data_id_(0),
		source(GeoMarkSource::NOT_SPECIFIED),
		source_id(0){};
	GeoMark(uint64_t id):
		data_id_(id),
		source(GeoMarkSource::NOT_SPECIFIED),
		source_id(0){};
	~GeoMark(){};

	uint64_t data_id_;
	Position3Dd position;
	GeoMarkSource source;
	uint64_t source_id;

	double GetHeuristic(const GeoMark& other_struct) const {
		double x_error =  this->position.x - other_struct.position.x;
		double y_error =  this->position.y - other_struct.position.y;
		double z_error =  this->position.z - other_struct.position.z;;

		return std::sqrt(std::pow(x_error,2) + std::pow(y_error,2) + std::pow(z_error, 2));
	}
};

}

#endif /* PLANNING_SRC_PATH_REPAIR_GEO_MARK_H_ */
