/*
 * geomark_graph.h
 *
 *  Created on: Feb 16, 2017
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PATH_REPAIR_GEOMARK_GRAPH_H_
#define PLANNING_SRC_PATH_REPAIR_GEOMARK_GRAPH_H_

#include <memory>

#include "eigen3/Eigen/Geometry"

#include "common/librav_types.h"
#include "common/librav_math.h"

#include "planning/graph/graph.h"
#include "map/map_type.h"
#include "map/map_info.h"
#include "map/map_utils.h"
#include "geometry/geo_mark.h"
#include "geometry/cube_array.h"

namespace librav
{
class GeoMarkGraph {
public:
	GeoMarkGraph():
		sgrid_vtx_max_(0),
		goal_height_min_(0.5),
		goal_height_max_(1.5){};
	~GeoMarkGraph(){};

private:
	MapInfo sgrid_map_info_;
	uint64_t sgrid_vtx_max_;
	std::shared_ptr<SquareGrid> sgrid_;
	std::shared_ptr<Graph_t<SquareCell*>> sgrid_graph_;

	Graph_t<GeoMark> sgrid_geomark_graph_;

	TransformationMath::Transform3D transf_;
public:
	Position3Dd pos_;
	Eigen::Quaterniond quat_;
	double goal_height_min_;
	double goal_height_max_;

public:
	Graph_t<GeoMark> combined_graph_base_;
	Graph_t<GeoMark> combined_graph_;

public:
	void SetGoalHeightRange(double height_min, double height_max);
	void UpdateVehiclePose(Position3Dd pos, Eigen::Quaterniond quat);
	void UpdateSquareGridInfo(std::shared_ptr<Graph_t<SquareCell*>> graph, const Map_t<SquareGrid>& map);

	int64_t MergeCubeArrayInfo(std::shared_ptr<Graph<CubeCell&>> cg, std::shared_ptr<CubeArray> ca);
};
}

#endif /* PLANNING_SRC_PATH_REPAIR_GEOMARK_GRAPH_H_ */
