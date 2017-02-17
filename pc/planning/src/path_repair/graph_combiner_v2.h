/*
 * graph_combiner.h
 *
 *  Created on: Sep 12, 2016
 *      Author: rdu
 */

#ifndef PLANNING_SRC_PATH_REPAIR_GRAPH_COMBINER_H_
#define PLANNING_SRC_PATH_REPAIR_GRAPH_COMBINER_H_

#include <memory>

#include "eigen3/Eigen/Geometry"

#include "graph/graph.h"
#include "common/planning_types.h"
#include "geometry/geo_mark.h"
#include "map/map_info.h"
#include "map/map_utils.h"
#include "geometry/cube_array/cube_array.h"

#include "utils/src/frame/transformation.h"

namespace srcl_ctrl
{
class GraphCombinerV2 {
public:
	GraphCombinerV2():
		max_id_val_(0),
		desired_height_min_(0.5),
		desired_height_max_(1.5){};
	~GraphCombinerV2(){};

private:
	std::shared_ptr<Graph_t<SquareCell*>> base_graph_;
	std::shared_ptr<SquareGrid> base_ds_;

	Graph_t<GeoMark> base_mark_graph_;
	MapInfo base_map_info_;

	uint64_t max_id_val_;

	utils::Transformation::Transform3D transf_;
public:
	Position3Dd pos_;
	Eigen::Quaterniond quat_;
	double desired_height_min_;
	double desired_height_max_;

public:
	Graph_t<GeoMark> combined_graph_base_;
	Graph_t<GeoMark> combined_graph_;

public:
	void SetDesiredHeight(double height_min, double height_max) {
		desired_height_min_ = height_min;
		desired_height_max_ = height_max;
	};

	void UpdateVehiclePose(Position3Dd pos, Eigen::Quaterniond quat)
	{
		pos_ = pos;
		quat_ = quat;

		transf_.trans = utils::Transformation::Translation3D(pos.x,pos.y,pos.z);
		transf_.quat = quat_;
	};

	void SetBaseGraph(std::shared_ptr<Graph_t<SquareCell*>> graph, std::shared_ptr<SquareGrid> base_ds,
			uint64_t max_id_val, MapInfo info)
	{
		base_graph_ = graph;
		base_ds_ = base_ds;
		max_id_val_ = max_id_val;
		base_map_info_ = info;

		GeoMark mark1, mark2;
		for(auto& edge:base_graph_->GetGraphEdges())
		{
			mark1.data_id_ = edge.src_->vertex_id_;
			Position2Dd ref_world_pos1 = MapUtils::CoordinatesFromMapPaddedToRefWorld(edge.src_->bundled_data_->location_, info);
			mark1.position.x = ref_world_pos1.x;
			mark1.position.y = ref_world_pos1.y;
			mark1.position.z = desired_height_min_;//pos_.z;
			mark1.source = GeoMarkSource::PLANAR_MAP;
			mark1.source_id = edge.src_->bundled_data_->data_id_;
			edge.src_->bundled_data_->geo_mark_id_ = mark1.data_id_;

			mark2.data_id_ = edge.dst_->vertex_id_;
			Position2Dd ref_world_pos2 = MapUtils::CoordinatesFromMapPaddedToRefWorld(edge.dst_->bundled_data_->location_, info);
			mark2.position.x = ref_world_pos2.x;
			mark2.position.y = ref_world_pos2.y;
			mark2.position.z = desired_height_min_;//pos_.z;
			mark2.source = GeoMarkSource::PLANAR_MAP;
			mark2.source_id = edge.dst_->bundled_data_->data_id_;
			edge.dst_->bundled_data_->geo_mark_id_ = mark2.data_id_;

			combined_graph_base_.AddEdge(mark1, mark2, edge.cost_);
		}
	};

	void CombineBaseWithCubeArrayGraph(std::shared_ptr<CubeArray> ca, std::shared_ptr<Graph<CubeCell&>> cg)
	{
		// TODO possible improvement: only erase newly added vertices in the last iteration
		combined_graph_.ClearGraph();
		for(auto& edge : combined_graph_base_.GetGraphEdges()) {
			edge.src_->bundled_data_.position.z = pos_.z;
			edge.dst_->bundled_data_.position.z = pos_.z;
			combined_graph_.AddEdge(edge.src_->bundled_data_, edge.dst_->bundled_data_, edge.cost_);
		}

		// first add all vertices from cube array graph to geomark graph
		GeoMark mark1, mark2;
		for(auto& edge:cg->GetGraphEdges())
		{
			mark1.data_id_ = max_id_val_ + edge.src_->vertex_id_;
			//mark1.position = utils::Transformation::TransformPosition3D(transf_, edge.src_->bundled_data_.location_);
			mark1.position = edge.src_->bundled_data_.location_;
			mark1.source = GeoMarkSource::LASER_OCTOMAP;
			mark1.source_id = edge.src_->bundled_data_.data_id_;
			edge.src_->bundled_data_.geo_mark_id_ = mark1.data_id_;

			mark2.data_id_ = max_id_val_ + edge.dst_->vertex_id_;
			//mark2.position =  utils::Transformation::TransformPosition3D(transf_, edge.dst_->bundled_data_.location_);
			mark2.position =  edge.dst_->bundled_data_.location_;
			mark2.source = GeoMarkSource::LASER_OCTOMAP;
			mark2.source_id = edge.dst_->bundled_data_.data_id_;
			edge.dst_->bundled_data_.geo_mark_id_ = mark2.data_id_;

			combined_graph_.AddEdge(mark1, mark2, edge.cost_);
		}

		// find boundary vertices in octomap, relative to 2d map
		std::vector<uint64_t> boundary_set = ca->GetFreeCubesAroundHeight(pos_.z);
//		if(!ca->GetCubeHeightIndexAtHeight(desired_height_min_, boundary_set))
//			return;

		// merge 2d and 3d graph
		for(auto bid:boundary_set)
		{
			GeoMark mark2d, mark3d;

			// find 3d geomark
			uint64_t geo3d_id = ca->cubes_[bid].geo_mark_id_;
			mark3d = combined_graph_.GetVertexFromID(geo3d_id)->bundled_data_;

			// find 2d geomark
			Position2D map_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(mark3d.position.x,mark3d.position.y), base_map_info_);
			uint64_t map2d_id = base_ds_->GetIDFromPosition(map_pos.x, map_pos.y);
			if(base_graph_->GetVertexFromID(map2d_id) == nullptr)
				continue;

			uint64_t geo2d_id = base_graph_->GetVertexFromID(map2d_id)->bundled_data_->geo_mark_id_;
			mark2d = combined_graph_.GetVertexFromID(geo2d_id)->bundled_data_;

			// create edge
			double cost;

			if(ca->cubes_[bid].location_.z <= desired_height_max_ &&
					ca->cubes_[bid].location_.z >= desired_height_min_) {
				cost = 0;
			}
			else {
				cost = std::sqrt(std::pow(mark3d.position.x - mark2d.position.x, 2) +
					std::pow(mark3d.position.y - mark2d.position.y, 2) +
					std::pow(mark3d.position.z - mark2d.position.z, 2));
			}

			combined_graph_.AddEdge(mark3d, mark2d, cost);
		}
	}
};
}

#endif /* PLANNING_SRC_PATH_REPAIR_GRAPH_COMBINER_H_ */
