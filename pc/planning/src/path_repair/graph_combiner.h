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
#include "quad_flat/quad_polyopt.h"

#include "utils/src/frame/transformation.h"

namespace srcl_ctrl
{
template<typename BaseGraphType, typename BaseDataStructType>
class GraphCombiner {
public:
	GraphCombiner():
		max_id_val_(0),
		desired_height_(0){};
	~GraphCombiner(){};

private:
	std::shared_ptr<Graph_t<BaseGraphType>> base_graph_;
	std::shared_ptr<BaseDataStructType> base_ds_;

	Graph_t<GeoMark> base_mark_graph_;
	MapInfo base_map_info_;

	uint64_t max_id_val_;

	utils::Transformation::Transform3D transf_;
public:
	Position3Dd pos_;
	Eigen::Quaterniond quat_;
	double desired_height_;

public:
	Graph_t<GeoMark> combined_graph_base_;
	Graph_t<GeoMark> combined_graph_;

public:
	void SetDesiredHeight(double height) { desired_height_ = height; };

	void UpdateVehiclePose(Position3Dd pos, Eigen::Quaterniond quat)
	{
		pos_ = pos;
		quat_ = quat;

		transf_.trans = utils::Transformation::Translation3D(pos.x,pos.y,pos.z);
		transf_.quat = quat_;
	};

	void SetBaseGraph(std::shared_ptr<Graph_t<BaseGraphType>> graph, std::shared_ptr<BaseDataStructType> base_ds,
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
			mark1.position.z = desired_height_;//pos_.z;
			mark1.source = GeoMarkSource::PLANAR_MAP;
			mark1.source_id = edge.src_->bundled_data_->data_id_;
			edge.src_->bundled_data_->geo_mark_id_ = mark1.data_id_;

			mark2.data_id_ = edge.dst_->vertex_id_;
			Position2Dd ref_world_pos2 = MapUtils::CoordinatesFromMapPaddedToRefWorld(edge.dst_->bundled_data_->location_, info);
			mark2.position.x = ref_world_pos2.x;
			mark2.position.y = ref_world_pos2.y;
			mark2.position.z = desired_height_;//pos_.z;
			mark2.source = GeoMarkSource::PLANAR_MAP;
			mark2.source_id = edge.dst_->bundled_data_->data_id_;
			edge.dst_->bundled_data_->geo_mark_id_ = mark2.data_id_;

			combined_graph_base_.AddEdge(mark1, mark2, edge.cost_);
		}
	};

	uint64_t CombineBaseWithCubeArrayGraph(std::shared_ptr<CubeArray> ca, std::shared_ptr<Graph<CubeCell&>> cg)
	{
		uint64_t start_id;

		// TODO possible improvement: only erase newly added vertices in the last iteration
		combined_graph_.ClearGraph();
		for(auto& edge : combined_graph_base_.GetGraphEdges()) {
			edge.src_->bundled_data_.position.z = desired_height_;//pos_.z;
			edge.dst_->bundled_data_.position.z = desired_height_;//pos_.z;
			combined_graph_.AddEdge(edge.src_->bundled_data_, edge.dst_->bundled_data_, edge.cost_);
		}

		GeoMark mark1, mark2;
		for(auto& edge:cg->GetGraphEdges())
		{
			mark1.data_id_ = max_id_val_ + edge.src_->vertex_id_;
			mark1.position = utils::Transformation::TransformPosition3D(transf_, edge.src_->bundled_data_.location_);
//			mark1.position = edge.src_->bundled_data_.location_;
			mark1.source = GeoMarkSource::LASER_OCTOMAP;
			mark1.source_id = edge.src_->bundled_data_.data_id_;
			edge.src_->bundled_data_.geo_mark_id_ = mark1.data_id_;

			mark2.data_id_ = max_id_val_ + edge.dst_->vertex_id_;
			mark2.position =  utils::Transformation::TransformPosition3D(transf_, edge.dst_->bundled_data_.location_);
//			mark2.position =  edge.dst_->bundled_data_.location_;
			mark2.source = GeoMarkSource::LASER_OCTOMAP;
			mark2.source_id = edge.dst_->bundled_data_.data_id_;
			edge.dst_->bundled_data_.geo_mark_id_ = mark2.data_id_;

			combined_graph_.AddEdge(mark1, mark2, edge.cost_);
		}

		// connect starting points
		GeoMark map2d_start_mark, cube_start_mark;

		Position2D map2d_start_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(pos_.x,pos_.y), base_map_info_);
		uint64_t map2d_start_id = base_ds_->GetIDFromPosition(map2d_start_pos.x, map2d_start_pos.y);

		// old policy
		//if(base_graph_->GetVertexFromID(map2d_start_id) == nullptr)
		//	return false;
		// new policy
		if(base_graph_->GetVertexFromID(map2d_start_id) != nullptr)
		{
			uint64_t geo_start_id = base_graph_->GetVertexFromID(map2d_start_id)->bundled_data_->geo_mark_id_;
			map2d_start_mark = combined_graph_.GetVertexFromID(geo_start_id)->bundled_data_;

			start_id = geo_start_id;
		}
		else
		{
			map2d_start_mark.data_id_ = max_id_val_ + cg->GetGraphVertices().size() + 1;
			map2d_start_mark.position = pos_;
			map2d_start_mark.source = GeoMarkSource::VIRTUAL_POINT;
			map2d_start_mark.source_id = 0;

			start_id = max_id_val_ + cg->GetGraphVertices().size() + 1;
		}

		std::set<uint32_t> hei_set;
		for(auto& st_cube : ca->GetStartingCubes())
		{
			uint64_t cube_start_id = ca->cubes_[st_cube].geo_mark_id_;
			cube_start_mark = combined_graph_.GetVertexFromID(cube_start_id)->bundled_data_;

			double cost = std::sqrt(std::pow(map2d_start_mark.position.x - cube_start_mark.position.x, 2) +
					std::pow(map2d_start_mark.position.y - cube_start_mark.position.y, 2) +
					std::pow(map2d_start_mark.position.z - cube_start_mark.position.z, 2));
			combined_graph_.AddEdge(map2d_start_mark, cube_start_mark, cost);

			hei_set.insert(ca->cubes_[st_cube].index_.z);
		}

		// get all vertices of the cube graph around the current flight height
		std::vector<uint64_t> hei_vertices;
		for(auto& vtx:cg->GetGraphVertices())
		{
			for(auto& hei_ele:hei_set)
			{
				if(vtx->bundled_data_.index_.z == hei_ele)
				{
					hei_vertices.push_back(vtx->bundled_data_.geo_mark_id_);
					break;
				}
			}
		}

		for(auto& v:hei_vertices)
		{
			GeoMark mark2d, mark3d;
			mark3d = combined_graph_.GetVertexFromID(v)->bundled_data_;

			Position2D map_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(mark3d.position.x,mark3d.position.y), base_map_info_);
			uint64_t map2d_id = base_ds_->GetIDFromPosition(map_pos.x, map_pos.y);
			if(base_graph_->GetVertexFromID(map2d_id) == nullptr)
				continue;
			uint64_t geo2d_id = base_graph_->GetVertexFromID(map2d_id)->bundled_data_->geo_mark_id_;
			mark2d = combined_graph_.GetVertexFromID(geo2d_id)->bundled_data_;

			double cost = std::sqrt(std::pow(mark3d.position.x - mark2d.position.x, 2) +
					std::pow(mark3d.position.y - mark2d.position.y, 2) +
					std::pow(mark3d.position.z - mark2d.position.z, 2));
			combined_graph_.AddEdge(mark3d, mark2d, cost);
		}

		return start_id;
	}
};
}

#endif /* PLANNING_SRC_PATH_REPAIR_GRAPH_COMBINER_H_ */
