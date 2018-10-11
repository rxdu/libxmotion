/*
 * geo_mark_graph.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: rdu
 */

#include "path_repair/geo_mark_graph.h"

using namespace librav;

void GeoMarkGraph::SetGoalHeightRange(double height_min, double height_max) {
	goal_height_min_ = height_min;
	goal_height_max_ = height_max;
};

void GeoMarkGraph::UpdateVehiclePose(Position3Dd pos, Eigen::Quaterniond quat)
{
	pos_ = pos;
	quat_ = quat;

	transf_.trans = TransMath::Translation3D(pos.x,pos.y,pos.z);
	transf_.quat = quat_;
};

void GeoMarkGraph::UpdateSquareGridInfo(std::shared_ptr<Graph_t<SquareCell*>> graph, const Map_t<SquareGrid>& map)
{
	sgrid_graph_ = graph;
	sgrid_ = map.data_model;
	sgrid_vtx_max_ = map.data_model->cells_.size();
	sgrid_map_info_ = map.info;

	GeoMark mark1, mark2;
	for(auto& edge:sgrid_graph_->GetGraphEdges())
	{
		mark1.data_id_ = edge.src_->vertex_id_;
		Position2Dd ref_world_pos1 = MapUtils::CoordinatesFromMapPaddedToRefWorld(edge.src_->bundled_data_->location_, sgrid_map_info_);
		mark1.position.x = ref_world_pos1.x;
		mark1.position.y = ref_world_pos1.y;
		mark1.position.z = goal_height_min_;//pos_.z;
		mark1.source = GeoMarkSource::PLANAR_MAP;
		mark1.source_id = edge.src_->bundled_data_->data_id_;
		edge.src_->bundled_data_->geo_mark_id_ = mark1.data_id_;

		mark2.data_id_ = edge.dst_->vertex_id_;
		Position2Dd ref_world_pos2 = MapUtils::CoordinatesFromMapPaddedToRefWorld(edge.dst_->bundled_data_->location_, sgrid_map_info_);
		mark2.position.x = ref_world_pos2.x;
		mark2.position.y = ref_world_pos2.y;
		mark2.position.z = goal_height_min_;//pos_.z;
		mark2.source = GeoMarkSource::PLANAR_MAP;
		mark2.source_id = edge.dst_->bundled_data_->data_id_;
		edge.dst_->bundled_data_->geo_mark_id_ = mark2.data_id_;

		combined_graph_base_.AddEdge(mark1, mark2, edge.cost_);
	}
}

int64_t GeoMarkGraph::MergeCubeArrayInfo(std::shared_ptr<Graph<CubeCell&>> cg, std::shared_ptr<CubeArray> ca)
{
	// update 2D geomark
	combined_graph_.ClearGraph();
	for(auto& edge : combined_graph_base_.GetGraphEdges()) {
		edge.src_->bundled_data_.position.z = pos_.z;
		edge.dst_->bundled_data_.position.z = pos_.z;
		combined_graph_.AddEdge(edge.src_->bundled_data_, edge.dst_->bundled_data_, edge.cost_);
	}

	// add all vertices from cube array graph to geomark graph
	GeoMark mark1, mark2;
	for(auto& edge:cg->GetGraphEdges())
	{
		mark1.data_id_ = sgrid_vtx_max_ + edge.src_->vertex_id_;
		//mark1.position = utils::Transformation::TransformPosition3D(transf_, edge.src_->bundled_data_.location_);
		mark1.position = edge.src_->bundled_data_.location_;
		mark1.source = GeoMarkSource::LASER_OCTOMAP;
		mark1.source_id = edge.src_->bundled_data_.data_id_;
		edge.src_->bundled_data_.geo_mark_id_ = mark1.data_id_;

		mark2.data_id_ = sgrid_vtx_max_ + edge.dst_->vertex_id_;
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
		Position2Di map_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(mark3d.position.x,mark3d.position.y), sgrid_map_info_);
		uint64_t map2d_id = sgrid_->GetIDFromPosition(map_pos.x, map_pos.y);
		if(sgrid_graph_->GetVertexFromID(map2d_id) == nullptr)
			continue;
		uint64_t geo2d_id = sgrid_graph_->GetVertexFromID(map2d_id)->bundled_data_->geo_mark_id_;
		mark2d = combined_graph_.GetVertexFromID(geo2d_id)->bundled_data_;

		// create edge
		double cost;

		if(ca->cubes_[bid].location_.z <= goal_height_max_ &&
				ca->cubes_[bid].location_.z >= goal_height_min_) {
			cost = 0;
		}
		else {
			cost = std::sqrt(std::pow(mark3d.position.x - mark2d.position.x, 2) +
					std::pow(mark3d.position.y - mark2d.position.y, 2) +
					std::pow(mark3d.position.z - mark2d.position.z, 2));
		}

		combined_graph_.AddEdge(mark3d, mark2d, cost);
	}

	// find the vertex id of current vehicle position
	uint64_t final_start_id;

	bool loc_found_2d = false;
	bool loc_found_3d = false;

	Position2Di map_pos = MapUtils::CoordinatesFromRefWorldToMapPadded(Position2Dd(pos_.x,pos_.y), sgrid_map_info_);
	uint64_t map2d_id = sgrid_->GetIDFromPosition(map_pos.x, map_pos.y);
	if(sgrid_graph_->GetVertexFromID(map2d_id) != nullptr)
		loc_found_2d = true;

	uint64_t map3d_id;
	loc_found_3d = ca->GetCubeIDAtPosition(pos_.x, pos_.y, pos_.y, map3d_id);

	if(!loc_found_2d && !loc_found_3d)
	{
		std::cerr << "Failed to start vertex from both 2d and 3d graph" << std::endl;
		return -1;
	}
	else
	{
		uint64_t geo2d_start_id;
		uint64_t geo3d_start_id;

		if(loc_found_2d)
			geo2d_start_id = sgrid_graph_->GetVertexFromID(map2d_id)->bundled_data_->geo_mark_id_;
		if(loc_found_3d)
			geo3d_start_id = ca->cubes_[map3d_id].geo_mark_id_;

		// if vehicle is above valid 2d map, connect 2d vertex and 3d vertex
		if(loc_found_2d)
		{
			if(loc_found_3d)
			{
				GeoMark vehicle_2dstart_mark, vehicle_3dstart_mark;

				if(combined_graph_.GetVertexFromID(geo2d_start_id) != nullptr)
					vehicle_2dstart_mark = combined_graph_.GetVertexFromID(geo2d_start_id)->bundled_data_;
				else
				{
					std::cerr << "Fatal error: failed to find 2d start geomark object" << std::endl;
					return -1;
				}

				if(combined_graph_.GetVertexFromID(geo3d_start_id) != nullptr)
					vehicle_3dstart_mark = combined_graph_.GetVertexFromID(geo3d_start_id)->bundled_data_;
				else
				{
					std::cerr << "Fatal error: failed to find 3d start geomark object" << std::endl;
					return -1;
				}

				double cost = std::sqrt(std::pow(vehicle_2dstart_mark.position.x - vehicle_3dstart_mark.position.x, 2) +
						std::pow(vehicle_2dstart_mark.position.y - vehicle_3dstart_mark.position.y, 2) +
						std::pow(vehicle_2dstart_mark.position.z - vehicle_3dstart_mark.position.z, 2));
				combined_graph_.AddEdge(vehicle_2dstart_mark, vehicle_3dstart_mark, cost);

				double dist_2d = std::sqrt(std::pow(vehicle_2dstart_mark.position.x - pos_.x, 2) +
						std::pow(vehicle_2dstart_mark.position.y - pos_.y, 2) +
						std::pow(vehicle_2dstart_mark.position.z - pos_.z, 2));
				double dist_3d = std::sqrt(std::pow(vehicle_3dstart_mark.position.x - pos_.x, 2) +
						std::pow(vehicle_3dstart_mark.position.y - pos_.y, 2) +
						std::pow(vehicle_3dstart_mark.position.z - pos_.z, 2));

				if(dist_2d <= dist_3d)
					final_start_id = geo2d_start_id;
				else
					final_start_id = geo3d_start_id;
			}
			else
				final_start_id = geo2d_start_id;
		}
		else
		{
			final_start_id = geo3d_start_id;
		}
	}

	return final_start_id;
}
