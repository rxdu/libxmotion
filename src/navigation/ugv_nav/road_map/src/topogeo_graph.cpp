/* 
 * topogeo_graph.cpp
 * 
 * Created on: Jul 25, 2018 06:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "road_map/details/topogeo_graph.hpp"

#include <set>
#include <algorithm>

#include "road_map/road_map.hpp"

using namespace librav;

TopoGeoGraph::TopoGeoGraph(RoadMap *map) : road_map_(map)
{
    ConstructGraph();
}

TopoGeoGraph::~TopoGeoGraph()
{
    for (auto &entry : lane_blocks_)
        delete entry.second;
}

void TopoGeoGraph::ConstructGraph()
{
    graph_ = std::make_shared<Graph<LaneBlock *>>();

    auto mapping = road_map_->GetLaneletIDNameMap();
    for (auto &entry : mapping)
    {
        auto new_block = new LaneBlock(entry.first, entry.second);
        new_block->center_line = road_map_->GetLaneCenterLine(entry.second);
        lane_blocks_.insert(std::make_pair(entry.first, new_block));
        graph_->AddVertex(new_block);
    }

    for (auto &ll1 : mapping)
        for (auto &ll2 : mapping)
        {
            if (ll1.first != ll2.first)
            {
                auto route12 = road_map_->FindShortestRoute(ll1.second, ll2.second);
                auto route21 = road_map_->FindShortestRoute(ll2.second, ll1.second);
                // check if direct connection
                if (route12.size() == 2)
                    graph_->AddEdge(lane_blocks_[ll1.first], lane_blocks_[ll2.first], 1.0);
                else if (route21.size() == 2)
                    graph_->AddEdge(lane_blocks_[ll2.first], lane_blocks_[ll1.first], 1.0);
                // otherwise check collision
                else if (road_map_->CheckLaneletCollision(ll1.second, ll2.second))
                {
                    lane_blocks_[ll1.first]->type = LaneBlockType::GeoConnected;
                    lane_blocks_[ll2.first]->type = LaneBlockType::GeoConnected;
                    graph_->AddUndirectedEdge(lane_blocks_[ll1.first], lane_blocks_[ll2.first], 0.0);
                }
            }
        }

    for (auto it = graph_->vertex_begin(); it != graph_->vertex_end(); ++it)
    {
        if (it->vertices_from_.empty() && !it->edges_to_.empty())
            sources_.push_back(it->state_->name);
        if (it->edges_to_.empty() && !it->vertices_from_.empty())
            sinks_.push_back(it->state_->name);
        if (it->edges_to_.empty() && it->vertices_from_.empty())
            isolated_lanes_.push_back(it->state_->name);

        // for (auto eit = it->edge_begin(); eit != it->edge_end(); ++eit)
        //     std::cout << "edge: " << eit->src_->state_->name << " -> " << eit->dst_->state_->name << std::endl;
    }
}

std::vector<std::string> TopoGeoGraph::BacktrackVertices(int32_t id)
{
    using GraphType = Graph<LaneBlock *>;
    auto vtx = graph_->FindVertex(id);

    // std::cout << "-----> back tracking lanelet " << vtx->state_->name << std::endl;

    // reset labels
    for (auto it = graph_->vertex_begin(); it != graph_->vertex_end(); ++it)
        it->state_->type = LaneBlockType::TopoConnected;

    std::vector<GraphType::vertex_iterator> vertices;
    std::vector<GraphType::vertex_iterator> vtx_candidates;
    for (auto vf : vtx->vertices_from_)
    {
        // std::cout << "added name: " << vf->state_->name << std::endl;
        vertices.push_back(vf);

        if (vf->FindEdge(vtx->vertex_id_)->cost_ == 0.0)
            vf->state_->type = LaneBlockType::GeoConnected;
        else
            vtx_candidates.push_back(vf);
    }

    // std::cout << ">>" << std::endl;

    // std::cout << "first level backtrack: " << vtx_candidates.size() << std::endl;
    while (!vtx_candidates.empty())
    {
        // for (auto cv : vtx_candidates)
        //     vertices.push_back(cv);

        std::vector<GraphType::vertex_iterator> candidates = vtx_candidates;
        vtx_candidates.clear();
        for (auto &v : candidates)
        {
            // std::cout << "id: " << v->vertex_id_ << " , num: " << v->vertices_from_.size() << std::endl;
            if (v->state_->type == LaneBlockType::TopoConnected)
            {
                for (auto vf : v->vertices_from_)
                {
                    if (vf->vertex_id_ != id &&
                        std::find(vertices.begin(), vertices.end(), vf) == vertices.end())
                    {
                        // std::cout << "added name: " << vf->state_->name << std::endl;
                        if (vf->FindEdge(v->vertex_id_)->cost_ != 0.0)
                        {
                            vertices.push_back(vf);
                            vtx_candidates.push_back(vf);
                        }
                    }
                }
            }
        }
        // std::cout << "vtx_candidates size: " << vtx_candidates.size() << std::endl;
    }

    std::vector<std::string> names;
    // add checked vertex first
    names.push_back(vtx->state_->name);
    for (auto v : vertices)
    {
        names.push_back(v->state_->name);
        // std::cout << "name: " << v->state_->name << " , id: " << v->state_->id << std::endl;
    }
    // std::cout << "total number: " << names.size() << std::endl;

    // std::cout << "-------------------------" << std::endl;

    return names;
}

std::vector<std::string> TopoGeoGraph::FindConflictingLanes(std::vector<std::string> names)
{
    std::vector<int32_t> ids;
    for (auto &name : names)
        ids.push_back(road_map_->GetLaneletIDFromName(name));
    return FindConflictingLanes(ids);
}

std::vector<std::string> TopoGeoGraph::FindConflictingLanes(std::vector<int32_t> ids)
{
    std::set<std::string> lanes;
    // add lanes that are possible for interactions
    for (auto id : ids)
    {
        auto vts = BacktrackVertices(id);
        for (auto vtname : vts)
            lanes.insert(vtname);
    }

    // save from set to vector
    std::vector<std::string> lane_names;
    for (auto &n : lanes)
        lane_names.push_back(n);

    return lane_names;
}

bool TopoGeoGraph::HasOnlyOneSubsequentLane(std::string name)
{
}
