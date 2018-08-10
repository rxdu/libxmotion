/* 
 * lattice_planner.cpp
 * 
 * Created on: Aug 07, 2018 09:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "lattice_planner/lattice_planner.hpp"

#include <cmath>
#include <tuple>
#include <queue>
#include <functional>
#include <utility>
#include <algorithm>
#include <type_traits>
#include <functional>
#include <iostream>

using namespace librav;

int64_t LatticeNode::instance_count = 0;

void LatticePlanner::SetEgoPlanningRoute(std::vector<std::string> ll)
{
    drivable_lanelets_ = ll;
    for (auto name : ll)
        drivable_polygons_.push_back(road_map_->GetLanePolygon(name));
}

LatticePath LatticePlanner::Search(LatticeNode start_state, int32_t horizon)
{
    using GraphType = Graph_t<LatticeNode, MotionPrimitive>;
    using GraphVertexType = Vertex_t<LatticeNode, MotionPrimitive>;

    auto graph = std::make_shared<GraphType>();
    std::vector<MotionPrimitive> mps;

    LatticeNode node(start_state.x, start_state.y, start_state.theta);
    graph->AddVertex(node);
    auto vtx = graph->GetVertex(node.id);

    std::vector<GraphType::VertexType *> last_level;
    last_level.push_back(vtx);

    for (size_t i = 0; i < horizon; ++i)
    {
        std::vector<GraphType::VertexType *> this_level;
        for (auto vtx : last_level)
        {
            auto neighbours = GenerateLattices(vtx->state_);
            std::cout << "neighbour size: " << neighbours.size() << std::endl;
            for (auto &nb : neighbours)
            {
                graph->AddEdge(vtx->state_, std::get<0>(nb), std::get<1>(nb));

                this_level.push_back(graph->GetVertex(std::get<0>(nb).id));
                mps.push_back(std::get<1>(nb));
            }
        }
        last_level = this_level;
        std::cout << "time horizon " << i << " added edge number: " << this_level.size() << std::endl;
    }

    if (!mps.empty())
        lattice_manager_->SavePrimitivesToFile(mps, "graph");

    // // reconstruct path from search
    LatticePath path;
    // if (found_path)
    // {
    //     std::cout << "path found with cost " << goal_vtx->g_cost_ << std::endl;
    //     auto path_vtx = ReconstructPath(start_vtx, goal_vtx);
    //     for (int i = 0; i < path_vtx.size() - 1; ++i)
    //         path.push_back(path_vtx[i]->GetEdgeCost(path_vtx[i + 1]));
    // }
    // else
    //     std::cout << "failed to find a path" << std::endl;

    return path;
};

bool LatticePlanner::VehicleInsideDrivableArea(const Polygon &footprint)
{
    for (auto &polygon : drivable_polygons_)
    {
        // at least one polygon contains given footprint
        if (polygon.Contain(footprint))
            return true;
    }
    return false;
}

bool LatticePlanner::IsCollisionFree(const MotionPrimitive &lattice)
{
    // check waypoints
    for (auto &nd : lattice.nodes)
    // for (int i = 0; i < lattice.nodes.size(); i = i + 2)
    {
        // auto nd = lattice.nodes[i];
        auto fp = footprint_.TransformRT(nd.x, nd.y, nd.theta);
        // std::cout << "checking vehicle state: " << nd.x << " , " <<  nd.y << " , " <<  nd.theta << std::endl;

        if (!VehicleInsideDrivableArea(fp))
            return false;
    }

    return true;
}

std::vector<std::tuple<LatticeNode, MotionPrimitive>> LatticePlanner::GenerateLattices(LatticeNode node)
{
    PrimitiveNode new_base(node.x, node.y, 0, node.theta);
    std::vector<MotionPrimitive> new_mps = lattice_manager_->TransformAllPrimitives(lattice_manager_->primitives_,
                                                                                    new_base.x,
                                                                                    new_base.y,
                                                                                    new_base.theta);
    std::vector<std::tuple<LatticeNode, MotionPrimitive>> neighbours;
    for (auto &mp : new_mps)
    {
        if (IsCollisionFree(mp))
        {
            LatticeNode lnode(mp.GetFinalNode().x, mp.GetFinalNode().y, mp.GetFinalNode().theta);
            neighbours.push_back(std::make_pair(lnode, mp));
        }
    }
    return neighbours;
}

double LatticePlanner::CalculateDistance(LatticeNode node0, LatticeNode node1)
{
    double x_err = node0.x - node1.x;
    double y_err = node0.y - node1.y;
    double theta_err = (node0.theta - node1.theta) * 10;
    return std::sqrt(x_err * x_err + y_err * y_err + theta_err * theta_err);
}

double LatticePlanner::CalculateHeuristic(LatticeNode node0, LatticeNode node1)
{
    double x_err = node0.x - node1.x;
    double y_err = node0.y - node1.y;
    double theta_err = node0.theta - node1.theta;
    return std::sqrt(x_err * x_err + y_err * y_err + theta_err * theta_err);

    // return std::hypot(node0.x - node1.x, node0.y - node1.y);
}

std::vector<Vertex_t<LatticeNode, MotionPrimitive> *> LatticePlanner::ReconstructPath(Vertex_t<LatticeNode, MotionPrimitive> *start_vtx, Vertex_t<LatticeNode, MotionPrimitive> *goal_vtx)
{
    std::vector<Vertex_t<LatticeNode, MotionPrimitive> *> path;

    Vertex_t<LatticeNode, MotionPrimitive> *waypoint = goal_vtx;
    while (waypoint != start_vtx)
    {
        path.push_back(waypoint);
        waypoint = waypoint->search_parent_;
    }
    // add the start node
    path.push_back(waypoint);
    std::reverse(path.begin(), path.end());

    auto traj_s = path.begin();
    auto traj_e = path.end() - 1;
    std::cout << "starting vertex id: " << (*traj_s)->vertex_id_ << std::endl;
    std::cout << "finishing vertex id: " << (*traj_e)->vertex_id_ << std::endl;
    std::cout << "path length: " << path.size() << std::endl;
    std::cout << "total cost: " << path.back()->g_cost_ << std::endl;

    return path;
}
