/* 
 * lattice_planner.cpp
 * 
 * Created on: Aug 07, 2018 09:37
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/lattice_planner.hpp"

#include <cmath>

using namespace librav;

int64_t LatticeNode::instance_count = 0;

LatticePath LatticePlanner::Search(LatticeNode start_state, LatticeNode goal_state)
{
    using GraphVertexType = Vertex_t<LatticeNode, MotionPrimitive>;

    // create a new graph with only start and goal vertices
    Graph_t<LatticeNode, MotionPrimitive> graph;
    graph.AddVertex(start_state);
    graph.AddVertex(goal_state);

    GraphVertexType *start_vtx = graph.GetVertex(start_state);
    GraphVertexType *goal_vtx = graph.GetVertex(goal_state);

    // open list - a list of vertices that need to be checked out
    PriorityQueue<GraphVertexType *> openlist;

    // begin with start vertex
    openlist.put(start_vtx, 0);
    start_vtx->is_in_openlist_ = true;
    start_vtx->g_cost_ = 0;

    int i = 0;

    // start search iterations
    bool found_path = false;
    GraphVertexType *current_vertex;
    while (!openlist.empty() && found_path != true)
    {
        current_vertex = openlist.get();
        if (current_vertex->is_checked_)
            continue;

        if (CalculateDistance(current_vertex->state_, goal_vtx->state_) < threshold_)
        {
            std::cout << "final dist error: " << CalculateDistance(current_vertex->state_, goal_vtx->state_) << std::endl;
            std::cout << "current node: " << current_vertex->state_.x << " , "
                      << current_vertex->state_.y << " , "
                      << current_vertex->state_.theta << std::endl;
            goal_vtx = current_vertex;

            // found_path = true;
            // break;
        }

        if (current_vertex == goal_vtx)
        {
            found_path = true;
            break;
        }

        // std::cout << "checking ... " << std::endl;
        current_vertex->is_in_openlist_ = false;
        current_vertex->is_checked_ = true;

        std::vector<std::tuple<LatticeNode, MotionPrimitive>> neighbours = GenerateLattices(current_vertex->state_);
        for (auto &nb : neighbours)
        {
            graph.AddEdge(current_vertex->state_, std::get<0>(nb), std::get<1>(nb));
            // std::cout << "state id: " << std::get<0>(nb).id << std::endl;
        }
        // std::cout << "number of edges_to_ : " << current_vertex->edges_to_.size() << std::endl;

        // check all adjacent vertices (successors of current vertex)
        for (auto &edge : current_vertex->edges_to_)
        {
            auto successor = edge.dst_;

            // check if the vertex has been checked (in closed list)
            if (successor->is_checked_ == false)
            {
                // first set the parent of the adjacent vertex to be the current vertex
                auto new_cost = current_vertex->g_cost_ + edge.cost_.length;
                // std::cout << "new cost: " << current_vertex->g_cost_ << " + " << edge.cost_.length
                //           << " = " << new_cost << std::endl;

                // if the vertex is not in open list
                // or if the vertex is in open list but has a higher cost
                if (successor->is_in_openlist_ == false || new_cost < successor->g_cost_)
                {
                    successor->search_parent_ = current_vertex;
                    successor->g_cost_ = new_cost;

                    openlist.put(successor, successor->g_cost_);
                    successor->is_in_openlist_ = true;
                }
            }
        }
    }

    // reconstruct path from search
    LatticePath path;
    if (found_path)
    {
        std::cout << "path found with cost " << goal_vtx->g_cost_ << std::endl;
        auto path_vtx = ReconstructPath(start_vtx, goal_vtx);
        for (int i = 0; i < path_vtx.size() - 1; ++i)
            path.push_back(path_vtx[i]->GetEdgeCost(path_vtx[i + 1]));
    }
    else
        std::cout << "failed to find a path" << std::endl;

    return path;
};

LatticePath LatticePlanner::AStarSearch(LatticeNode start_state, LatticeNode goal_state)
{
    using GraphVertexType = Vertex_t<LatticeNode, MotionPrimitive>;

    // create a new graph with only start and goal vertices
    Graph_t<LatticeNode, MotionPrimitive> graph;
    graph.AddVertex(start_state);
    graph.AddVertex(goal_state);

    GraphVertexType *start_vtx = graph.GetVertex(start_state);
    GraphVertexType *goal_vtx = graph.GetVertex(goal_state);

    // open list - a list of vertices that need to be checked out
    PriorityQueue<GraphVertexType *> openlist;

    // begin with start vertex
    openlist.put(start_vtx, 0);
    start_vtx->is_in_openlist_ = true;
    start_vtx->g_cost_ = 0;

    int i = 0;

    // start search iterations
    bool found_path = false;
    GraphVertexType *current_vertex;
    while (!openlist.empty() && found_path != true && openlist.size() < 1000000UL)
    {
        current_vertex = openlist.get();
        if (current_vertex->is_checked_)
            continue;

        if (CalculateDistance(current_vertex->state_, goal_vtx->state_) < threshold_)
        {
            std::cout << "final distance error: " << CalculateDistance(current_vertex->state_, goal_vtx->state_) << std::endl;
            std::cout << "final state: " << current_vertex->state_.x << " , "
                      << current_vertex->state_.y << " , "
                      << current_vertex->state_.theta << std::endl;
            goal_vtx = current_vertex;

            found_path = true;
            break;
        }

        // std::cout << "checking ... " << std::endl;
        current_vertex->is_in_openlist_ = false;
        current_vertex->is_checked_ = true;

        std::vector<std::tuple<LatticeNode, MotionPrimitive>> neighbours = GenerateLattices(current_vertex->state_);
        for (auto &nb : neighbours)
        {
            // check collision
            // auto grid_pos = road_map_->coordinate_.ConvertToGridPixel(CartCooridnate(std::get<0>(nb).x, std::get<0>(nb).y));
            // std::cout << "checking pos: " << std::get<0>(nb).x << " , " << std::get<0>(nb).y << " , grid pos: " << grid_pos.x << " , " << grid_pos.y << std::endl;

            // only check end point
            // if (drivable_mask_->GetValueAtCoordinate(grid_pos.x, grid_pos.y) != 0)
            //     continue;

            if (road_map_ != nullptr)
            {
                bool occupied = false;
                // check waypoints
                // for (auto &nd : std::get<1>(nb).nodes)
                for (int i = 0; i < std::get<1>(nb).nodes.size(); i = i + 2)
                {
                    auto nd = std::get<1>(nb).nodes[i];
                    auto grid_pos = road_map_->coordinate_.ConvertToGridPixel(CartCooridnate(nd.x, nd.y));
                    if (drivable_mask_->GetValueAtCoordinate(grid_pos.x, grid_pos.y) != 0)
                    {
                        occupied = true;
                        break;
                    }
                }

                if (!occupied)
                    graph.AddEdge(current_vertex->state_, std::get<0>(nb), std::get<1>(nb));
            }
            else
            {
                graph.AddEdge(current_vertex->state_, std::get<0>(nb), std::get<1>(nb));
            }
            // std::cout << "state id: " << std::get<0>(nb).id << std::endl;
        }
        // std::cout << "number of edges_to_ : " << current_vertex->edges_to_.size() << std::endl;

        // check all adjacent vertices (successors of current vertex)
        for (auto &edge : current_vertex->edges_to_)
        {
            auto successor = edge.dst_;

            // check if the vertex has been checked (in closed list)
            if (successor->is_checked_ == false)
            {
                // first set the parent of the adjacent vertex to be the current vertex
                auto new_cost = current_vertex->g_cost_ + edge.cost_.length;
                // std::cout << "new cost: " << current_vertex->g_cost_ << " + " << edge.cost_.length
                //           << " = " << new_cost << std::endl;

                // if the vertex is not in open list
                // or if the vertex is in open list but has a higher cost
                if (successor->is_in_openlist_ == false || new_cost < successor->g_cost_)
                {
                    successor->search_parent_ = current_vertex;

                    // update costs
                    successor->g_cost_ = new_cost;
                    successor->h_cost_ = CalculateHeuristic(successor->state_, goal_vtx->state_);
                    successor->f_cost_ = successor->g_cost_ + successor->h_cost_;

                    openlist.put(successor, successor->f_cost_);
                    successor->is_in_openlist_ = true;
                }
            }
        }
    }

    // reconstruct path from search
    LatticePath path;
    if (found_path)
    {
        std::cout << "path found with cost " << goal_vtx->g_cost_ << std::endl;
        auto path_vtx = ReconstructPath(start_vtx, goal_vtx);
        for (int i = 0; i < path_vtx.size() - 1; ++i)
            path.push_back(path_vtx[i]->GetEdgeCost(path_vtx[i + 1]));
    }
    else
        std::cout << "failed to find a path" << std::endl;

    return path;
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
        LatticeNode lnode(mp.GetFinalNode().x, mp.GetFinalNode().y, mp.GetFinalNode().theta);
        neighbours.push_back(std::make_pair(lnode, mp));
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
