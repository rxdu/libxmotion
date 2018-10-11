/* 
 * traffic_flow.cpp
 * 
 * Created on: Aug 22, 2018 09:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_map/traffic_flow.hpp"

#include <queue>

#include "tree/algorithms/traversal.hpp"
#include "tree/algorithms/search.hpp"

using namespace librav;

TrafficFlow::TrafficFlow(TrafficChannel channel)
{
    source_ = channel.source;
    channels_.push_back(channel);

    flow_tree_ = BuildTree(channels_);
}

TrafficFlow::TrafficFlow(std::vector<TrafficChannel> channels) : channels_(channels)
{
    source_ = channels_.front().source;

    flow_tree_ = BuildTree(channels_);
}

void TrafficFlow::AssignConstantSpeedProfile(int32_t start_id, double dt)
{
    if (channels_.size() != 1)
        std::cerr << "Ego traffic flow should only contain one traffic channel" << std::endl;

    int32_t node_index = 0;
    UniqueTree<FlowUnit>::NodeType *node = flow_tree_.GetRootNode();
    while (node != nullptr)
    {
        if (node_index >= start_id)
        {
            node->state.time_stamp = (node_index - start_id) * dt;
        }
        else
        {
            node->state.time_stamp = 0;
        }
        ++node_index;
        if (node->children.empty())
            node = nullptr;
        else
            node = (*node->children.begin()).first;
    }
    std::cout << "constant time profile assigned" << std::endl;
}

std::vector<Polygon> TrafficFlow::GetAllLaneBlocks()
{
    auto all_units = Traversal::GetAllStatesBF(&flow_tree_);
    std::vector<Polygon> blks;
    for (auto &fu : all_units)
        blks.push_back(fu.footprint);
    return blks;
}

std::vector<Polygon> TrafficFlow::GetConflictingLaneBlocks()
{
    auto all_units = Traversal::GetAllStatesBF(&flow_tree_);
    std::vector<Polygon> blks;
    for (auto &fu : all_units)
    {
        if (fu.in_collision)
            blks.push_back(fu.footprint);
    }
    return blks;
}

UniqueTree<FlowUnit> TrafficFlow::BuildTree(const std::vector<TrafficChannel> &chns)
{
    UniqueTree<FlowUnit> tree;
    for (auto &chn : chns)
    {
        for (int32_t i = 0; i < chn.lane_blocks.size() - 1; ++i)
        {
            FlowUnit start(chn.lane_blocks[i], chn.block_poses[i]);
            FlowUnit end(chn.lane_blocks[i + 1], chn.block_poses[i + 1]);

            tree.ConnectNodes(start, end, 1.0);
        }
    }
    return tree;
}

std::vector<FlowTrackPoint> TrafficFlow::BackTrackSingleChannelCollision(TrafficFlow *flow, double v)
{
    std::vector<FlowTrackPoint> bk_poses;
    collision_units_.clear();
    // only when current flow is single-channel flow
    if (channels_.size() != 1)
        return bk_poses;

    // BF traversal
    UniqueTree<FlowUnit>::NodeType *node = flow->flow_tree_.GetRootNode();
    std::queue<UniqueTree<FlowUnit>::NodeType *> q;
    q.push(node);
    while (!q.empty())
    {
        node = q.front();
        q.pop();

        LabelUnitCollision(&node->state);

        if (node->state.in_collision)
        {
            auto cspose = flow->BackTrackFlowUnit(node->state, v, node->state.time_stamp);
            std::cout << "back tracked " << cspose.time_stamp << "s : " << cspose.pose.x << " , " << cspose.pose.y << " , " << cspose.pose.theta << std::endl;
            bk_poses.push_back(cspose);
            collision_units_.push_back(node->state);
            continue;
        }

        for (auto &nd : node->children)
            q.push(nd.first);
    }

    return bk_poses;
}

void TrafficFlow::LabelUnitCollision(FlowUnit *unit)
{
    // BF traversal
    UniqueTree<FlowUnit>::NodeType *node = flow_tree_.GetRootNode();
    std::queue<UniqueTree<FlowUnit>::NodeType *> q;
    q.push(node);
    while (!q.empty())
    {
        node = q.front();
        q.pop();
        if (node->state.footprint.Intersect(unit->footprint))
        {
            unit->in_collision = true;
            unit->time_stamp = node->state.time_stamp;
            // std::cout << "-- collision time label: " << unit->time_stamp << std::endl;
        }
        else
        {
            for (auto &nd : node->children)
                q.push(nd.first);
        }
    }
}

FlowTrackPoint TrafficFlow::BackTrackFlowUnit(const FlowUnit &start, double v, double t)
{
    VehiclePose final_pose;

    double distance = 0.0;
    double travel_dist = v * t;

    auto tree_node = Search::BFS(&flow_tree_, start);
    assert(tree_node != nullptr);

    while (tree_node != nullptr)
    {
        FlowNode *parent_node = tree_node->parent;

        if (parent_node != nullptr)
        {
            double seg_len = tree_node->state.CalculateDistance(parent_node->state);

            if (distance + seg_len < travel_dist)
            {
                distance += seg_len;
            }
            else
            {
                double s = travel_dist - distance;
                final_pose = InterpolatePoseInversed(tree_node->state.pose, parent_node->state.pose, s);
                break;
            }
        }

        tree_node = parent_node;
    }
    return FlowTrackPoint(start.pose, final_pose, t);
}

VehiclePose TrafficFlow::InterpolatePose(VehiclePose pt0, VehiclePose pt1, double s)
{
    Eigen::Vector2d base(pt1.x - pt0.x, pt1.y - pt0.y);
    double base_len = base.dot(base.normalized());
    Eigen::Vector2d ps = s / base_len * base;
    // std::cout << "s/base_len: " << s / base_len << std::endl;
    Eigen::Vector2d dir = base.normalized();

    Eigen::Vector2d position = Eigen::Vector2d(pt0.x, pt0.y) + ps;
    double yaw = std::atan2(dir(1), dir(0));

    return VehiclePose(position(0), position(1), yaw);
}

VehiclePose TrafficFlow::InterpolatePoseInversed(VehiclePose pt0, VehiclePose pt1, double s)
{
    Eigen::Vector2d base(pt1.x - pt0.x, pt1.y - pt0.y);
    double base_len = base.dot(base.normalized());
    Eigen::Vector2d ps = s / base_len * base;
    Eigen::Vector2d dir = -base.normalized();

    Eigen::Vector2d position = Eigen::Vector2d(pt0.x, pt0.y) + ps;
    double yaw = std::atan2(dir(1), dir(0));

    return VehiclePose(position(0), position(1), yaw);
}
