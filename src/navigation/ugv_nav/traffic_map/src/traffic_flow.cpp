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

std::vector<Polygon> TrafficFlow::GetAllLaneBlocks()
{
    // std::vector<Polygon> blks;
    // for (auto &chn : channels_)
    //     blks.insert(blks.end(), chn.lane_blocks.begin(), chn.lane_blocks.end());
    // return blks;
    // UniqueTree<librav::FlowUnit> tree = BuildTree(channels_);
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
        auto blocks = chn.lane_blocks;
        for (auto it = blocks.begin(); it != blocks.end() - 1; ++it)
        {
            FlowUnit start(*it);
            FlowUnit end(*(it + 1));

            tree.ConnectNodes(start, end, 1.0);
        }
    }
    return tree;
}

std::vector<TrafficFlow> TrafficFlow::CheckConflicts(const std::vector<TrafficFlow> &flows)
{
    std::vector<TrafficFlow> labeled = flows;
    for (auto &flow : labeled)
    {
        LabelConflictBlocks(&flow);
    }
    return labeled;
}

void TrafficFlow::LabelConflictBlocks(TrafficFlow *flow)
{
    UniqueTree<FlowUnit> &ego_tree = flow_tree_;
    UniqueTree<FlowUnit> &other_tree = flow->flow_tree_;

    UniqueTree<FlowUnit>::NodeType *node = ego_tree.GetRootNode();

    std::queue<UniqueTree<FlowUnit>::NodeType *> q;
    q.push(node);
    while (!q.empty())
    {
        node = q.front();
        q.pop();
        // std::cout << node->state << std::endl;
        CheckCollision(&other_tree, node->state);
        for (auto &nd : node->children)
            q.push(nd.first);
    }
}

void TrafficFlow::CheckCollision(UniqueTree<FlowUnit> *tree, const FlowUnit &other)
{
    UniqueTree<FlowUnit>::NodeType *node = tree->GetRootNode();

    // BF traversal
    std::queue<UniqueTree<FlowUnit>::NodeType *> q;
    q.push(node);
    while (!q.empty())
    {
        node = q.front();
        q.pop();
        if (node->state.footprint.Intersect(other.footprint))
        {
            node->state.in_collision = true;
            node->state.time_label = other.time_label;
        }
        // else
        // {
            for (auto &nd : node->children)
                q.push(nd.first);
        // }
    }
}