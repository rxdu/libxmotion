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

std::vector<FlowUnit> TrafficFlow::CheckSingleChannelCollision(TrafficFlow *flow)
{
    std::vector<FlowUnit> cnodes;

    // only when current flow is single-channel flow
    if (channels_.size() != 1)
        return cnodes;

    UniqueTree<FlowUnit>::NodeType *node = flow_tree_.GetRootNode();
    while (node != nullptr)
    {
        auto nds = CheckUnitCollision(flow, node->state);
        cnodes.insert(cnodes.end(), nds.begin(), nds.end());

        if (node->children.empty())
            node = nullptr;
        else
            node = (*node->children.begin()).first;
    }

    std::cout << "collision number: " << cnodes.size() << std::endl;

    return cnodes;
}

std::vector<FlowUnit> TrafficFlow::CheckUnitCollision(TrafficFlow *flow, const FlowUnit &unit)
{
    std::vector<FlowUnit> cnodes;

    UniqueTree<FlowUnit>::NodeType *node = flow->flow_tree_.GetRootNode();

    // BF traversal
    std::queue<UniqueTree<FlowUnit>::NodeType *> q;
    q.push(node);
    while (!q.empty())
    {
        node = q.front();
        q.pop();
        if (node->state.footprint.Intersect(unit.footprint))
        {
            node->state.in_collision = true;
            node->state.time_label = unit.time_label;
            cnodes.push_back(node->state);
        }
        else
        {
            for (auto &nd : node->children)
                q.push(nd.first);
        }
    }

    return cnodes;
}