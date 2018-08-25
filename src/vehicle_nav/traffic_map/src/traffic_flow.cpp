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

    // flow_tree_ = BuildTree(channels_);
}

TrafficFlow::TrafficFlow(std::vector<TrafficChannel> channels) : channels_(channels)
{
    source_ = channels_.front().source;

    flow_tree_ = BuildTree(channels_);
}

std::vector<Polygon> TrafficFlow::GetAllLaneBlocks()
{
    std::vector<Polygon> blks;
    for (auto &chn : channels_)
        blks.insert(blks.end(), chn.lane_blocks.begin(), chn.lane_blocks.end());
    return blks;
    // UniqueTree<librav::FlowUnit> tree = BuildTree(channels_);
    // auto all_units = Traversal::GetAllStatesBF(&flow_tree_);
    // std::vector<Polygon> blks;
    // for (auto &fu : all_units)
    //     blks.push_back(fu.footprint);
    // return blks;
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

void TrafficFlow::CheckConflicts(const std::vector<TrafficFlow> &flows)
{
    // std::cout << "checking conflicts" << std::endl;
    // UniqueTree<librav::FlowUnit> tree = BuildTree(channels_);
    // UniqueTree<librav::FlowUnit> tree = BuildTree(channels_);

    // for (auto &)
}

void TrafficFlow::CheckConflicts(const TrafficFlow &flow)
{
    std::cout << "checking conflicts" << std::endl;
    UniqueTree<librav::FlowUnit> ego_tree = BuildTree(channels_);
    UniqueTree<librav::FlowUnit> other_tree = BuildTree(channels_);

    // for (auto &)
}

void TrafficFlow::CheckCollision(UniqueTree<FlowUnit> *tree, const FlowUnit &other)
{
    UniqueTree<FlowUnit>::NodeType *root = tree->GetRootNode();

    std::queue<UniqueTree<FlowUnit>::NodeType *> q;
    q.push(root);
    while (!q.empty())
    {
        root = q.front();
        q.pop();
        std::cout << root->state << std::endl;
        if (root->state.footprint.Intersect(other.footprint))
        {
            root->state.in_collision = true;
            root->state.time_label = other.time_label;
        }
        for (auto &nd : root->children)
            q.push(nd.first);
    }
}