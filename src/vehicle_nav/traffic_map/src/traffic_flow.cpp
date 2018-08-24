/* 
 * traffic_flow.cpp
 * 
 * Created on: Aug 22, 2018 09:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_map/traffic_flow.hpp"

using namespace librav;

TrafficFlow::TrafficFlow(std::string src, std::vector<TrafficChannel> channels) : source_(src), channels_(channels)
{
}

std::vector<Polygon> TrafficFlow::GetAllLaneBlocks() const
{
    std::vector<Polygon> blks;
    for (auto &chn : channels_)
        blks.insert(blks.end(), chn.lane_blocks.begin(), chn.lane_blocks.end());
    return blks;
}

void TrafficFlow::CheckConflicts(const TrafficChannel &other)
{
}

UniqueTree<FlowUnit>  TrafficFlow::BuildTree()
{
    // for (auto &chn : channels_)
    // {
    //     auto blocks = chn.lane_blocks;
    //     for (auto it = blocks.begin(); it != blocks.end() - 1; ++it)
    //     {
    //         FlowUnit *start = new FlowUnit(*it);
    //         FlowUnit *end = new FlowUnit(*(it + 1));

    //         if (root_ == nullptr)
    //             root_ = start;
    //         start->children.push_back(end);
    //     }
    // }
}