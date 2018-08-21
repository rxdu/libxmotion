/* 
 * traffic_flow_map.cpp
 * 
 * Created on: Aug 20, 2018 22:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_ranking/traffic_flow_map.hpp"

using namespace librav;

TrafficFlowMap::TrafficFlowMap(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    ConstructLaneGraph();
}

TrafficFlowMap::~TrafficFlowMap()
{
    for (auto &entry : flow_regions_)
        delete entry.second;
}

void TrafficFlowMap::ConstructLaneGraph()
{
    graph_ = std::make_shared<Graph_t<FlowRegion *>>();

    for (auto &source : road_map_->GetSources())
        for (auto &sink : road_map_->GetSinks())
        {
            auto path = road_map_->FindShortestRoute(source, sink);
            if (!path.empty())
            {
                for (auto it = path.begin(); it != path.end() - 1; ++it)
                {
                    int32_t src_id = *it;
                    int32_t dst_id = *(it + 1);

                    std::string src_name = road_map_->GetLaneletNameFromID(src_id);
                    std::string dst_name = road_map_->GetLaneletNameFromID(dst_id);

                    FlowRegion *src_bk = new FlowRegion(src_id, src_name);
                    FlowRegion *dst_bk = new FlowRegion(dst_id, dst_name);
                    flow_regions_.insert(std::make_pair(src_id, src_bk));
                    flow_regions_.insert(std::make_pair(dst_id, dst_bk));

                    src_bk->center_line = road_map_->GetLaneCenterLine(src_name);
                    dst_bk->center_line = road_map_->GetLaneCenterLine(dst_name);

                    graph_->AddEdge(src_bk, dst_bk, 1.0);
                }
            }
        }
}

void TrafficFlowMap::DecomposeCenterlines()
{
    
}