/* 
 * traffic_flow.cpp
 * 
 * Created on: Aug 22, 2018 09:38
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_ranking/traffic_flow.hpp"

using namespace librav;

TrafficFlow::TrafficFlow(std::vector<std::vector<std::string>> subflows) : subflows_(subflows)
{
    BuildTree();
}

void TrafficFlow::BuildTree()
{
    for(auto& path : subflows_)
    {
        for(auto& segment : path)
        {

        }
    }
}