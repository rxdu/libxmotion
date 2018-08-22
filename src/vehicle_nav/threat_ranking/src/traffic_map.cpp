/* 
 * traffic_flow_map.cpp
 * 
 * Created on: Aug 20, 2018 22:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_ranking/traffic_map.hpp"

#include <cmath>

using namespace librav;

TrafficMap::TrafficMap(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    ConstructLaneGraph();
    DecomposeRoadNetwork(1.0);
}

TrafficMap::~TrafficMap()
{
    for (auto &entry : flow_regions_)
        delete entry.second;
}

void TrafficMap::ConstructLaneGraph()
{
    graph_ = std::make_shared<Graph_t<TrafficRegion *>>();

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

                    TrafficRegion *src_bk = new TrafficRegion(src_id, src_name);
                    TrafficRegion *dst_bk = new TrafficRegion(dst_id, dst_name);
                    flow_regions_.insert(std::make_pair(src_id, src_bk));
                    flow_regions_.insert(std::make_pair(dst_id, dst_bk));

                    src_bk->center_line = road_map_->GetLaneCenterLine(src_name);
                    dst_bk->center_line = road_map_->GetLaneCenterLine(dst_name);

                    graph_->AddEdge(src_bk, dst_bk, 1.0);
                }
            }
        }
}

void TrafficMap::DecomposeRoadNetwork(double resolution)
{
    for (auto &source : road_map_->GetSources())
        for (auto &sink : road_map_->GetSinks())
        {
            // std::cout << source << " ---> " << sink << std::endl;
            auto path = road_map_->FindShortestRouteName(source, sink);
            if (!path.empty())
            {
                Polyline line = graph_->GetVertex(road_map_->GetLaneletIDFromName(path.front()))->state_->center_line;
                for (auto it = path.begin() + 1; it != path.end(); ++it)
                    line = line.Concatenate(graph_->GetVertex(road_map_->GetLaneletIDFromName(*it))->state_->center_line);

                double remaining = 0;
                for (int i = 0; i < line.GetPointNumer() - 1; ++i)
                {
                    // int32_t block_index = 0;
                    // int32_t pt_num = 0;
                    // while (block_index < path.size())
                    // {
                    //     pt_num += flow_regions_[road_map_->GetLaneletIDFromName(path[block_index])]->center_line.GetPointNumer();
                    //     if (pt_num > i)
                    //         break;
                    //     ++block_index;
                    // }
                    // std::cout << "segment: " << block_index << " pt index: " << i << std::endl;
                    auto p0 = line.GetPoint(i);
                    auto p1 = line.GetPoint(i + 1);
                    Eigen::Vector2d ref_vec(p1.x - p0.x, p1.y - p0.y);
                    double segment_len = ref_vec.dot(ref_vec.normalized());

                    // add starting position
                    if (i == 0)
                    {
                        VehiclePose new_pose = InterpolatePose(p0, p1, 0);
                        // fps.push_back(fp.TransformRT(new_pose.x, new_pose.y, new_pose.theta));
                    }

                    // add positions between subsequent points
                    double accumulated = 0;
                    if (remaining != 0)
                    {
                        // clear remaining first
                        accumulated = resolution - remaining;
                        VehiclePose new_pose = InterpolatePose(p0, p1, accumulated);
                        // fps.push_back(fp.TransformRT(new_pose.x, new_pose.y, new_pose.theta));
                    }

                    while (accumulated + resolution <= segment_len)
                    {
                        accumulated += resolution;

                        VehiclePose new_pose = InterpolatePose(p0, p1, accumulated);
                        // std::cout << "new pose: " << new_pose.x << " , " << new_pose.y << " , " << new_pose.theta << std::endl;
                        // fps.push_back(fp.TransformRT(new_pose.x, new_pose.y, new_pose.theta));
                    }
                    remaining = segment_len - accumulated;
                }
            }
        }
}

void TrafficMap::BuildTrafficFlowMap()
{
}

std::vector<Polygon> TrafficMap::DecomposeCenterlines(std::vector<std::string> lanelets, double step)
{
    std::vector<Polygon> fps;

    Polygon fp;
    fp.AddPoint(1.2 * 2, 0.9);
    fp.AddPoint(1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, -0.9);
    fp.AddPoint(-1.2 * 2, 0.9);

    Polyline line = graph_->GetVertex(road_map_->GetLaneletIDFromName(lanelets.front()))->state_->center_line;
    for (auto it = lanelets.begin() + 1; it != lanelets.end(); ++it)
        line = line.Concatenate(graph_->GetVertex(road_map_->GetLaneletIDFromName(*it))->state_->center_line);

    double remaining = 0;
    for (int i = 0; i < line.GetPointNumer() - 1; ++i)
    {
        auto p0 = line.GetPoint(i);
        auto p1 = line.GetPoint(i + 1);
        Eigen::Vector2d ref_vec(p1.x - p0.x, p1.y - p0.y);
        double segment_len = ref_vec.dot(ref_vec.normalized());

        // add starting position
        if (i == 0)
        {
            VehiclePose new_pose = InterpolatePose(p0, p1, 0);
            fps.push_back(fp.TransformRT(new_pose.x, new_pose.y, new_pose.theta));
        }

        // add positions between subsequent points
        double accumulated = 0;
        if (remaining != 0)
        {
            // clear remaining first
            accumulated = step - remaining;
            VehiclePose new_pose = InterpolatePose(p0, p1, accumulated);
            fps.push_back(fp.TransformRT(new_pose.x, new_pose.y, new_pose.theta));
        }

        while (accumulated + step <= segment_len)
        {
            accumulated += step;

            VehiclePose new_pose = InterpolatePose(p0, p1, accumulated);
            // std::cout << "new pose: " << new_pose.x << " , " << new_pose.y << " , " << new_pose.theta << std::endl;
            fps.push_back(fp.TransformRT(new_pose.x, new_pose.y, new_pose.theta));
        }
        remaining = segment_len - accumulated;
    }

    return fps;
}

VehiclePose TrafficMap::InterpolatePose(SimplePoint pt0, SimplePoint pt1, double s)
{
    Eigen::Vector2d base(pt1.x - pt0.x, pt1.y - pt0.y);
    double base_len = base.dot(base.normalized());
    Eigen::Vector2d ps = s / base_len * base;
    Eigen::Vector2d dir = base.normalized();

    Eigen::Vector2d position = Eigen::Vector2d(pt0.x, pt0.y) + ps;
    double yaw = std::atan2(dir(1), dir(0));

    return VehiclePose(position(0), position(1), yaw);
}