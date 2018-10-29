/* 
 * traffic_elements.hpp
 * 
 * Created on: Aug 23, 2018 10:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_ELEMENTS_HPP
#define TRAFFIC_ELEMENTS_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "geometry/polyline.hpp"

namespace librav
{
struct VehiclePose
{
    VehiclePose() : x(0), y(0), theta(0) {}
    VehiclePose(double _x, double _y, double _theta) : x(_x), y(_y), theta(_theta) {}

    double x;
    double y;
    double theta;
};

struct TrafficRegion
{
    TrafficRegion() : id(-1), name("null") {}
    explicit TrafficRegion(int32_t _id, std::string _name = "default") : id(_id), name(_name) {}

    int32_t id;
    std::string name;
    Polyline center_line;

    bool discretized = false;

    double remainder = 0.0;
    std::vector<VehiclePose> anchor_points;
    std::vector<Polygon> lane_blocks;

    std::map<std::string, std::vector<VehiclePose>> discrete_anchor_points;
    std::map<std::string, std::vector<Polygon>> discrete_lane_blocks;
    std::map<std::string, double> discrete_remainders;

    bool IsDiscretized(std::string last_region)
    {
        if (discrete_anchor_points.find(last_region) != discrete_anchor_points.end())
            return true;
        else
            return false;
    }

    int64_t GetUniqueID() const
    {
        return id;
    }

    bool operator==(const TrafficRegion &other)
    {
        if (other.id == this->id)
            return true;
        else
            return false;
    }
};

struct TrafficChannel
{
    TrafficChannel(std::string src, std::string dst, std::vector<TrafficRegion *> rgs) : source(src), sink(dst), regions(rgs)
    {
        for (auto region : regions)
            center_line = center_line.SeriesConcatenate(region->center_line);
    }

    TrafficChannel(std::string src, std::string dst, std::vector<TrafficRegion *> rgs, std::vector<Polygon> lbs) : source(src), sink(dst), regions(rgs), lane_blocks(lbs)
    {
        for (auto region : regions)
            center_line = center_line.SeriesConcatenate(region->center_line);
    }

    TrafficChannel(std::string src, std::string dst, std::vector<TrafficRegion *> rgs, std::vector<Polygon> lbs, std::vector<VehiclePose> poses) : source(src), sink(dst), regions(rgs), lane_blocks(lbs), block_poses(poses)
    {
        for (auto region : regions)
            center_line = center_line.SeriesConcatenate(region->center_line);
    }

    std::string source;
    std::string sink;

    std::vector<TrafficRegion *> regions;
    Polyline center_line;
    std::vector<Polygon> lane_blocks;
    std::vector<VehiclePose> block_poses;
};
} // namespace librav

#endif /* TRAFFIC_ELEMENTS_HPP */
