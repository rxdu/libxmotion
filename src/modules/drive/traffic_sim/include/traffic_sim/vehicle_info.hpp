/* 
 * vehicle_info.hpp
 * 
 * Created on: Dec 06, 2018 21:41
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef VEHICLE_INFO_HPP
#define VEHICLE_INFO_HPP

#include <string>
#include <cstdint>
#include <atomic>
#include <utility>
#include <memory>

#include "traffic_map/traffic_channel.hpp"

namespace autodrive
{
enum class DriveMode
{
    ConstantSpeed = 0,
    UserDefined
};

struct VehicleInfo
{
    VehicleInfo() = default;

    VehicleInfo(double init_distance) : init_s(init_distance)
    {
        id = VehicleInfo::count;
        VehicleInfo::count.fetch_add(1);
    }

    VehicleInfo(std::pair<std::string, std::string> chn, double init_distance) : channel_name(chn), init_s(init_distance)
    {
        id = VehicleInfo::count;
        VehicleInfo::count.fetch_add(1);
    }

    VehicleInfo(std::pair<std::string, std::string> chn, double init_distance, double speed) : channel_name(chn), init_s(init_distance), init_speed(speed)
    {
        id = VehicleInfo::count;
        VehicleInfo::count.fetch_add(1);
    }

    VehicleInfo(std::pair<std::string, std::string> chn, double init_distance, double speed, double start_time) : channel_name(chn), init_s(init_distance), init_t(start_time), init_speed(speed)
    {
        id = VehicleInfo::count;
        VehicleInfo::count.fetch_add(1);
    }

    // id of ego vehicle reserved to be -1
    int32_t id = -1;

    // information to be set by users
    std::pair<std::string, std::string> channel_name;
    double init_s = 0.0;
    double init_t = 0.0;
    double init_speed = 0.0;
    DriveMode drive_mode = DriveMode::ConstantSpeed;

    // valid if "init_s" at "channel" exists
    bool valid = false;
    std::shared_ptr<TrafficChannel> channel;

    // used to generate id automatically
    static std::atomic<int32_t> count;
};
} // namespace autodrive

#endif /* VEHICLE_INFO_HPP */
