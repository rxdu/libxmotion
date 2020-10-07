/* 
 * ctrl_types.hpp
 * 
 * Created on: Oct 10, 2018 11:50
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef CTRL_TYPES_HPP
#define CTRL_TYPES_HPP

#include <cstdint>
#include <vector>
#include <iostream>

#include "adtypes/types/base_types.hpp"

namespace autodrive
{
struct EulerAngle
{
    double roll;
    double pitch;
    double yaw;
};

struct Quaternion
{
    Quaternion() = default;
    Quaternion(double _x, double _y, double _z, double _w) : x(_x), y(_y), z(_z), w(_w) {}

    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

struct Pose3f
{
    Point3f pos;
    EulerAngle ori;
};

template<typename T>
struct Pose2
{
    Pose2() = default;
    Pose2(T _x, T _y, T _theta, TimeStamp ts = 0) : position({_x, _y}), theta(_theta), t(ts) {}

    TimeStamp t;
    value2<T> position;
    T theta;
};

using Pose2f = Pose2<float>;
using Pose2d = Pose2<double>;

// struct Pose2d
// {
//     Pose2d() = default;
//     Pose2d(double _x, double _y, double _theta, TimeStamp ts = 0) : position({_x, _y}), theta(_theta), t(ts) {}

//     TimeStamp t;
//     Point2d position;
//     double theta;
// };

struct Pose3d
{
    Pose3d() = default;
    Pose3d(double _x, double _y, double _z, TimeStamp ts = 0) : position({_x, _y, _z}), t(ts) {}
    Pose3d(double _x, double _y, double _z,
           double _qx, double _qy, double _qz, double _qw,
           TimeStamp ts = 0) : position({_x, _y, _z}), orientation({_qx, _qy, _qz, _qw}), t(ts) {}

    TimeStamp t;
    Point3d position;
    Quaternion orientation;
};

using Position2d = Point2d;
using Position2f = Point2f;
using Position2i = Point2i;

struct Speed
{
    Speed() : mtime(0),
              speed(0.0){};

    Speed(int64_t time, float spd) : mtime(time),
                                     speed(spd){};

    TimeStamp mtime;
    float speed;

    friend std::ostream &operator<<(std::ostream &os, const Speed &data)
    {
        os << "time_stamp: " << data.mtime << " ; speed: " << data.speed << std::endl;
        return os;
    }
};

struct UAVTrajectoryPoint
{
    bool point_empty;
    float positions[3];
    float velocities[3];
    float accelerations[3];
    float jerks[3];
    float yaw;
    float yaw_rate;
    uint64_t duration; // in milliseconds
};

using UAVTrajectory = std::vector<UAVTrajectoryPoint>;
} // namespace autodrive

#endif /* CTRL_TYPES_HPP */
