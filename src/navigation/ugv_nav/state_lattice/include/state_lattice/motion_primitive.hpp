/* 
 * motion_primitive.hpp
 * 
 * Created on: Aug 11, 2018 02:48
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MOTION_PRIMITIVE_HPP
#define MOTION_PRIMITIVE_HPP

#include <string>
#include <vector>
#include <cstdint>

#include "geometry/polyline.hpp"

namespace librav
{
struct PrimitiveNode
{
    PrimitiveNode() : x(0), y(0), v(0), theta(0) {}
    PrimitiveNode(double _x, double _y, double _v, double _theta) : x(_x), y(_y), v(_v), theta(_theta) {}

    double x;
    double y;
    double v;
    double theta;
};

struct MotionPrimitive
{
    MotionPrimitive() : id(-1) {}
    explicit MotionPrimitive(int32_t mpid) : id(mpid){};

    int32_t id;
    double length;
    std::vector<PrimitiveNode> nodes;

    PrimitiveNode GetInitNode() const { return nodes.front(); }
    PrimitiveNode GetFinalNode() const { return nodes.back(); }
    Polyline ToPolyline() const
    {
        Polyline line;
        for (auto &node : nodes)
            line.AddPoint(node.x, node.y);
        return line;
    }
};
} // namespace librav

#endif /* MOTION_PRIMITIVE_HPP */
