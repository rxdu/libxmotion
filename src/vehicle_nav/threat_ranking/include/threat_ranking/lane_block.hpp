/* 
 * lane_block.hpp
 * 
 * Created on: Aug 03, 2018 22:30
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LANE_BLOCK_HPP
#define LANE_BLOCK_HPP

#include <string>
#include <cstdint>

#include "geometry/polyline.hpp"

namespace librav
{
enum class LaneBockType
{
    LaneSegment = 0,
    LaneConnector
};

struct LaneBlock
{
    LaneBlock() : id(-1), name("null"){}
    LaneBlock(int32_t _id, std::string _name = "default") : id(_id), name(_name) {}
    LaneBlock(int32_t _id, LaneBockType _type, std::string _name = "default") : id(_id), name(_name), type(_type) {}

    int32_t id;
    std::string name;
    LaneBockType type;
    Polyline center_line;

    int64_t GetUniqueID() const
    {
        return id;
    }

    bool operator==(const LaneBlock &other)
    {
        if (other.id == this->id)
            return true;
        else
            return false;
    }
};
} // namespace librav

#endif /* LANE_BLOCK_HPP */
