/* 
 * lattice_node.hpp
 * 
 * Created on: Aug 08, 2018 23:48
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_NODE_HPP
#define LATTICE_NODE_HPP

#include <cstdint>

namespace librav
{
// Similar to class PrimitiveNode with additional support for graph
struct LatticeNode
{
    LatticeNode(double _x, double _y, double _theta)
        : x(_x), y(_y), theta(_theta) { id = (++LatticeNode::instance_count); }
    LatticeNode() { id = (++LatticeNode::instance_count); }

    static int64_t instance_count;

    int64_t id;
    double x = 0;
    double y = 0;
    // double v = 0;
    double theta = 0;

    int64_t GetUniqueID() const { return id; }
};
} // namespace librav

#endif /* LATTICE_NODE_HPP */
