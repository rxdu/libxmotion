/* 
 * lattice_manager.hpp
 * 
 * Created on: Aug 07, 2018 04:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LATTICE_MANAGER_HPP
#define LATTICE_MANAGER_HPP

#include <string>
#include <vector>
#include <unordered_map>

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
    MotionPrimitive(int32_t mpid) : id(mpid){};

    int32_t id;
    std::vector<PrimitiveNode> nodes;
    double length;

    PrimitiveNode GetInitNode() { return nodes.front(); }
    PrimitiveNode GetFinalNode() { return nodes.back(); }
};

class LatticeManager
{
  public:
    LatticeManager() = default;

    std::vector<MotionPrimitive> primitives_;

    void LoadPrimitivesFromFile(std::string file);
    void SavePrimitivesToFile(std::vector<MotionPrimitive> mps, std::string file);

    std::vector<MotionPrimitive> TransformAllPrimitives(const std::vector<MotionPrimitive> &input, double x, double y, double theta);

  private:
    std::unordered_map<int32_t, MotionPrimitive> primitive_map_;

    void PostProcessingPrimitives();

    MotionPrimitive TransformPrimitive(const MotionPrimitive &input, double dx, double dy, double dtheta);
    PrimitiveNode TransformNode(const PrimitiveNode &input, double dx, double dy, double dtheta);
};
} // namespace librav

#endif /* LATTICE_MANAGER_HPP */
