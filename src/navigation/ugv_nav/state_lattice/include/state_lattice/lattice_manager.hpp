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

#include "state_lattice/motion_primitive.hpp"

namespace librav
{
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
