/* 
 * lookup_table.hpp
 * 
 * Created on: Oct 25, 2018 23:20
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LOOKUP_TABLE_HPP
#define LOOKUP_TABLE_HPP

#include <string>
#include <vector>
#include <cstdint>

#include "state_lattice/details/motion_state.hpp"
#include "state_lattice/details/point_kinematics.hpp"
#include "state_lattice/motion_primitive.hpp"

namespace robosw
{
class LookupTable
{
    struct TableEntry
    {
        TableEntry() = default;
        TableEntry(MotionState _target, PointKinematics::Param _p) : target(_target), p(_p) {}

        MotionState target;
        PointKinematics::Param p;
    };

  public:
    LookupTable() = default;
    explicit LookupTable(std::string filename);

    LookupTable(const LookupTable &other) = default;
    LookupTable &operator=(const LookupTable &other) = default;
    LookupTable(LookupTable &&other) = default;
    LookupTable &operator=(LookupTable &&other) = default;

    // lookup table generation
    void GenerateLookupTable(bool save_to_file = false, std::string filename = "lookup");
    void SaveLookupTableToFile(std::string filename);

    // lookup table loading and query
    void LoadLookupTableFromFile(std::string filename);
    TableEntry SearchNearest(MotionState target);

    // visualization
    std::vector<MotionPrimitive> GetAllSeedPrimitives();

  private:
    std::vector<TableEntry> entries_;

    // parameters that controls the states in the lookup table
    static constexpr double max_s_ = 30.0;
    static constexpr double s_interval_ = 5.0;
    static constexpr double max_delta_ = 5.0;
    static constexpr double delta_interval_ = 1.0;

    std::vector<MotionState> GenerateStateList();
};
} // namespace robosw

#endif /* LOOKUP_TABLE_HPP */
