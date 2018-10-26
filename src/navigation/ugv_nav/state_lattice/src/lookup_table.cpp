/* 
 * lookup_table.cpp
 * 
 * Created on: Oct 25, 2018 23:18
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/details/lookup_table.hpp"

#include <numeric>

#include "state_lattice/primitive_generator.hpp"

#include "logging/logger.hpp"
#include "csv_parser/csv.h"

using namespace librav;

LookupTable::LookupTable(std::string filename)
{
    LoadLookupTableFromFile(filename);
}

std::vector<MotionState> LookupTable::GenerateStateList()
{
    std::vector<MotionState> states;

    std::vector<double> posx, posy, theta, kappa;
    int32_t s_limit = static_cast<int32_t>(max_s_ / s_interval_);
    for (int32_t i = 1; i < s_limit; ++i)
        posx.push_back(s_interval_ * i);

    int32_t d_limit = static_cast<int32_t>(max_delta_ / delta_interval_);
    for (int32_t i = -d_limit; i <= d_limit; ++i)
    {
        if (i == 0)
            continue;
        posy.push_back(delta_interval_ * i);
    }
    std::cout << "s limit: " << s_limit << " , delta limit: " << d_limit << std::endl;

    theta.push_back(0);
    kappa.push_back(0);

    for (auto &x : posx)
        for (auto &y : posy)
            for (auto &t : theta)
                for (auto &k : kappa)
                    states.emplace_back(x, y, t, k);

    // std::cout << "number of states generated: " << states.size() << std::endl;
    return states;
}

void LookupTable::GenerateLookupTable(bool save_to_file, std::string filename)
{
    // add known straight seed paths
    double s_step = s_interval_ / 2;
    int32_t s_limit = static_cast<int32_t>(max_s_ / s_interval_) * 2;
    for (int32_t i = 1; i <= s_limit; ++i)
        entries_.push_back(TableEntry(MotionState(s_step * i, 0, 0, 0), PointKinematics::Param(0.0, 0.0, 0.0, 0, s_step * i)));

    // generate lookup table
    std::vector<MotionState> states = GenerateStateList();

    PrimitiveGenerator gen;
    MotionPrimitive mp;
    MotionState start(0, 0, 0, 0);
    for (auto &state : states)
    {
        auto init = SearchNearest(state);
        bool result = gen.Calculate(start, state, init.p, mp);
        if (result)
            entries_.push_back(TableEntry(mp.GetFinalState(), mp.GetParameters()));
    }

    std::cout << "number of seed states: " << states.size() << ", added table entries: " << entries_.size() - s_limit << std::endl;

    if (save_to_file)
        SaveLookupTableToFile(filename);
}

void LookupTable::SaveLookupTableToFile(std::string filename)
{
    std::string location = "/home/rdu";
    CsvLogger logger(filename, location);

    for (auto &entry : entries_)
    {
        // data: x, y, theta, kappa, p0, p1, p2, p3, sf
        logger.LogData(entry.target.x, entry.target.y, entry.target.theta, entry.target.kappa,
                       entry.p.p0, entry.p.p1, entry.p.p2, entry.p.p3, entry.p.sf);
    }

    std::cout << "lookup table saved to: " << location + "/" + filename << std::endl;
}

void LookupTable::LoadLookupTableFromFile(std::string filename)
{
    std::cout << "read lookup table from file: " << filename << std::endl;

    const int32_t entry_size = 9;
    io::CSVReader<entry_size> mpfile(filename);

    // clear existing table entries
    entries_.clear();

    // load entries from file
    double data_entry[entry_size];
    while (mpfile.read_row(data_entry[0], data_entry[1], data_entry[2], data_entry[3],                 // target state
                           data_entry[4], data_entry[5], data_entry[6], data_entry[7], data_entry[8])) // spriral param
    {
        entries_.emplace_back(TableEntry(MotionState(data_entry[0], data_entry[1], data_entry[2], data_entry[3]),
                                         PointKinematics::Param(data_entry[4], data_entry[5], data_entry[6], data_entry[7], data_entry[8])));
    }

    std::cout << "number of table entries loaded: " << entries_.size() << std::endl;
}

LookupTable::TableEntry LookupTable::SearchNearest(MotionState target)
{
    double min_cost = std::numeric_limits<double>::max();
    TableEntry selected_entry;

    for (auto &entry : entries_)
    {
        double dx = target.x - entry.target.x;
        double dy = target.y - entry.target.y;
        double dtheta = target.theta - entry.target.theta;
        double dkappa = target.kappa - entry.target.kappa;
        double cost = std::sqrt(dx * dx + dy * dy + dtheta * dtheta + dkappa * dkappa);

        if (cost <= min_cost)
        {
            min_cost = cost;
            selected_entry = entry;
        }
    }

    return selected_entry;
}

std::vector<MotionPrimitive> LookupTable::GetAllSeedPrimitives()
{
    std::vector<MotionPrimitive> mps;
    MotionState start(0, 0, 0, 0);

    for (auto &entry : entries_)
        mps.push_back(MotionPrimitive(start, entry.target, entry.p));

    return mps;
}