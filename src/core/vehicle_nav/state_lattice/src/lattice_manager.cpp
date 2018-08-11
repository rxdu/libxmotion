/* 
 * lattice_manager.cpp
 * 
 * Created on: Aug 07, 2018 04:36
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/lattice_manager.hpp"

#include <iostream>
#include <cmath>

#include "csv_parser/csv.h"
#include "logging/logger.hpp"

using namespace librav;

void LatticeManager::LoadPrimitivesFromFile(std::string file)
{
    const int32_t entry_size = 9;
    io::CSVReader<entry_size> mpfile(file);

    // mpfile.read_header(io::ignore_extra_column, "vendor", "size", "speed");

    double data_entry[entry_size];
    while (mpfile.read_row(data_entry[0],                                               // index
                           data_entry[1], data_entry[2], data_entry[3], data_entry[4],  // init state
                           data_entry[5], data_entry[6], data_entry[7], data_entry[8])) // output state
    {
        if (primitive_map_.find(data_entry[0]) == primitive_map_.end())
            primitive_map_.insert(std::make_pair(data_entry[0], MotionPrimitive(data_entry[0])));

        PrimitiveNode node(data_entry[5], data_entry[6], data_entry[7], data_entry[8]);
        primitive_map_[data_entry[0]].nodes.push_back(node);
    }

    std::cout << "number of primitives loaded: " << primitive_map_.size() << std::endl;

    PostProcessingPrimitives();
}

void LatticeManager::SavePrimitivesToFile(std::vector<MotionPrimitive> mps, std::string file)
{
    if (mps.empty())
        return;

    PrimitiveNode origin = mps.front().GetInitNode();
    for (const auto &mp : mps)
    {
        for (auto &nd : mp.nodes)
            GlobalCsvLogger::GetLogger(file, "/home/rdu").LogData(mp.id, origin.x, origin.y, origin.v, origin.theta, nd.x, nd.y, nd.v, nd.theta);
    }
    std::cout << "number of primitives written: " << mps.size() << std::endl;
}

void LatticeManager::PostProcessingPrimitives()
{
    for (auto &pr : primitive_map_)
    {
        double len = 0;
        for (std::size_t i = 0; i < pr.second.nodes.size() - 1; ++i)
        {
            len += std::hypot(pr.second.nodes[i].x - pr.second.nodes[i + 1].x,
                              pr.second.nodes[i].y - pr.second.nodes[i + 1].y);
        }
        pr.second.length = len;
        primitives_.push_back(pr.second);
        // std::cout << "primitive " << pr.second.id << " length: " << pr.second.length << std::endl;
    }

    std::cout << "finished post-processing" << std::endl;
}

std::vector<MotionPrimitive> LatticeManager::TransformAllPrimitives(const std::vector<MotionPrimitive> &input, double x, double y, double theta)
{
    std::vector<MotionPrimitive> new_mps;
    for (auto &mp : input)
    {
        auto new_mp = TransformPrimitive(mp, x, y, theta);
        new_mps.push_back(new_mp);
    }
    return new_mps;
}

MotionPrimitive LatticeManager::TransformPrimitive(const MotionPrimitive &input, double dx, double dy, double dtheta)
{
    // copy all data from input first
    MotionPrimitive output = input;

    // clear nodes and replace with transformed ones
    output.nodes.clear();
    for (const auto &nd : input.nodes)
    {
        auto new_nd = TransformNode(nd, dx, dy, dtheta);
        output.nodes.push_back(new_nd);
    }

    return output;
}

PrimitiveNode LatticeManager::TransformNode(const PrimitiveNode &input, double dx, double dy, double dtheta)
{
    PrimitiveNode output;

    output.x = input.x * std::cos(dtheta) - input.y * std::sin(dtheta) + dx;
    output.y = input.x * std::sin(dtheta) + input.y * std::cos(dtheta) + dy;
    output.v = input.v;
    output.theta = input.theta + dtheta;

    return output;
}