/* 
 * lattice_graph.cpp
 * 
 * Created on: Aug 08, 2018 23:44
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "state_lattice/lattice_graph.hpp"

#include <queue>

using namespace librav;

LatticeGraph::LatticeGraph()
{
    lattice_manager_ = std::make_shared<LatticeManager>();
}

void LatticeGraph::LoadMotionPrimitives(std::string file)
{
    lattice_manager_->LoadPrimitivesFromFile("/home/rdu/Workspace/librav/data/lattice/primitives/mp.three-level.data");
}

void LatticeGraph::GenerateGraph(int32_t n_unit_time)
{
    graph_ = std::make_shared<GraphType>();
    std::vector<MotionPrimitive> mps;

    LatticeNode node(0, 0, 0);
    graph_->AddVertex(node);
    auto vtx = graph_->GetVertex(node.id);

    std::vector<GraphType::VertexType *> last_level;
    last_level.push_back(vtx);

    for (size_t i = 0; i < n_unit_time; ++i)
    {
        std::vector<GraphType::VertexType *> this_level;
        for (auto vtx : last_level)
        {
            auto neighbours = GenerateLattices(vtx->state_);
            for (auto &nb : neighbours)
            {
                graph_->AddEdge(vtx->state_, std::get<0>(nb), std::get<1>(nb));

                this_level.push_back(graph_->GetVertex(std::get<0>(nb).id));
                mps.push_back(std::get<1>(nb));
            }
        }
        last_level = this_level;
    }

    lattice_manager_->SavePrimitivesToFile(mps, "graph");
}

std::vector<std::tuple<LatticeNode, MotionPrimitive>> LatticeGraph::GenerateLattices(LatticeNode node)
{
    PrimitiveNode new_base(node.x, node.y, 0, node.theta);
    std::vector<MotionPrimitive> new_mps = lattice_manager_->TransformAllPrimitives(lattice_manager_->primitives_,
                                                                                    new_base.x,
                                                                                    new_base.y,
                                                                                    new_base.theta);
    std::vector<std::tuple<LatticeNode, MotionPrimitive>> neighbours;
    for (auto &mp : new_mps)
    {
        LatticeNode lnode(mp.GetFinalNode().x, mp.GetFinalNode().y, mp.GetFinalNode().theta);
        neighbours.push_back(std::make_pair(lnode, mp));
    }
    return neighbours;
}
