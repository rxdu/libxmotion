/* 
 * topogeo_graph.cpp
 * 
 * Created on: Jul 25, 2018 06:56
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "traffic_flow/topogeo_graph.hpp"

#include <set>
#include <algorithm>

using namespace librav;

void TopoGeoGraph::GenerateGraph()
{
    graph_ = std::make_shared<Graph_t<LaneBlock>>();

    // TODO: the topo-geo graph could be generated automatically from traffic flow analysis
    LaneBlock lb1(1, LaneBockType::LaneSegment, "s1");
    LaneBlock lb2(2, LaneBockType::LaneSegment, "s2");
    LaneBlock lb3(3, LaneBockType::LaneSegment, "s3");
    LaneBlock lb4(4, LaneBockType::LaneSegment, "s4");
    LaneBlock lb5(5, LaneBockType::LaneSegment, "s5");
    LaneBlock lb6(6, LaneBockType::LaneSegment, "s6");
    LaneBlock lb7(7, LaneBockType::LaneConnector, "icm2");
    LaneBlock lb8(8, LaneBockType::LaneConnector, "is1");
    LaneBlock lb9(9, LaneBockType::LaneConnector, "icm1");
    LaneBlock lb10(10, LaneBockType::LaneConnector, "im1");
    LaneBlock lb11(11, LaneBockType::LaneConnector, "is2");
    LaneBlock lb12(12, LaneBockType::LaneConnector, "im2");

    // topological connections
    // source "s2"
    graph_->AddEdge(lb2, lb8, 1.0);
    graph_->AddEdge(lb8, lb1, 1.0);

    graph_->AddEdge(lb2, lb9, 1.0);
    graph_->AddEdge(lb9, lb5, 1.0);

    // source "s4"
    graph_->AddEdge(lb4, lb10, 1.0);
    graph_->AddEdge(lb10, lb3, 1.0);

    graph_->AddEdge(lb4, lb7, 1.0);
    graph_->AddEdge(lb7, lb1, 1.0);

    // source "s6"
    graph_->AddEdge(lb6, lb11, 1.0);
    graph_->AddEdge(lb11, lb3, 1.0);

    graph_->AddEdge(lb6, lb11, 1.0);
    graph_->AddEdge(lb11, lb3, 1.0);

    // geometrical connections
    graph_->AddUndirectedEdge(lb7, lb8, 1.0);
    graph_->AddUndirectedEdge(lb8, lb9, 1.0);
    graph_->AddUndirectedEdge(lb7, lb9, 1.0);
    graph_->AddUndirectedEdge(lb11, lb7, 1.0);
    graph_->AddUndirectedEdge(lb11, lb9, 1.0);
    graph_->AddUndirectedEdge(lb11, lb10, 1.0);
    graph_->AddUndirectedEdge(lb11, lb11, 1.0);
    graph_->AddUndirectedEdge(lb7, lb10, 1.0);
    graph_->AddUndirectedEdge(lb9, lb12, 1.0);
}

std::vector<std::string> TopoGeoGraph::BacktrackVertices(int32_t id)
{
    auto vtx = graph_->FindVertex(id);
    std::vector<std::string> names;

    std::set<Graph_t<LaneBlock>::VertexType *> vertices;

    std::vector<Graph_t<LaneBlock>::VertexType *> checked_vtx = vtx->vertices_from_;
    // std::cout << "first level backtrack: " << checked_vtx.size() << std::endl;
    while (!checked_vtx.empty())
    {
        // vertices.insert(vertices.end(), checked_vtx.begin(), checked_vtx.end());
        for (auto cv : checked_vtx)
            vertices.insert(cv);

        std::vector<Graph_t<LaneBlock>::VertexType *> inserted = checked_vtx;
        checked_vtx.clear();
        for (auto &v : inserted)
        {
            // std::cout << "id: " << v->vertex_id_ << " , num: " << v->vertices_from_.size() << std::endl;
            for (auto vf : v->vertices_from_)
            {
                if (v->state_.type == LaneBockType::LaneSegment)
                {
                    if (vf->vertex_id_ != id &&
                        std::find(vertices.begin(), vertices.end(), vf) == vertices.end())
                        checked_vtx.push_back(vf);
                }
                else
                {
                    if (vf->state_.type != LaneBockType::LaneConnector &&
                        vf->vertex_id_ != id &&
                        std::find(vertices.begin(), vertices.end(), vf) == vertices.end())
                        checked_vtx.push_back(vf);
                }
            }
        }
        // std::cout << "checked_vtx size: " << checked_vtx.size() << std::endl;
    }

    // std::cout << "back tracked vertex number: " << vertices.size() << std::endl;
    // add checked vertex first
    names.push_back(vtx->state_.name);
    for (auto v : vertices)
    {
        names.push_back(v->state_.name);
        std::cout << "name: " << v->state_.name << " , id: " << v->state_.id << std::endl;
    }
    std::cout << "total number: " << names.size() << std::endl;

    return names;
}

std::vector<std::string> TopoGeoGraph::FindInteractingLanes(std::vector<int32_t> ids)
{
    std::set<std::string> lanes;
    // add lanes that are possible for interactions
    for (auto id : ids)
    {
        // add lanes that are possible for interactions
        auto vts = BacktrackVertices(id);
        for (auto vtname : vts)
            lanes.insert(vtname);
    }

    // save from set to vector
    std::vector<std::string> lane_names;
    for (auto &n : lanes)
        lane_names.push_back(n);

    return lane_names;
}