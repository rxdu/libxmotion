/* 
 * motion_model.cpp
 * 
 * Created on: Aug 03, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_ranking/motion_model.hpp"

#include <queue>

using namespace librav;

MotionChain::MotionChain(MotionModel *model, MotionPoint pt, int32_t start) : start_id_(start), model_(model), mp_pt_(pt)
{
    // setup the base link polyline
    base_link_ = new MChainLink(start_id_);
    base_link_->polyline = model_->road_map_->GetLaneCenterLine(model_->road_map_->GetLaneletNameFromID(start_id_));

    // create the chains
    auto start_vtx = model_->cline_graph_->GetVertex(start_id_);
    std::cout << "starting vertex: " << start_vtx->state_.name << std::endl;

    chain_depth_ = 1;
    auto neighbours = start_vtx->edges_to_;
    chain_width_ = neighbours.size();

    while (!neighbours.empty())
    {
        std::vector<Graph_t<LaneBlock>::VertexType *> vertices;
        for (auto &nb : neighbours)
            vertices.push_back(nb.dst_);

        neighbours.clear();
        for (auto vt : vertices)
        {
            if (!vt->edges_to_.empty())
                neighbours.insert(neighbours.end(), vt->edges_to_.begin(), vt->edges_to_.end());
        }

        if (neighbours.size() > chain_width_)
            chain_width_ = neighbours.size();
        ++chain_depth_;
    }
    std::cout << "chain depth: " << chain_depth_ << " , width: " << chain_width_ << std::endl;

    std::vector<MChainLink *> lks;
    lks.push_back(base_link_);
    while (!lks.empty())
    {
        std::vector<MChainLink *> next_lks;

        // look for child links for each link at this depth
        for (auto &lk : lks)
        {
            for (auto &edge : model_->cline_graph_->GetVertex(lk->lanelet_id)->edges_to_)
            {
                MChainLink *new_link = new MChainLink(edge.dst_->state_.id);
                new_link->polyline = model_->road_map_->GetLaneCenterLine(model_->road_map_->GetLaneletNameFromID(edge.dst_->state_.id));
                new_link->parent_link = lk;
                lk->child_links.push_back(new_link);

                next_lks.push_back(new_link);
            }
        }

        lks = next_lks;
    }

    std::cout << "-------------" << std::endl;
    TraverseChain();
}

MotionChain::~MotionChain()
{
    delete base_link_;
}

void MotionChain::TraverseChain()
{
    std::queue<MChainLink *> q;
    MChainLink *node;
    q.push(base_link_);
    while (!q.empty())
    {
        node = q.front();
        q.pop();
        std::cout << "lanelet id: " << model_->road_map_->GetLaneletNameFromID(node->lanelet_id) << std::endl;
        for (auto &nd : node->child_links)
            q.push(nd);
    }
}

//-----------------------------------------------------------------------------------//

MotionModel::MotionModel(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    ConstructLineNetwork();
}

void MotionModel::ConstructLineNetwork()
{
    cline_graph_ = std::make_shared<Graph_t<LaneBlock>>();

    for (auto &source : road_map_->traffic_sources_)
        for (auto &sink : road_map_->traffic_sinks_)
        {
            auto path = road_map_->FindShortestRoute(source, sink);
            if (!path.empty())
            {
                for (auto it = path.begin(); it != path.end() - 1; ++it)
                {
                    int32_t src_id = *it;
                    int32_t dst_id = *(it + 1);

                    std::string src_name = road_map_->GetLaneletNameFromID(src_id);
                    std::string dst_name = road_map_->GetLaneletNameFromID(dst_id);

                    LaneBlock src_bk(src_id, src_name);
                    LaneBlock dst_bk(dst_id, dst_name);

                    src_bk.center_line = road_map_->GetLaneCenterLine(src_name);
                    dst_bk.center_line = road_map_->GetLaneCenterLine(dst_name);

                    cline_graph_->AddEdge(src_bk, dst_bk, 1.0);
                }
            }
        }
}

void MotionModel::MergePointsToNetwork()
{
    int32_t ignored_pt_num = 0;

    for (auto &est : ests_)
    {
        // find occupied lanelets
        auto lanelet_ids = road_map_->OccupiedLanelet(CartCooridnate(est.position_x, est.position_y));

        if (lanelet_ids.empty())
        {
            ++ignored_pt_num;
            continue;
        }

        std::cout << "occupied lanelet: " << lanelet_ids.size() << std::endl;

        auto pos = road_map_->coordinate_.ConvertToGridPixel(CartCooridnate(est.position_x, est.position_y));
        std::cout << "(x,y): " << pos << std::endl;

        MotionPoint pt(est);
        pt.SetGridCoordinate(pos);
        points_.push_back(pt);

        // find the closest center line
        for (auto &id : lanelet_ids)
        {
            // create chains to be maintained
            std::cout << "occupied lanes: " << road_map_->GetLaneletNameFromID(id) << std::endl;

            std::shared_ptr<MotionChain> chain = std::make_shared<MotionChain>(this, pt, id);
            chains_.push_back(chain);
        }
    }
}

void MotionModel::GenerateCollisionField()
{
    cfield_ = std::make_shared<CollisionField>(road_map_->grid_size_x_, road_map_->grid_size_y_);
    cfield_->SetOriginCoordinate(0, 0);

    int32_t pt_count = 0;
    for (auto &pt : points_)
    {
        auto tf_pt = std::make_shared<CollisionField::TrafficParticipantType>(road_map_->grid_size_x_, road_map_->grid_size_y_);
        tf_pt->SetPositionVelocity(pt.grid_position_.x, pt.grid_position_.y, pt.estimate_.velocity_x, pt.estimate_.velocity_y);
        cfield_->AddTrafficParticipant(pt_count++, tf_pt);
    }
    cfield_->UpdateCollisionField();
}

Eigen::MatrixXd MotionModel::GetThreatFieldVisMatrix()
{
    return road_map_->GetFullCenterLineGrid()->GetGridMatrix(true) + road_map_->GetFullDrivableAreaGrid()->GetGridMatrix(true) +
           cfield_->GetGridMatrix(true);
}