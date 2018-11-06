/* 
 * motion_model.cpp
 * 
 * Created on: Aug 03, 2018 11:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "collision_threat/motion_model.hpp"

#include <cmath>
#include <queue>
#include <set>

#include "threat_field/threat_distribution.hpp"

#include "lightviz/lightviz.hpp"

using namespace librav;

MChainLink::MChainLink(int32_t id, Polyline ln) : lanelet_id_(id), polyline_(ln)
{
    length_ = GetLength();
};

MChainLink::~MChainLink()
{
    for (auto &child : child_links)
        delete child;
}

double MChainLink::GetLength()
{
    double len = 0;
    for (int i = 0; i < polyline_.GetPointNumer() - 1; ++i)
    {
        double errx = polyline_.GetPoint(i).x - polyline_.GetPoint(i + 1).x;
        double erry = polyline_.GetPoint(i).y - polyline_.GetPoint(i + 1).y;

        len += std::sqrt(errx * errx + erry * erry);
    }
    return len;
}

//-----------------------------------------------------------------------------------//

MotionChain::MotionChain(MotionModel *model, MotionPoint pt, int32_t start) : start_id_(start), model_(model), mp_pt_(pt)
{
    // setup the base link polyline
    auto pln = model_->road_map_->GetLaneCenterLine(model_->road_map_->GetLaneletNameFromID(start_id_));
    base_link_ = new MChainLink(start_id_, pln);

    // create the chains
    std::vector<MChainLink *> lks;
    lks.push_back(base_link_);
    while (!lks.empty())
    {
        std::vector<MChainLink *> next_lks;

        // look for child links for each link at this depth
        for (auto &lk : lks)
        {
            for (auto &edge : model_->cline_graph_->FindVertex(lk->lanelet_id_)->edges_to_)
            {
                // auto pln = model_->road_map_->GetLaneCenterLine(model_->road_map_->GetLaneletNameFromID(edge.dst_->state_->id));
                auto pln = edge.dst_->state_.center_line;

                MChainLink *new_link = new MChainLink(edge.dst_->state_.id, pln);
                new_link->parent_link = lk;
                lk->child_links.push_back(new_link);

                next_lks.push_back(new_link);
            }
        }

        lks = next_lks;
    }

    // std::cout << "-------------" << std::endl;
    CreateChainPolylines();
    FindStartingPoint();
}

MotionChain::~MotionChain()
{
    delete base_link_;
}

void MotionChain::FindStartingPoint()
{
    // SimplePoint pt;
    double shortest_dist = std::numeric_limits<double>::max();
    std::cout << "base link polyline point number: " << base_link_->polyline_.GetPointNumer() << std::endl;
    for (int32_t i = 0; i < base_link_->polyline_.GetPointNumer() - 1; ++i)
    {
        auto start = base_link_->polyline_.GetPoint(i);
        auto end = base_link_->polyline_.GetPoint(i + 1);
        SimplePoint pt(mp_pt_.estimate_.position_x, mp_pt_.estimate_.position_y);

        Eigen::Vector2d ref_vec(end.x - start.x, end.y - start.y);
        Eigen::Vector2d pt_vec(pt.x - start.x, pt.y - start.y);

        double dot_product = pt_vec.dot(ref_vec);
        if (!(dot_product >= 0 && dot_product < ref_vec.dot(ref_vec)))
            continue;

        double dist = GetPointLineDistance(start, end, pt);
        if (dist < shortest_dist)
        {
            shortest_dist = dist;
            dist_before_start_ = pt_vec.dot(ref_vec.normalized());
            start_segment_idx_ = i;
        }
    }
    std::cout << "shortest distance: " << shortest_dist << std::endl;

    double accumulated = 0;
    for (int i = 0; i < start_segment_idx_; ++i)
    {
        auto start = base_link_->polyline_.GetPoint(i);
        auto end = base_link_->polyline_.GetPoint(i + 1);
        Eigen::Vector2d ref_vec(end.x - start.x, end.y - start.y);

        accumulated += ref_vec.dot(ref_vec.normalized());
    }
    dist_before_start_ += accumulated;
    std::cout << "path segment: " << start_segment_idx_ << " , traveled: " << dist_before_start_ << std::endl;
}

std::vector<MMStatePrediction> MotionChain::Propagate(double t)
{
    double dist_todo = std::hypot(mp_pt_.estimate_.velocity_x, mp_pt_.estimate_.velocity_y) * t;
    double total_dist = dist_before_start_ + dist_todo;

    std::vector<MMStatePrediction> candidates;
    for (auto &line : chain_polylines_)
    {
        double accumulated = 0;
        // check each segment
        for (int i = 0; i < line.GetPointNumer() - 1; ++i)
        {
            auto p0 = line.GetPoint(i);
            auto p1 = line.GetPoint(i + 1);
            Eigen::Vector2d ref_vec(p1.x - p0.x, p1.y - p0.y);
            double segment_len = ref_vec.dot(ref_vec.normalized());

            if (accumulated + segment_len > total_dist)
            {
                double rem_dist = total_dist - accumulated;
                auto ps = CalculatePrediction(p0, p1, rem_dist);
                candidates.push_back(ps);
                break;
            }
            else
            {
                accumulated += segment_len;
            }
        }
    }

    std::vector<MMStatePrediction> predictions;
    for (auto &s : candidates)
    {
        bool duplicated = false;
        for (auto &ps : predictions)
        {
            if (s == ps)
            {
                duplicated = true;
                break;
            }
        }

        if (!duplicated)
            predictions.push_back(s);
    }

    std::cout << "number of predicated states: " << predictions.size() << std::endl;
    for (auto &ps : predictions)
    {
        std::cout << "- state: " << ps << std::endl;
    }

    return predictions;
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
        // std::cout << "lanelet id: " << model_->road_map_->GetLaneletNameFromID(node->lanelet_id_) << " length: " << node->length_ << std::endl;
        if (node->child_links.empty())
            leaf_links_.push_back(node);
        for (auto &nd : node->child_links)
            q.push(nd);
    }
    std::cout << "number of leaf links: " << leaf_links_.size() << std::endl;
}

void MotionChain::CreateChainPolylines()
{
    TraverseChain();

    chain_polylines_.clear();
    for (auto link : leaf_links_)
    {
        MChainLink *parent = link->parent_link;
        std::vector<Polyline::Point> chain_pts;
        chain_pts = link->polyline_.GetPoints();
        while (parent != nullptr)
        {
            auto parent_pts = parent->polyline_.GetPoints();
            // for (auto it = parent_pts.begin() + 1; it != parent_pts.end(); ++it)
            //     line.AddPoint(*it);
            chain_pts.insert(chain_pts.begin(), parent_pts.begin(), parent_pts.end());
            parent = parent->parent_link;
        }
        Polyline line(chain_pts);
        chain_polylines_.push_back(line);
    }
    std::cout << "number of chain polylines: " << chain_polylines_.size() << std::endl;
}

double MotionChain::GetPointLineDistance(SimplePoint ln_pt1, SimplePoint ln_pt2, SimplePoint pt)
{
    double x1 = ln_pt1.x;
    double y1 = ln_pt1.y;

    double x2 = ln_pt2.x;
    double y2 = ln_pt2.y;

    double x0 = pt.x;
    double y0 = pt.y;

    return std::abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1) / std::sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1));
}

MMStatePrediction MotionChain::CalculatePrediction(SimplePoint pt0, SimplePoint pt1, double dist)
{
    Eigen::Vector2d base(pt1.x - pt0.x, pt1.y - pt0.y);
    double base_len = base.dot(base.normalized());
    Eigen::Vector2d ps = dist / base_len * base;
    // Eigen::Vector2d dir = ps.normalized();
    Eigen::Vector2d dir = base.normalized();

    Eigen::Vector2d position = Eigen::Vector2d(pt0.x, pt0.y) + ps;
    double abs_vel = std::hypot(mp_pt_.estimate_.velocity_x, mp_pt_.estimate_.velocity_y);
    Eigen::Vector2d velocity = abs_vel * dir;

    return MMStatePrediction(mp_pt_.estimate_, position(0), position(1), velocity(0), velocity(1));
}

//-----------------------------------------------------------------------------------//

MotionModel::MotionModel(std::shared_ptr<RoadMap> map) : road_map_(map)
{
    ConstructLineNetwork();
}

void MotionModel::ConstructLineNetwork()
{
    cline_graph_ = std::make_shared<Graph<LaneBlock>>();

    for (auto &source : road_map_->GetSources())
        for (auto &sink : road_map_->GetSinks())
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
        auto lanelet_ids = road_map_->FindOccupiedLanelet(CartCooridnate(est.position_x, est.position_y));

        if (lanelet_ids.empty())
        {
            ++ignored_pt_num;
            continue;
        }

        std::cout << "occupied lanelet number: " << lanelet_ids.size() << std::endl;

        MotionPoint pt(est);
        points_.push_back(pt);

        // find the closest center line
        for (auto &id : lanelet_ids)
        {
            // create chains to be maintained
            std::cout << "occupied lanes: " << road_map_->GetLaneletNameFromID(id) << std::endl;

            std::shared_ptr<MotionChain> chain = std::make_shared<MotionChain>(this, pt, id);
            chains_.push_back(chain);
            std::cout << "------" << std::endl;
        }
        std::cout << "total number of chains to be tracked: " << chains_.size() << std::endl;
    }
}

std::vector<MMStatePrediction> MotionModel::PropagateMotionChains(double t)
{
    std::vector<MMStatePrediction> predictions;
    for (auto chain : chains_)
    {
        auto ps = chain->Propagate(t);
        predictions.insert(predictions.end(), ps.begin(), ps.end());
    }

    std::cout << "number of predictions from all chains: " << predictions.size() << std::endl;

    return predictions;
}

std::shared_ptr<CollisionField> MotionModel::GenerateCollisionField()
{
    std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(road_map_->xmin_, road_map_->xmax_, road_map_->ymin_, road_map_->ymax_);

    int32_t pt_count = 0;
    for (auto &pt : points_)
    {
        GaussianPositionVelocityThreat threat_model(pt.estimate_.position_x, pt.estimate_.position_y,
                                                    pt.estimate_.velocity_x, pt.estimate_.velocity_y,
                                                    pt.estimate_.sigma_px, pt.estimate_.sigma_py);
        std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(pt.estimate_.position_x, pt.estimate_.position_y,
                                                                                               pt.estimate_.velocity_x, pt.estimate_.velocity_y);
        participant->id = pt.estimate_.id;
        participant->threat_func = threat_model;
        cfield->AddTrafficParticipant(pt_count++, participant);
    }

    return cfield;
}

std::shared_ptr<CollisionField> MotionModel::GeneratePredictedCollisionField(double t)
{
    if (t == 0)
    {
        return GenerateCollisionField();
    }

    std::shared_ptr<CollisionField> cfield = std::make_shared<CollisionField>(road_map_->xmin_, road_map_->xmax_, road_map_->ymin_, road_map_->ymax_);

    auto predictions = PropagateMotionChains(t);

    int32_t pt_count = 0;
    for (auto &ps : predictions)
    {
        GaussianPositionVelocityThreat threat_model(ps.position_x, ps.position_y,
                                                    ps.velocity_x, ps.velocity_y,
                                                    ps.base_state.sigma_px, ps.base_state.sigma_py);
        std::shared_ptr<TrafficParticipant> participant = std::make_shared<TrafficParticipant>(ps.position_x, ps.position_y,
                                                                                               ps.velocity_x, ps.velocity_y);
        participant->id = ps.base_state.id;
        participant->threat_func = threat_model;
        cfield->AddTrafficParticipant(pt_count++, participant);
    }

    return cfield;
}
