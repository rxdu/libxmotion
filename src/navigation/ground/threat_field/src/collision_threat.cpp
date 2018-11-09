/* 
 * collision_threat.cpp
 * 
 * Created on: Nov 02, 2018 01:40
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/collision_threat.hpp"

#include "file_io/folder_path.hpp"

using namespace librav;

CollisionThreat::CollisionThreat(VehicleEstimation est, std::shared_ptr<TrafficChannel> chn) : vehicle_est_(est), traffic_chn_(chn)
{
    SetupPredictionModel();
}

void CollisionThreat::SetupPredictionModel()
{
    // convert vehicle pose info from world frame to path frame
    auto pose_wf = vehicle_est_.GetPose();
    auto pose_pf = traffic_chn_->ConvertToPathCoordinate(SimplePoint(pose_wf.position.x, pose_wf.position.y));

    // instantiate Markov model
    occupancy_model_ = std::make_shared<MarkovModel>(0, s_max_, 0, v_max_);
    auto s_var_matrix = vehicle_est_.GetPositionVariance();
    double spd_var = vehicle_est_.GetSpeedVariance();

    s_offset_ = pose_pf.s - s_starting_;
    occupancy_model_->SetupMarkovModel(s_starting_, s_var_matrix(0, 0), vehicle_est_.GetSpeed(), spd_var, true,
                                       FolderPath::GetDataFolderPath() + "/reachability/vehicle_threat_combined_transition.data",
                                       FolderPath::GetDataFolderPath() + "/reachability/vehicle_threat_combined_transition_interval.data");

    std::cout << "finished setting up markov model" << std::endl;
}

void CollisionThreat::ComputeOccupancyDistribution(int32_t k, bool calc_interval_dist)
{
    assert(k > 0);

    // propagate for k steps from init state
    //  => (k+1) states, k intervals
    occupancy_model_->Propagate(k, calc_interval_dist);

    for (int32_t i = 0; i < k; ++i)
    {
        threat_record_.insert(std::make_pair(i, GetOccupancyDistributionAt(i)));
        if (calc_interval_dist)
            intv_threat_record_.insert(std::make_pair(i, GetIntervalOccupancyDistributionAt(i)));
    }
    threat_record_.insert(std::make_pair(k, GetOccupancyDistributionAt(k)));

    // save last record for visualization
    occupancy_grid_ = threat_record_[k].occupancy_grid;
    sub_threats_ = threat_record_[k].sub_threats;
    if (calc_interval_dist)
    {
        interval_occupancy_grid_ = intv_threat_record_[k - 1].occupancy_grid;
        sub_int_threats_ = intv_threat_record_[k - 1].sub_threats;
    }
}

CollisionThreat::ThreatDist CollisionThreat::GetOccupancyDistributionAt(int32_t t_k)
{
    ThreatDist dist_result;

    dist_result.occupancy_grid = std::make_shared<CurvilinearGrid>(traffic_chn_->center_curve_, s_step_, delta_step_, delta_size_, s_offset_);
    dist_result.sub_threats.clear();

    Eigen::VectorXd dist = occupancy_model_->GetOccupancyDistribution(t_k);

    double probability_max = 0;
    for (std::size_t i = 0; i < dist.size(); ++i)
    {
        if (i < dist_result.occupancy_grid->GetTangentialGridNum())
        {
            for (int32_t j = -dist_result.occupancy_grid->GetOneSideGridNumber(); j <= dist_result.occupancy_grid->GetOneSideGridNumber(); ++j)
            {
                dist_result.occupancy_grid->GetCell(i, j)->extra_attribute = dist(i) * LateralDistribution::GetProbability(j);
                if (dist_result.occupancy_grid->GetCell(i, j)->extra_attribute > probability_max)
                    probability_max = dist_result.occupancy_grid->GetCell(i, j)->extra_attribute;
            }
        }
    }

    for (std::size_t i = 0; i < dist.size(); ++i)
    {
        if (i < dist_result.occupancy_grid->GetTangentialGridNum())
        {
            for (int32_t j = -dist_result.occupancy_grid->GetOneSideGridNumber(); j <= dist_result.occupancy_grid->GetOneSideGridNumber(); ++j)
            {
                auto cell = dist_result.occupancy_grid->GetCell(i, j);

                if (cell->extra_attribute > 0)
                {
                    // Note: extra_attribute is the true probability value
                    auto pt = dist_result.occupancy_grid->ConvertToCurvePoint(cell->center);
                    dist_result.sub_threats.emplace_back(Pose2d(pt.x, pt.y, pt.theta), cell->extra_attribute);

                    // values here are normalized for better visualization
                    cell->cost_map = cell->extra_attribute / probability_max;
                }
            }
        }
    }

    return dist_result;
}

CollisionThreat::ThreatDist CollisionThreat::GetIntervalOccupancyDistributionAt(int32_t t_k)
{
    ThreatDist dist_result;

    dist_result.occupancy_grid = std::make_shared<CurvilinearGrid>(traffic_chn_->center_curve_, s_step_, delta_step_, delta_size_, s_offset_);
    dist_result.sub_threats.clear();

    Eigen::VectorXd dist_interval = occupancy_model_->GetIntervalOccupancyDistribution(t_k);

    double probability_max = 0;
    for (std::size_t i = 0; i < dist_interval.size(); ++i)
    {
        if (i < dist_result.occupancy_grid->GetTangentialGridNum())
        {
            for (int32_t j = -dist_result.occupancy_grid->GetOneSideGridNumber(); j <= dist_result.occupancy_grid->GetOneSideGridNumber(); ++j)
            {
                dist_result.occupancy_grid->GetCell(i, j)->extra_attribute = dist_interval(i) * LateralDistribution::GetProbability(j);
                if (dist_result.occupancy_grid->GetCell(i, j)->extra_attribute > probability_max)
                    probability_max = dist_result.occupancy_grid->GetCell(i, j)->extra_attribute;
            }
        }
    }

    for (std::size_t i = 0; i < dist_interval.size(); ++i)
    {
        if (i < dist_result.occupancy_grid->GetTangentialGridNum())
        {
            for (int32_t j = -dist_result.occupancy_grid->GetOneSideGridNumber(); j <= dist_result.occupancy_grid->GetOneSideGridNumber(); ++j)
            {
                auto cell = dist_result.occupancy_grid->GetCell(i, j);

                if (cell->extra_attribute > 0)
                {
                    // Note: extra_attribute is the true probability value
                    auto pt = dist_result.occupancy_grid->ConvertToCurvePoint(cell->center);
                    dist_result.sub_threats.emplace_back(Pose2d(pt.x, pt.y, pt.theta), cell->extra_attribute);

                    // values here are normalized for better visualization
                    cell->cost_map = cell->extra_attribute / probability_max;
                }
            }
        }
    }

    return dist_result;
}

double CollisionThreat::operator()(double x, double y, bool is_interval)
{
    double threat = 0.0;

    if (!is_interval)
    {
        for (auto &sub : sub_threats_)
            threat += sub(x, y) * sub.probability;
    }
    else
    {
        for (auto &sub : sub_int_threats_)
            threat += sub(x, y) * sub.probability;
    }

    return threat;
}

Point2d CollisionThreat::GetThreatCenter()
{
    Point2d pos(0, 0);
    for (auto &sub : sub_threats_)
    {
        pos.x += sub.pose.position.x;
        pos.y += sub.pose.position.y;
    }
    pos.x = pos.x / sub_threats_.size();
    pos.y = pos.y / sub_threats_.size();
    return pos;
}

double CollisionThreat::operator()(double x, double y, int32_t t_k)
{
    // assert(t_k < threat_record_.size());

    auto threats = threat_record_[t_k].sub_threats;
    double threat = 0.0;
    for (auto &sub : threats)
        threat += sub(x, y) * sub.probability;
    return threat;
}

Point2d CollisionThreat::GetThreatCenter(int32_t t_k)
{
    // assert(t_k < threat_record_.size());

    auto threats = threat_record_[t_k].sub_threats;
    Point2d pos(0, 0);
    for (auto &sub : threats)
    {
        pos.x += sub.pose.position.x;
        pos.y += sub.pose.position.y;
    }
    pos.x = pos.x / threats.size();
    pos.y = pos.y / threats.size();
    return pos;
}