/* 
 * vehicle_threat.cpp
 * 
 * Created on: Nov 11, 2018 09:59
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "threat_field/vehicle_threat.hpp"

using namespace librav;

VehicleThreat::VehicleThreat(VehicleEstimation est, std::shared_ptr<TrafficMap> tmap) : vehicle_est_(est),
                                                                                        traffic_map_(tmap)
{
    ExtractTrafficInfo();
}

void VehicleThreat::ExtractTrafficInfo()
{
    auto vpose = vehicle_est_.GetPose();
    traffic_chns_ = traffic_map_->FindTrafficChannels(SimplePoint(vpose.position.x, vpose.position.y));

    std::cout << "occupied traffic channel num: " << traffic_chns_.size() << std::endl;

    for (auto &chn : traffic_chns_)
    {
        // convert vehicle pose info from world frame to path frame
        auto pose_wf = vehicle_est_.GetPose();
        auto pose_pf = chn->ConvertToPathCoordinate(SimplePoint(pose_wf.position.x, pose_wf.position.y));
        double offset = pose_pf.s - threat_model_.s_starting_;
        possible_cases_.emplace_back(chn, offset);

        std::cout << "chn: " << chn->source_ << " -> " << chn->sink_ << " ; offset: " << offset << std::endl;
    }
}

void VehicleThreat::ComputeOccupancyDistribution(int32_t k, bool calc_interval_dist)
{
    assert(k > 0);

    // propagate for k steps from init state
    //  => (k+1) states, k intervals
    threat_model_.occupancy_model_->Propagate(k, calc_interval_dist);

    for (int32_t i = 0; i < k; ++i)
    {
        threat_record_.insert(std::make_pair(i, GetOccupancyDistributionAt(i)));
        if (calc_interval_dist)
            intv_threat_record_.insert(std::make_pair(i, GetIntervalOccupancyDistributionAt(i)));
    }
    threat_record_.insert(std::make_pair(k, GetOccupancyDistributionAt(k)));
}

VehicleThreat::ThreatDist VehicleThreat::GetOccupancyDistributionAt(int32_t t_k)
{
    ThreatDist dist_result;

    for (auto &tcase : possible_cases_)
    {
        dist_result.occupancy_grid = std::make_shared<CurvilinearGrid>(tcase.channel->center_curve_,
                                                                       threat_model_.s_step_,
                                                                       threat_model_.delta_step_,
                                                                       threat_model_.delta_size_,
                                                                       tcase.start_offset);
        dist_result.sub_threats.clear();

        Eigen::VectorXd dist = threat_model_.occupancy_model_->GetOccupancyDistribution(t_k);

        double probability_max = 0;
        for (std::size_t i = 0; i < dist.size(); ++i)
        {
            if (i < dist_result.occupancy_grid->GetTangentialGridNum())
            {
                for (int32_t j = -dist_result.occupancy_grid->GetOneSideGridNumber(); j <= dist_result.occupancy_grid->GetOneSideGridNumber(); ++j)
                {
                    dist_result.occupancy_grid->GetCell(i, j)->extra_attribute = dist(i) * DynamicThreatModel::LateralDistribution::GetProbability(j);
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
    }

    return dist_result;
}

VehicleThreat::ThreatDist VehicleThreat::GetIntervalOccupancyDistributionAt(int32_t t_k)
{
    ThreatDist dist_result;

    for (auto &tcase : possible_cases_)
    {

        dist_result.occupancy_grid = std::make_shared<CurvilinearGrid>(tcase.channel->center_curve_,
                                                                       threat_model_.s_step_,
                                                                       threat_model_.delta_step_,
                                                                       threat_model_.delta_size_,
                                                                       tcase.start_offset);

        Eigen::VectorXd dist_interval = threat_model_.occupancy_model_->GetIntervalOccupancyDistribution(t_k);

        double probability_max = 0;
        for (std::size_t i = 0; i < dist_interval.size(); ++i)
        {
            if (i < dist_result.occupancy_grid->GetTangentialGridNum())
            {
                for (int32_t j = -dist_result.occupancy_grid->GetOneSideGridNumber(); j <= dist_result.occupancy_grid->GetOneSideGridNumber(); ++j)
                {
                    dist_result.occupancy_grid->GetCell(i, j)->extra_attribute = dist_interval(i) * DynamicThreatModel::LateralDistribution::GetProbability(j);
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
    }

    return dist_result;
}