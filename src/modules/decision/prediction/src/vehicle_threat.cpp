/* 
 * vehicle_threat.cpp
 * 
 * Created on: Nov 11, 2018 09:59
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "prediction/vehicle_threat.hpp"

// #include <tbb/tbb.h>

using namespace autodrive;

VehicleThreat::VehicleThreat(VehicleEstimation est, std::shared_ptr<TrafficMap> tmap) : vehicle_est_(est),
                                                                                        traffic_map_(tmap),
                                                                                        threat_model_(est)
{
    ExtractTrafficInfo();
}

void VehicleThreat::ExtractTrafficInfo()
{
    auto vpose = vehicle_est_.GetPose();
    traffic_chns_ = traffic_map_->FindTrafficChannels(SimplePoint(vpose.position.x, vpose.position.y));

    // std::cout << "occupied traffic channel num: " << traffic_chns_.size() << std::endl;

    for (auto &chn : traffic_chns_)
    {
        // convert vehicle pose info from world frame to path frame
        auto pose_wf = vehicle_est_.GetPose();
        auto pose_pf = chn->ConvertToPathCoordinate(SimplePoint(pose_wf.position.x, pose_wf.position.y));
        double offset = pose_pf.s - threat_model_.s_starting_;
        possible_cases_.emplace_back(chn, offset);

        // std::cout << "chn: " << chn->source_ << " -> " << chn->sink_ << " ; offset: " << offset << std::endl;
    }
}

void VehicleThreat::ComputeOccupancyDistribution(int32_t k, bool calc_interval_dist)
{
    assert(k > 0);

    // propagate for k steps from init state
    //  => (k+1) states, k intervals
    threat_model_.occupancy_model_->Propagate(k, calc_interval_dist);

    for (int32_t i = 0; i < k + 1; ++i)
        ComputeOccupancyRecord(i);
    // tbb::parallel_for(size_t(0), size_t(k+1), std::bind(&VehicleThreat::ComputeOccupancyRecord, this, std::placeholders::_1));

    if (calc_interval_dist)
    {
        for (int32_t i = 0; i < k; ++i)
            ComputeIntervalOccupancyRecord(i);
    }
}

void VehicleThreat::ComputeOccupancyRecord(int32_t t_k)
{
    // update each case
    for (auto &tcase : possible_cases_)
    {
        ThreatRecord record;
        record.occupancy_grid = std::make_shared<CurvilinearGrid>(tcase.channel->center_curve_,
                                                                  threat_model_.s_step_,
                                                                  threat_model_.delta_step_,
                                                                  threat_model_.delta_size_,
                                                                  tcase.start_offset);

        Eigen::VectorXd dist = threat_model_.occupancy_model_->GetOccupancyDistribution(t_k);

        double probability_max = 0;
        std::vector<CurvilinearCell *> nonzero_cells;
        for (std::size_t i = 0; i < dist.size(); ++i)
        {
            if (i < record.occupancy_grid->GetTangentialGridNum())
            {
                for (int32_t j = -record.occupancy_grid->GetOneSideGridNumber(); j <= record.occupancy_grid->GetOneSideGridNumber(); ++j)
                {
                    // Note: extra_attribute is the true probability value
                    auto cell = record.occupancy_grid->GetCell(i, j);
                    // lateral dist * longitudinal dist
                    cell->extra_attribute = dist(i) * DynamicThreatModel::LateralDistribution::GetProbability(j);

                    if (cell->extra_attribute)
                    {
                        auto pt = record.occupancy_grid->ConvertToCurvePoint(cell->center);
                        record.sub_threats.emplace_back(Pose2d(pt.x, pt.y, pt.theta), cell->extra_attribute);

                        nonzero_cells.push_back(cell);
                    }

                    if (cell->extra_attribute > probability_max)
                        probability_max = cell->extra_attribute;
                }
            }
        }

        for (auto cell : nonzero_cells)
        {
            // values here are normalized for better visualization
            cell->cost_map = cell->extra_attribute / probability_max;
        }

        tcase.threat_record_[t_k] = record;
    }
}

void VehicleThreat::ComputeIntervalOccupancyRecord(int32_t t_k)
{
    // update each case
    for (auto &tcase : possible_cases_)
    {
        ThreatRecord record;
        record.occupancy_grid = std::make_shared<CurvilinearGrid>(tcase.channel->center_curve_,
                                                                  threat_model_.s_step_,
                                                                  threat_model_.delta_step_,
                                                                  threat_model_.delta_size_,
                                                                  tcase.start_offset);

        Eigen::VectorXd dist = threat_model_.occupancy_model_->GetIntervalOccupancyDistribution(t_k);

        double probability_max = 0;
        std::vector<CurvilinearCell *> nonzero_cells;
        for (std::size_t i = 0; i < dist.size(); ++i)
        {
            if (i < record.occupancy_grid->GetTangentialGridNum())
            {
                for (int32_t j = -record.occupancy_grid->GetOneSideGridNumber(); j <= record.occupancy_grid->GetOneSideGridNumber(); ++j)
                {
                    // Note: extra_attribute is the true probability value
                    auto cell = record.occupancy_grid->GetCell(i, j);
                    // lateral dist * longitudinal dist
                    cell->extra_attribute = dist(i) * DynamicThreatModel::LateralDistribution::GetProbability(j);

                    if (cell->extra_attribute)
                    {
                        auto pt = record.occupancy_grid->ConvertToCurvePoint(cell->center);
                        record.sub_threats.emplace_back(Pose2d(pt.x, pt.y, pt.theta), cell->extra_attribute);

                        nonzero_cells.push_back(cell);
                    }

                    if (cell->extra_attribute > probability_max)
                        probability_max = cell->extra_attribute;
                }
            }
        }

        for (auto cell : nonzero_cells)
        {
            // values here are normalized for better visualization
            cell->cost_map = cell->extra_attribute / probability_max;
        }

        tcase.intv_threat_record_[t_k] = record;
    }
}

void VehicleThreat::PrintThreatRecordInfo()
{
    std::cout << "number of threat cases: " << possible_cases_.size() << std::endl;

    for (auto &tcase : possible_cases_)
    {
        std::cout << "number of record: " << tcase.threat_record_.size() << std::endl;
        std::cout << "number of interval record: " << tcase.intv_threat_record_.size() << std::endl;
    }
}

double VehicleThreat::operator()(double x, double y, int32_t t_k)
{
    double threat = 0.0;
    const double coeff = 1.0 / possible_cases_.size();
    for (auto &tcase : possible_cases_)
    {
        auto threats = tcase.threat_record_[t_k].sub_threats;
        for (auto &sub : threats)
            threat += sub(x, y) * sub.probability * coeff;
    }
    return threat;
}

Point2d VehicleThreat::GetThreatCenter(int32_t t_k)
{
    Point2d pos(0, 0);
    int32_t total_num = 0;

    for (auto &tcase : possible_cases_)
    {
        auto threats = tcase.threat_record_[t_k].sub_threats;
        for (auto &sub : threats)
        {
            pos.x += sub.pose.position.x;
            pos.y += sub.pose.position.y;
            ++total_num;
        }
    }
    pos.x = pos.x / total_num;
    pos.y = pos.y / total_num;

    return pos;
}
