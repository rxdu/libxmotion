/* 
 * collision_threat.cpp
 * 
 * Created on: Nov 02, 2018 01:40
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "prediction/dynamic_threat_model.hpp"

#include "file_io/folder_path.hpp"

using namespace xmotion;

DynamicThreatModel::DynamicThreatModel(VehicleEstimation est) : vehicle_est_(est)
{
    SetupPredictionModel();
}

void DynamicThreatModel::SetupPredictionModel()
{
    // instantiate Markov model
    occupancy_model_ = std::make_shared<MarkovModel>(0, s_max_, 0, v_max_);

    auto s_var_matrix = vehicle_est_.GetPositionVariance();
    double spd_var = vehicle_est_.GetSpeedVariance();

    occupancy_model_->SetupMarkovModel(s_starting_, s_var_matrix(0, 0), vehicle_est_.GetSpeed(), spd_var, true,
                                       FolderPath::GetDataFolderPath() + "/reachability/vehicle_threat_combined_transition.data",
                                       FolderPath::GetDataFolderPath() + "/reachability/vehicle_threat_combined_transition_interval.data");

    std::cout << "finished setting up markov model" << std::endl;
}
