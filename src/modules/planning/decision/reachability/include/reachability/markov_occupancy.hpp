/* 
 * markov_occupancy.hpp
 * 
 * Created on: Oct 29, 2018 07:50
 * Description: markov occupancy estimation setup
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_OCCUPANCY_HPP
#define MARKOV_OCCUPANCY_HPP

#include <cstdint>
#include <memory>

#include "reachability/tstate_space.hpp"
#include "reachability/details/markov_command.hpp"
#include "reachability/details/markov_motion.hpp"
#include "reachability/details/tstate_transition_sim.hpp"

#include "file_io/matrix_file.hpp"

namespace rnav
{
// N: number of system states - i
// M: number of control inputs - alpha, beta
template <int32_t SSize, int32_t VSize>
class MarkovOccupancy
{
  public:
    // Markov model for occupancy estimation
    static constexpr int32_t N = SSize * VSize;
    static constexpr int32_t M = 6;

    using CommandModel = MarkovCommand<N, M>;
    using MotionModel = MarkovMotion<N, M>;

  public:
    MarkovOccupancy(double smin, double smax, double vmin, double vmax, double T = 0.5) : state_space_(std::make_shared<TStateSpace>(smin, smax, vmin, vmax)), T_(T)
    {
        // discretize state space
        state_space_->DiscretizeSpaceBySize(SSize, VSize);
    }

    void PrecomputeStateTransition(std::string file_name)
    {
        PrepareModelParams(true, true, file_name);
    }

    void SetupMarkovModel(double s_mean, double s_var, double v_mean, double v_var, bool trans_precomputed = false, std::string trans_file = "", std::string int_trans_file = "")
    {
        if (trans_precomputed)
        {
            PrepareModelParams();
            // load Psi_,Psi_T_ from files
            Eigen::MatrixXd Psi, Psi_T;
            MatrixFile::LoadMatrix(trans_file, Psi);
            MatrixFile::LoadMatrix(int_trans_file, Psi_T);
            motion_.SetupPrecomputedModel(state_space_, Psi, Psi_T, command_, s_mean, s_var, v_mean, v_var);
        }
        else
        {
            PrepareModelParams(true);
            // Psi_,Psi_T_ computed in PrepareModelParams()
            motion_.SetupModel(state_space_, Psi_, Psi_T_, command_, s_mean, s_var, v_mean, v_var);
        }
    }

    void Propagate(int32_t k, bool calc_interval_dist = false)
    {
        if (!calc_interval_dist)
            motion_.Propagate(k);
        else
            motion_.PropagateWithIntervalDist(k);
    }

    Eigen::VectorXd GetOccupancyDistribution(int32_t t_k, double min_p = 1e-3)
    {
        // typename MotionModel::State statef = motion_.CalculateStateAt(t_k);
        typename MotionModel::State statef = motion_.GetStateAt(t_k);

        return ConvertToPositionDist(statef, min_p);
    }

    Eigen::VectorXd GetIntervalOccupancyDistribution(int32_t t_km1_t_k, double min_p = 1e-3)
    {
        // typename MotionModel::State statef = motion_.CalculateStateAt(t_k - 1);
        // typename MotionModel::State statef_int = motion_.CalculateIntervalState(statef);
        typename MotionModel::State statef_int = motion_.GetIntervalStateAt(t_km1_t_k);

        return ConvertToPositionDist(statef_int, min_p);
    }

    double GetPrecitionStepIncrement() const { return T_; }

  private:
    // Tangential state space
    std::shared_ptr<TStateSpace> state_space_;
    // Markov model for motion and command
    std::shared_ptr<CommandModel> command_;
    MotionModel motion_;
    Eigen::MatrixXd Psi_;
    Eigen::MatrixXd Psi_T_;
    // prediction step increment
    double T_;

    void PrepareModelParams(bool compute_transition = false, bool save_to_file = false, std::string file_name = "")
    {
        // setup command Markov model
        command_ = std::make_shared<CommandModel>();

        typename CommandModel::State init_state;
        init_state.resize(1, 6);
        init_state << 0, 0, 0.5, 0.5, 0, 0;

        typename CommandModel::ControlSet cmds;
        cmds.resize(6);
        // cmds << -1, -0.5, 0, 0.3, 0.6, 1.0;
        cmds << -0.6, -0.3, 0, 0.3, 0.6, 1.0;

        typename CommandModel::PriorityVector priority_vec;
        priority_vec.resize(6);
        priority_vec << 0.01, 0.04, 0.25, 0.25, 0.4, 0.05;

        double gamma = 0.2;

        command_->SetupModel(init_state, cmds, priority_vec, gamma);

        if (compute_transition)
        {
            RunSimulation(state_space_, cmds, save_to_file, file_name);
        }
    }

    void RunSimulation(std::shared_ptr<TStateSpace> space, typename CommandModel::ControlSet cmds, bool save_to_file, std::string file_name)
    {
        // calculate transition matrix
        TStateTransitionSim sim;
        sim.SetupStateSpace(space);
        sim.SetControlSet(cmds);
        Psi_ = sim.RunSim(T_);
        Psi_T_ = sim.RunIntervalSim(T_, 10);

        // std::cout << "Psi: \n" << Psi_ << std::endl;
        // std::cout << "Psi_T: \n" << Psi_T_ << std::endl;

        // save to file
        if (save_to_file)
        {
            typename MotionModel::Transition combined_trans = command_->GetTransitionMatrix() * Psi_;
            MatrixFile::SaveMatrix(file_name + ".data", combined_trans, true);

            typename MotionModel::Transition combined_trans_T = command_->GetTransitionMatrix() * Psi_T_;
            MatrixFile::SaveMatrix(file_name + "_interval" + ".data", combined_trans_T, true);
        }
    }

    inline Eigen::VectorXd ConvertToPositionDist(typename MotionModel::State statef, double min_p)
    {
        Eigen::VectorXd pos_prob_vec;
        pos_prob_vec.setZero(state_space_->GetSSize());

        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                pos_prob_vec(i / state_space_->GetVSize()) += statef(i * M + j);

        for (int i = 0; i < state_space_->GetSSize(); ++i)
        {
            if (pos_prob_vec(i) < min_p)
                pos_prob_vec(i) = 0;
        }
        pos_prob_vec = pos_prob_vec / pos_prob_vec.sum();

        return pos_prob_vec;
    }
};
} // namespace rnav

#endif /* MARKOV_OCCUPANCY_HPP */
