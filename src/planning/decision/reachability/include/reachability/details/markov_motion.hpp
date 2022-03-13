/* 
 * markov_motion.hpp
 * 
 * Created on: Oct 29, 2018 08:39
 * Description: Markov-chain longitudinal motion model
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_MOTION_HPP
#define MARKOV_MOTION_HPP

#include <cstdint>
#include <memory>

#include "reachability/tstate_space.hpp"
#include "reachability/details/markov_command.hpp"
#include "reachability/details/occupancy_markov_chain.hpp"

namespace robotnav
{
class SVDistribution
{
  public:
    // SVDistribution() = default;
    SVDistribution(double s_mean, double v_mean, double s_var, double v_var) : s_mean_(s_mean), v_mean_(v_mean), s_var_(s_var), v_var_(v_var)
    {
        SetParameters(s_mean_, v_mean_, s_var_, v_var_);
    };

    void SetParameters(double s_mean, double v_mean, double s_var, double v_var)
    {
        s_mean_ = s_mean;
        v_mean_ = v_mean;
        s_var_ = s_var;
        v_var_ = v_var;

        coeff1_ = -(2 * s_var_);
        coeff2_ = -(2 * v_var_);
        coeff3_ = 2 * M_PI * std::sqrt(s_var_ * v_var_);

        // std::cout << "s-v distribution coefficients (us,uv,sigs,sigv);(3-temp): \n(" << s_mean_ << " , " << v_mean_ << " , " << s_var_ << " , " << v_var_
        //           << " );( " << coeff1_ << " , " << coeff2_ << " , " << coeff3_ << " )\n"
        //           << std::endl;
    }

    double operator()(double s, double v)
    {
        double s_err = s - s_mean_;
        double v_err = v - v_mean_;

        double val = std::exp(s_err * s_err / coeff1_ + v_err * v_err / coeff2_) / coeff3_;

        return val;
    }

    double GetMeanX() const { return s_mean_; }
    double GetMeanY() const { return v_mean_; }

  private:
    double s_mean_ = 0;
    double v_mean_ = 0;

    // variance = sigma^2 , sigma is standard deviation
    double s_var_ = 1;
    double v_var_ = 1;

    double coeff1_ = 1;
    double coeff2_ = 1;
    double coeff3_ = 1;
};

// N: number of system states - i
// M: number of control inputs - alpha, beta
template <int32_t N, int32_t M>
class MarkovMotion : public OccupancyMarkovChain<M * N>
{
  public:
    using Model = OccupancyMarkovChain<M * N>;
    using State = typename OccupancyMarkovChain<M * N>::State;
    using Transition = typename OccupancyMarkovChain<M * N>::Transition;
    using CommandModel = MarkovCommand<N, M>;

  public:
    void SetupModel(std::shared_ptr<TStateSpace> space, Transition trans, Transition int_trans, std::shared_ptr<CommandModel> cmd_model, double s_mean, double s_var, double v_mean, double v_var)
    {
        state_space_ = space;
        cmd_model_ = cmd_model;

        Model::SetInitialState(GenerateInitState(s_mean, v_mean, s_var, v_var));
        Model::SetTransitionMatrix(cmd_model_->GetTransitionMatrix() * trans);
        Model::SetIntervalTransitionMatrix(cmd_model_->GetTransitionMatrix() * int_trans);

        // std::cout << "model init state: \n" << Model::GetInitialState() << std::endl;
        // std::cout << "Combined transition matrix: \n" << combined_trans << std::endl;
    }

    void SetupPrecomputedModel(std::shared_ptr<TStateSpace> space, Transition trans, Transition int_trans, std::shared_ptr<CommandModel> cmd_model, double s_mean, double s_var, double v_mean, double v_var)
    {
        state_space_ = space;
        cmd_model_ = cmd_model;

        Model::SetInitialState(GenerateInitState(s_mean, v_mean, s_var, v_var));
        Model::SetTransitionMatrix(trans);
        Model::SetIntervalTransitionMatrix(int_trans);

        // std::cout << "model init state: \n" << Model::GetInitialState() << std::endl;
        // std::cout << "Combined transition matrix: \n" << combined_trans << std::endl;
    }

    void Propagate(int32_t k)
    {
        Model::Propagate(k);
    }

    State GenerateInitState(double s_mean, double v_mean, double s_var, double v_var)
    {
        SVDistribution dist(s_mean, v_mean, s_var, v_var);

        State init_state;
        init_state.resize(M * N);

        Eigen::VectorXd nc_state;
        nc_state.resize(N);

        auto all_states = state_space_->GetAllStateCells();
        double cell_area = state_space_->GetSStep() * state_space_->GetVStep();
        for (auto &cell_col : all_states)
            for (auto cell : cell_col)
            {
                cell->probability = dist(cell->GetSMiddle(), cell->GetVMiddle()) * cell_area;
                nc_state[cell->id] = cell->probability;
                // if (cell->probability < 1e-8)
                //     cell->probability = 0;
                // std::cout << "cell probability: " << cell->id << " , " << cell->probability << std::endl;
            }

        Eigen::VectorXd pos_prob_vec;
        pos_prob_vec.resize(state_space_->GetSSize());
        for (int i = 0; i < all_states.size(); ++i)
        {
            double col_probability = 0;
            for (auto cell : all_states[i])
                col_probability += cell->probability;
            pos_prob_vec(i) = col_probability;
            // std::cout << "pos probability: " << col_probability << std::endl;
        }
        pos_prob_vec = pos_prob_vec / pos_prob_vec.sum();
        // std::cout << "init position distribution: \n"
        //           << pos_prob_vec << std::endl;

        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                init_state(i * M + j) = nc_state(i) / M;
        init_state = init_state / init_state.sum();

        // std::cout << "p(0): \n" << init_state << std::endl;
        // std::cout << "init sum: " << init_state.sum() << std::endl;

        return init_state;
    }

  private:
    std::shared_ptr<TStateSpace> state_space_;
    std::shared_ptr<CommandModel> cmd_model_;
};
} // namespace robotnav

#endif /* MARKOV_MOTION_HPP */
