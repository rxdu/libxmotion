/* 
 * markov_command.hpp
 * 
 * Created on: Oct 29, 2018 06:56
 * Description: Markov-chain acceleration command model
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MARKOV_COMMAND_HPP
#define MARKOV_COMMAND_HPP

#include <cstdint>
#include <vector>

#include "markov/markov_chain.hpp"

namespace librav
{
// N: number of system states - i
// M: number of control inputs - alpha, beta
template <int32_t N, int32_t M>
class MarkovCommand
{
  public:
    using ConcatenatedTransition = Eigen::Matrix<double, M * N, M * N>;
    using PriorityVector = Eigen::Matrix<double, M, 1>;

    MarkovCommand() = default;

    ConcatenatedTransition GetTransitionMatrix()
    {
    }

  private:
    std::vector<MarkovChain<M>> chains_;
    PriorityVector m_;

    void SetCharacteristicDistribution(PriorityVector m)
    {
    }

    void GenerateInputDynamicMatrix(double gamma)
    {
        for (int32_t i = 0; i < N; ++i)
        {
            MarkovChain<M> Gamma_i;
        }
    }
};
} // namespace librav

#endif /* MARKOV_COMMAND_HPP */
