/* 
 * space_base.hpp
 * 
 * Created on: Dec 29, 2018 05:34
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SPACE_BASE_HPP
#define SPACE_BASE_HPP

#include <cstdint>
#include <functional>

#include "eigen3/Eigen/Dense"

#include "sampling/details/sampler/rand_num_gen.hpp"

namespace librav
{
class State
{
  public:
    explicit State(int64_t id) : id_(id){};
    virtual ~State() = default;

    // non-copyable
    State(const State &other) = delete;
    State &operator=(const State &other) = delete;

    int64_t id_;
};

class SpaceBase
{
  public:
    SpaceBase() = default;
    virtual ~SpaceBase() = default;

    // non-copyable
    SpaceBase(const SpaceBase &other) = delete;
    SpaceBase &operator=(const SpaceBase &other) = delete;

    /****************** To Be Implemented ******************/
    // common interface for space
    virtual int32_t GetDimension() const = 0;
    virtual double EvaluateDistance(const State *sstate, const State *dstate) = 0;

    virtual State *CreateState(std::initializer_list<double> elements) = 0;
    virtual State *CreateState(const Eigen::MatrixXd &elements) = 0;

    virtual State *SampleUniform() = 0;
    virtual State *SampleUniformNear(const State *near, double distance) = 0;
    virtual State *SampleGaussian(const State *mean, double stdDev) = 0;
    /*******************************************************/

  protected:
    RandNumGen rng_;
};
} // namespace librav

#endif /* SPACE_BASE_HPP */
