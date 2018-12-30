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

namespace librav
{
class StateBase
{
  public:
    StateBase() = default;
    virtual ~StateBase() = default;

    // non-copyable
    StateBase(const StateBase &other) = delete;
    StateBase &operator=(const StateBase &other) = delete;
};

class SpaceBase
{
  public:
    SpaceBase() = default;
    virtual ~SpaceBase() = default;

    // non-copyable
    SpaceBase(const SpaceBase &other) = delete;
    SpaceBase &operator=(const SpaceBase &other) = delete;

    // basic info
    virtual int32_t GetDimension() const = 0;

    // sampling
    virtual StateBase *SampleUniform() = 0;
    virtual StateBase *SampleUniformNear(const StateBase *near, double distance) = 0;
    virtual StateBase *SampleGaussian(const StateBase *mean, double stdDev) = 0;
};
} // namespace librav

#endif /* SPACE_BASE_HPP */
