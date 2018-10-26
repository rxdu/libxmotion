/* 
 * state.hpp
 * 
 * Created on: Jun 23, 2018 12:02
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef STATE_HPP
#define STATE_HPP

namespace librav
{
class State
{
  public:
    State() = default;
    virtual ~State() = default;

    virtual void Create() = 0;
};
} // namespace librav

#endif /* STATE_HPP */
