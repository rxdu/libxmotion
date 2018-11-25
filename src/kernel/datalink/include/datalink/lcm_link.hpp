/* 
 * lcm_link.hpp
 * 
 * Created on: Nov 22, 2018 07:32
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LCM_LINK_HPP
#define LCM_LINK_HPP

#include <string>

// headers for lcm
#include <lcm/lcm-cpp.hpp>
#include "lcmtypes/librav.hpp"

namespace librav
{
class LCMLink : public lcm::LCM
{
  public:
    LCMLink(std::string lcm_url = "");

    bool IsGood() const { return this->good(); }
};
} // namespace librav

#endif /* LCM_LINK_HPP */
