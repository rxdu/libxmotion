/* 
 * lcm_link.cpp
 * 
 * Created on: Nov 22, 2018 07:52
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "datalink/lcm_link.hpp"

using namespace librav;

LCMLink::LCMLink(std::string lcm_url) : lcm::LCM(lcm_url)
{
}
