/* 
 * LaneletReference.hpp
 * 
 * Created on: Apr 08, 2018 00:46
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LANELETREFERENCE_HPP
#define LANELETREFERENCE_HPP

#include <string>
#include <vector>

#include "liblanelet/LaneletPoint.hpp"

namespace LLet
{

class LaneletReference
{
public:
  LaneletReference(std::string filename);

  const point_with_id_t &lanelet_by_id(int32_t id) const;

private:
  void init();
  const std::vector<point_with_id_t> reference_points_;
};
}

#endif /* LANELETREFERENCE_HPP */
