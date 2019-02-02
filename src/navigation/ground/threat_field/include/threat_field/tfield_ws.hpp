/* 
 * tfield_ws.hpp
 * 
 * Created on: Jan 30, 2019 04:27
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TFIELD_WS_HPP
#define TFIELD_WS_HPP

#include "geometry/polygon.hpp"

namespace librav
{
class TFieldWorkspace
{
  public:
    TFieldWorkspace() = default;
    TFieldWorkspace(double xmin, double xmax, double ymin, double ymax);

  private:
    double xmin_;
    double xmax_;
    double ymin_;
    double ymax_;

    Polygon drivable_;
};
} // namespace librav

#endif /* TFIELD_WS_HPP */
