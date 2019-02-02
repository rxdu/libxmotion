/* 
 * tfield_ws.cpp
 * 
 * Created on: Jan 30, 2019 04:29
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#include "threat_field/tfield_ws.hpp"

namespace librav
{
TFieldWorkspace::TFieldWorkspace(double xmin, double xmax, double ymin, double ymax) : xmin_(xmin),
                                                                                       xmax_(xmax),
                                                                                       ymin_(ymin),
                                                                                       ymax_(ymax)
{
}
} // namespace librav