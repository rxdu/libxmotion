/* 
 * mathgl_surf.hpp
 * 
 * Created on: Mar 14, 2018 21:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MATHGL_SURF_HPP
#define MATHGL_SURF_HPP

#include <iostream>

#include "fastplot/mathgl_plot.hpp"

namespace librav
{
class MathGLSurf : public mglDraw
{
  public:
    int Draw(mglGraph *gr);
};
}

#endif /* MATHGL_SURF_HPP */
