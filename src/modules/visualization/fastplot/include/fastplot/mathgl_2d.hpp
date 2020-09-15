/* 
 * mathgl_2d.hpp
 * 
 * Created on: Mar 14, 2018 21:28
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MATHGL_2D_HPP
#define MATHGL_2D_HPP

#include <iostream>

#include "fastplot/mathgl_plot.hpp"

namespace librav
{
class MathGLDens : public MathGLBase
{
public:
  int Draw(mglGraph *gr);
};

class MathGLSurf : public MathGLBase
{
public:
  int Draw(mglGraph *gr);
};

class MathGLMesh : public MathGLBase
{
public:
  int Draw(mglGraph *gr);
};
}

#endif /* MATHGL_2D_HPP */
