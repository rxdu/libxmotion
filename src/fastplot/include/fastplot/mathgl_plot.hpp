/* 
 * mathgl_plot.hpp
 * 
 * Created on: Mar 12, 2018 23:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef MATHGL_PLOT_HPP
#define MATHGL_PLOT_HPP

#include <memory>
#include <functional>

#include <Eigen/Dense>

#include "mgl2/mgl.h"
#include <mgl2/glut.h>

#if __cplusplus == 201103L
namespace std
{
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&... args)
{
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
}
#endif

namespace librav
{
namespace MathGLPlot
{

using MathGLPlotGraph_t = std::function<int(mglGraph *)>;

void Run(MathGLPlotGraph_t plot_func, std::string window_name = "MathGL Plot");
void Run(mglDraw *draw, std::string window_name = "MathGL Plot");
};
}

#endif /* MATHGL_PLOT_HPP */
