/* 
 * mathgl_plot.cpp
 * 
 * Created on: Mar 12, 2018 23:05
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "fastplot/mathgl_plot.hpp"

using namespace librav;

// Reference:
//  [1] http://naipc.uchicago.edu/2015/ref/cppreference/en/cpp/utility/functional/function/target.html
void MathGLPlot::Run(MathGLPlotGraph_t plot_func, std::string window_name)
{
    using fptr = int (*)(mglGraph *);

    fptr func_ptr = *plot_func.target<int (*)(mglGraph * gr)>();
    std::unique_ptr<mglGLUT> mgl_glut = std::make_unique<mglGLUT>(func_ptr, window_name.c_str());
}

void MathGLPlot::Run(mglDraw *draw, std::string window_name)
{
    std::unique_ptr<mglGLUT> mgl_glut = std::make_unique<mglGLUT>(draw, window_name.c_str());
}