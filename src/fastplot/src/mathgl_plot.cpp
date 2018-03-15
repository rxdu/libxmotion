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

void MathGLBase::SetAspect(double ax, double ay)
{
    aspect_ratio_x_ = ax;
    aspect_ratio_y_ = ay;

    set_aspect_ratio_ = true;
}

void MathGLBase::SetViewAngles(double ang1, double ang2)
{
    view_angles_[0] = ang1;
    view_angles_[1] = ang2;

    apply_view_angles_ = true;
}

void MathGLBase::SetRangeX(double min, double max)
{
    x_range_[0] = min;
    x_range_[1] = max;
    apply_range_x_ = true;
}

void MathGLBase::SetRangeY(double min, double max)
{
    y_range_[0] = min;
    y_range_[1] = max;
    apply_range_y_ = true;
}

void MathGLBase::SetRangeZ(double min, double max)
{
    z_range_[0] = min;
    z_range_[1] = max;
    apply_range_z_ = true;
}

void MathGLBase::ApplyRangeSettings(mglGraph *gr)
{
    if (apply_range_x_)
        gr->SetRange('x', x_range_[0], x_range_[1]);

    if (apply_range_y_)
        gr->SetRange('y', y_range_[0], y_range_[1]);

    if (apply_range_z_)
        gr->SetRange('z', z_range_[0], z_range_[1]);

    if (apply_range_data_x_)
        gr->SetRange('x', x_range_data_);

    if (apply_range_data_y_)
        gr->SetRange('y', y_range_data_);
}

void MathGLBase::SetBasicElements(mglGraph *gr, std::string title, bool enable_light, double view_angle0, double view_angle1)
{
    gr->Light(enable_light);
    
    gr->Title(title.c_str());
    if (set_aspect_ratio_)
        gr->Aspect(aspect_ratio_x_, aspect_ratio_y_);
    gr->Rotate(view_angle0, view_angle1);
}

void MathGLBase::SetAxisLabels(mglGraph *gr, std::string xlabel, std::string ylabel, std::string zlabel)
{
    gr->Label('x', xlabel.c_str(), 0);
    gr->Label('y', ylabel.c_str(), 0);
    gr->Label('z', zlabel.c_str(), 0);
}