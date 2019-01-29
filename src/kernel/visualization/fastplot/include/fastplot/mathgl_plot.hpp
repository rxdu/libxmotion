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
#include <cstdint>

#include <Eigen/Dense>

#include "mgl2/mgl.h"

#include "fastplot/mathgl_utils.hpp"

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

class MathGLBase : public mglDraw
{
  public:
    virtual int Draw(mglGraph *gr) = 0;

  public:
    void SetAspect(double ax, double ay);
    void SetViewAngles(double ang1, double ang2);

    void SetRangeX(double min, double max);
    void SetRangeY(double min, double max);
    void SetRangeZ(double min, double max);

    template <typename Derived>
    void SetRangeDataX(const Eigen::MatrixBase<Derived> &x)
    {
        apply_range_data_x_ = true;
        x_range_data_ = MathGLUtils::ConvertToMathGLData(x);
    }

    template <typename Derived>
    void SetRangeDataY(const Eigen::MatrixBase<Derived> &y)
    {
        apply_range_data_y_ = true;
        y_range_data_ = MathGLUtils::ConvertToMathGLData(y);
    }

    template <typename Derived>
    void SetData(char axis, const Eigen::MatrixBase<Derived> &matrix)
    {
        switch (axis)
        {
        case 'x':
            x_data_ = MathGLUtils::ConvertToMathGLData(matrix);
            break;
        case 'y':
            y_data_ = MathGLUtils::ConvertToMathGLData(matrix);
            break;
        case 'z':
            z_data_ = MathGLUtils::ConvertToMathGLData(matrix);
            break;
        }
    }

  protected:
    bool set_aspect_ratio_ = false;
    double aspect_ratio_x_ = 1;
    double aspect_ratio_y_ = 1;

    bool apply_range_x_ = false;
    bool apply_range_y_ = false;
    bool apply_range_z_ = false;
    double x_range_[2];
    double y_range_[2];
    double z_range_[2];

    bool apply_range_data_x_ = false;
    bool apply_range_data_y_ = false;
    mglData x_range_data_;
    mglData y_range_data_;

    bool apply_view_angles_ = false;
    double view_angles_[2] = {50, 60};

    mglData x_data_;
    mglData y_data_;
    mglData z_data_;

    void ApplyRangeSettings(mglGraph *gr);
    void SetBasicElements(mglGraph *gr, std::string title = "", bool enable_light = false, double view_angle0 = 50, double view_angle1 = 60);
    void SetAxisLabels(mglGraph *gr, std::string xlabel = "x", std::string ylabel = "y", std::string zlabel = "z");
};
}

#endif /* MATHGL_PLOT_HPP */
