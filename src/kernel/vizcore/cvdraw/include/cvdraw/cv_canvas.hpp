/* 
 * cv_canvas.hpp
 * 
 * Created on: Jan 04, 2019 10:40
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef CV_CANVAS_HPP
#define CV_CANVAS_HPP

#include "cvdraw/details/cvdraw_headers.hpp"
#include "cvdraw/details/cv_colors.hpp"

namespace librav
{
class CvCanvas
{
  public:
    CvCanvas(cv::Mat background, std::string name = "CvCanvas");
    CvCanvas(int32_t ppu, std::string name = "CvCanvas") : ppu_(ppu), canvas_name_(name) {}
    CvCanvas(int32_t px, int32_t py, cv::Scalar bg_color = CvColors::bg_color, std::string name = "CvCanvas");

    void SetupCanvas(double xmin, double xmax, double ymin, double ymax, cv::Scalar bg_color = CvColors::bg_color);
    void ClearCanvas(cv::Scalar bg_color);

    cv::Mat GetPaintArea() { return paint_area_; }

    void Save(std::string filename);
    void Show(bool use_img_size = false);
    void ShowFrame(int32_t frame_period_ms = 0);

  private:
    int32_t ppu_ = 10;

    // size parameters
    double xmin_ = 0.0;
    double xmax_ = 80.0;
    double ymin_ = 0.0;
    double ymax_ = 60.0;
    double xspan_ = 80.0;
    double yspan_ = 60.0;
    int32_t canvas_size_x_ = 800;
    int32_t canvas_size_y_ = 600;

    cv::Mat paint_area_;
    cv::Scalar bg_color_;
    std::string canvas_name_;

    void InitCanvas();
};
} // namespace librav

#endif /* CV_CANVAS_HPP */
