/* 
 * threat_draw.cpp
 * 
 * Created on: Nov 09, 2018 23:24
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "ugvnav_viz/details/threat_draw.hpp"
#include "lightviz/details/matrix_draw.hpp"

using namespace librav;

void ThreatDraw::DrawCollisionThreat(CollisionThreat &threat, double cx, double cy, double xspan, double yspan)
{
    assert(cx >= canvas_.xmin_ && cx < canvas_.xmax_ && cy >= canvas_.ymin_ && cy < canvas_.ymax_);

    // distributions coverage x/y limits
    double dxmin = cx - xspan / 2.0;
    double dxmax = cx + xspan / 2.0;
    double dymin = cy - yspan / 2.0;
    double dymax = cy + yspan / 2.0;

    // crop distribution to canvas area
    if (dxmin < canvas_.xmin_)
        dxmin = canvas_.xmin_;
    if (dxmax > canvas_.xmax_)
        dxmax = canvas_.xmax_;
    if (dymin < canvas_.ymin_)
        dymin = canvas_.ymin_;
    if (dymax > canvas_.ymax_)
        dymax = canvas_.ymax_;

    double dxspan = dxmax - dxmin;
    double dyspan = dymax - dymin;
    int32_t x_size = dxspan * canvas_.ppu_;
    int32_t y_size = dyspan * canvas_.ppu_;

    Eigen::MatrixXd threat_matrix = Eigen::MatrixXd::Zero(y_size, x_size);
    int32_t meter_per_pixel = 1 / canvas_.ppu_;

    std::cout << "draw size: " << x_size << " , " << y_size << std::endl;

    double ppu = canvas_.ppu_;
    for (int32_t i = 0; i < x_size; i = i + 2)
        for (int32_t j = 0; j < y_size; j = j + 2)
        {
            // convert to cartisian coordinate then evaluate
            threat_matrix(j, i) = threat(dxmin + i / ppu, dymin + j / ppu);
        }

    cv::Mat threat_vis = LightViz::CreateColorMapFromEigenMatrix(threat_matrix, true);

    // merge threat distribution to canvas
    auto top_left_pixel = canvas_.ConvertCartisianToPixel(dxmin, dymax); // y inverted in cartesian coordinate
    threat_vis.copyTo(canvas_.paint_area(cv::Rect(top_left_pixel.x, top_left_pixel.y, threat_vis.cols, threat_vis.rows)));
}

void ThreatDraw::DrawCollisionThreatField(ThreatField &field, double cx, double cy, double xspan, double yspan)
{
    assert(cx >= canvas_.xmin_ && cx < canvas_.xmax_ && cy >= canvas_.ymin_ && cy < canvas_.ymax_);

    // distributions coverage x/y limits
    double dxmin = cx - xspan / 2.0;
    double dxmax = cx + xspan / 2.0;
    double dymin = cy - yspan / 2.0;
    double dymax = cy + yspan / 2.0;

    // crop distribution to canvas area
    if (dxmin < canvas_.xmin_)
        dxmin = canvas_.xmin_;
    if (dxmax > canvas_.xmax_)
        dxmax = canvas_.xmax_;
    if (dymin < canvas_.ymin_)
        dymin = canvas_.ymin_;
    if (dymax > canvas_.ymax_)
        dymax = canvas_.ymax_;

    double dxspan = dxmax - dxmin;
    double dyspan = dymax - dymin;
    int32_t x_size = dxspan * canvas_.ppu_;
    int32_t y_size = dyspan * canvas_.ppu_;

    Eigen::MatrixXd threat_matrix = Eigen::MatrixXd::Zero(y_size, x_size);
    int32_t meter_per_pixel = 1 / canvas_.ppu_;

    std::cout << "draw size: " << x_size << " , " << y_size << std::endl;

    double ppu = canvas_.ppu_;
    for (int32_t i = 0; i < x_size; i = i + 1)
        for (int32_t j = 0; j < y_size; j = j + 1)
        {
            // convert to cartisian coordinate then evaluate
            threat_matrix(j, i) = field(dxmin + i / ppu, dymin + j / ppu);
        }

    cv::Mat threat_vis = LightViz::CreateColorMapFromEigenMatrix(threat_matrix, true);

    // merge threat distribution to canvas
    auto top_left_pixel = canvas_.ConvertCartisianToPixel(dxmin, dymax); // y inverted in cartesian coordinate
    threat_vis.copyTo(canvas_.paint_area(cv::Rect(top_left_pixel.x, top_left_pixel.y, threat_vis.cols, threat_vis.rows)));
}