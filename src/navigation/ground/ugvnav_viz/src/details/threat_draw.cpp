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
#include "ugvnav_viz/details/parallel_for.h"

using namespace librav;

void ThreatDraw::DrawCollisionThreat(VehicleThreat &threat, int32_t t_k, double cx, double cy, double xspan, double yspan)
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

    // serial version
    // for (int32_t i = 0; i < x_size; i = i + 2)
    //     for (int32_t j = 0; j < y_size; j = j + 2)
    //     {
    //         // convert to cartisian coordinate then evaluate
    //         threat_matrix(j, i) = threat(dxmin + i / ppu, dymin + j / ppu, t_k);
    //     }

    int64_t pixel_num = x_size * y_size;

    // OpenMP version
    // #pragma omp parallel for
    //     for (int64_t k = 0; k < pixel_num; ++k)
    //     {
    //         threat_matrix(k % y_size, k / y_size) = threat(dxmin + (k / y_size) / ppu, dymin + (k % y_size) / ppu, t_k);
    //     }

    // parallel version
    const auto &fill_threat_matrix = [&threat, &threat_matrix, x_size, y_size, t_k, dxmin, dymin, ppu](int64_t k) {
        threat_matrix(k % y_size, k / y_size) = threat.GetThreatValueAt(dxmin + (k / y_size) / ppu, dymin + (k % y_size) / ppu, t_k);
    };
    igl::parallel_for(pixel_num, fill_threat_matrix, 100);

    cv::Mat threat_vis = LightViz::CreateColorMapFromEigenMatrix(threat_matrix, true);

    // merge threat distribution to canvas
    auto top_left_pixel = canvas_.ConvertCartisianToPixel(dxmin, dymax); // y inverted in cartesian coordinate
    threat_vis.copyTo(canvas_.paint_area(cv::Rect(top_left_pixel.x, top_left_pixel.y, threat_vis.cols, threat_vis.rows)));
}

void ThreatDraw::DrawCollisionThreatField(ThreatField &field, int32_t t_k, double cx, double cy, double xspan, double yspan)
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

    // serial version
    // for (int32_t i = 0; i < x_size; i = i + 1)
    //     for (int32_t j = 0; j < y_size; j = j + 1)
    //     {
    //         // convert to cartisian coordinate then evaluate
    //         threat_matrix(j, i) = field(dxmin + i / ppu, dymin + j / ppu, t_k);
    //     }

    // OpenMP version
    // #pragma omp parallel for
    //     for (int64_t k = 0; k < pixel_num; ++k)
    //     {
    //         threat_matrix(k % y_size, k / y_size) = field(dxmin + (k / y_size) / ppu, dymin + (k % y_size) / ppu, t_k);
    //     }

    // parallel version
    const auto &fill_threat_matrix = [&field, &threat_matrix, x_size, y_size, t_k, dxmin, dymin, ppu](int64_t k) {
        threat_matrix(k % y_size, k / y_size) = field(dxmin + (k / y_size) / ppu, dymin + (k % y_size) / ppu, t_k);
    };
    int64_t pixel_num = x_size * y_size;
    igl::parallel_for(pixel_num, fill_threat_matrix, 100);

    cv::Mat threat_vis = LightViz::CreateColorMapFromEigenMatrix(threat_matrix, true);

    // merge threat distribution to canvas
    auto top_left_pixel = canvas_.ConvertCartisianToPixel(dxmin, dymax); // y inverted in cartesian coordinate
    threat_vis.copyTo(canvas_.paint_area(cv::Rect(top_left_pixel.x, top_left_pixel.y, threat_vis.cols, threat_vis.rows)));
}