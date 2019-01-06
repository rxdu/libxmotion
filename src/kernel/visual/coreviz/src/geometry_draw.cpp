/* 
 * geometry_draw.cpp
 * 
 * Created on: Aug 10, 2018 09:18
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "coreviz/geometry_draw.hpp"
#include "coreviz/matrix_draw.hpp"

#include <cassert>

#include <tbb/tbb.h>

using namespace librav;

void GeometryDraw::DrawPolyline(CvCanvas &canvas, const Polyline &polyline, bool show_dot, cv::Scalar ln_color, int32_t thickness)
{
    std::size_t pt_num = polyline.GetPointNumer();

    if (pt_num == 0)
        return;

    if (pt_num > 1)
    {
        for (std::size_t i = 0; i < pt_num - 1; ++i)
        {
            CPoint pt1(polyline.GetPoint(i).x, polyline.GetPoint(i).y);
            CPoint pt2(polyline.GetPoint(i + 1).x, polyline.GetPoint(i + 1).y);
            canvas.DrawLine(pt1, pt2, ln_color, thickness);
        }
    }

    if (show_dot)
    {
        for (std::size_t i = 0; i < pt_num; ++i)
            canvas.DrawPoint({polyline.GetPoint(i).x, polyline.GetPoint(i).y}, 1, CvColors::red_color);
    }
}

void GeometryDraw::DrawCubicSpline(CvCanvas &canvas, const CSpline &spline, double step, cv::Scalar ln_color, int32_t thickness)
{
    std::vector<cv::Point2d> pts;
    std::vector<CSpline::Knot> knots(spline.GetAllKnots());
    for (double x = knots.front().x; x < knots.back().x; x += step)
        pts.emplace_back(x, spline.Evaluate(x));

    std::cout << "intermediate points: " << pts.size() << std::endl;

    for (std::size_t i = 0; i < pts.size() - 1; ++i)
        canvas.DrawLine({pts[i].x, pts[i].y}, {pts[i + 1].x, pts[i + 1].y}, ln_color, thickness);
}

void GeometryDraw::DrawParametricCurve(CvCanvas &canvas, const ParametricCurve &pcurve, double step, cv::Scalar ln_color, int32_t thickness)
{
    std::vector<cv::Point2d> pts;

    for (double s = 0; s < pcurve.GetLength(); s += step)
        pts.emplace_back(pcurve.GetXSpline().Evaluate(s), pcurve.GetYSpline().Evaluate(s));

    // std::cout << "intermediate points: " << pts.size() << std::endl;

    for (std::size_t i = 0; i < pts.size() - 1; ++i)
        canvas.DrawLine({pts[i].x, pts[i].y}, {pts[i + 1].x, pts[i + 1].y}, ln_color, thickness);
}

void GeometryDraw::DrawLabelPoint(CvCanvas &canvas, double x, double y, cv::Scalar ln_color, int32_t thickness)
{
    canvas.DrawPoint({x, y}, 2, ln_color, thickness);
}

void GeometryDraw::DrawPolygon(CvCanvas &canvas, const Polygon &polygon, bool show_dot, cv::Scalar ln_color, int32_t thickness)
{
    std::size_t pt_num = polygon.GetPointNumer();

    if (pt_num < 3)
        return;

    for (std::size_t i = 0; i < pt_num - 1; ++i)
    {
        CPoint pt1(polygon.GetPoint(i).x, polygon.GetPoint(i).y);
        CPoint pt2(polygon.GetPoint(i + 1).x, polygon.GetPoint(i + 1).y);
        canvas.DrawLine(pt1, pt2, ln_color, thickness);
    }
    CPoint last_pt(polygon.GetPoint(pt_num - 1).x, polygon.GetPoint(pt_num - 1).y);
    CPoint first_pt(polygon.GetPoint(0).x, polygon.GetPoint(0).y);
    canvas.DrawLine(last_pt, first_pt, ln_color, thickness);

    if (show_dot)
    {
        for (std::size_t i = 0; i < pt_num; ++i)
            canvas.DrawPoint({polygon.GetPoint(i).x, polygon.GetPoint(i).y}, 1, CvColors::red_color);
    }
}

void GeometryDraw::FillPolygon(CvCanvas &canvas, const Polygon &polygon, bool show_dot, cv::Scalar fill_color, cv::Scalar ln_color, int32_t thickness)
{
    std::size_t pt_num = polygon.GetPointNumer();

    if (pt_num < 3)
        return;

    std::vector<CPoint> pts;
    for (int i = 0; i < polygon.GetPointNumer(); ++i)
        pts.emplace_back(polygon.GetPoint(i).x, polygon.GetPoint(i).y);

    canvas.FillPoly(pts, fill_color);
}

/*
 * Assumptions:
 *  1. Front line: first two points
*/
void GeometryDraw::DrawPolygonDirection(CvCanvas &canvas, const Polygon &polygon, cv::Scalar ln_color, int32_t thickness)
{
    if (polygon.GetPointNumer() < 3)
        return;

    std::vector<CPoint> points;
    for (int i = 0; i < polygon.GetPointNumer(); ++i)
        points.emplace_back(polygon.GetPoint(i).x, polygon.GetPoint(i).y);

    double center_x = 0;
    double center_y = 0;

    for (auto &pt : points)
    {
        center_x += pt.x;
        center_y += pt.y;
    }
    center_x = center_x / polygon.GetPointNumer();
    center_y = center_y / polygon.GetPointNumer();

    double front_x = (points[0].x + points[1].x) / 2.0;
    double front_y = (points[0].y + points[1].y) / 2.0;

    canvas.DrawArrowedLine({center_x, center_y}, {front_x, front_y}, 0.1, ln_color, thickness);
}

void GeometryDraw::WritePointPosition(CvCanvas &canvas, const std::vector<SimplePoint> &points)
{
    for (auto &pt : points)
    {
        std::string pos_str = "(" + std::to_string(static_cast<int32_t>(pt.x)) + "," + std::to_string(static_cast<int32_t>(pt.y)) + ")";
        canvas.WriteText(pos_str, {pt.x, pt.y});
    }
}

void GeometryDraw::WriteTextAtPosition(CvCanvas &canvas, std::string txt, SimplePoint pt)
{
    canvas.WriteText(txt, {pt.x, pt.y});
}

void GeometryDraw::DrawDistribution(CvCanvas &canvas, double cx, double cy, double xspan, double yspan, std::function<double(double, double)> dist_fun)
{
    double xmin, xmax, ymin, ymax;
    canvas.GetCanvasRange(xmin, xmax, ymin, ymax);

    assert(cx >= xmin && cx < xmax && cy >= ymin && cy < ymax);

    // distributions coverage x/y limits
    double dxmin = cx - xspan / 2.0;
    double dxmax = cx + xspan / 2.0;
    double dymin = cy - yspan / 2.0;
    double dymax = cy + yspan / 2.0;

    // crop distribution to canvas area
    if (dxmin < xmin)
        dxmin = xmin;
    if (dxmax > xmax)
        dxmax = xmax;
    if (dymin < ymin)
        dymin = ymin;
    if (dymax > ymax)
        dymax = ymax;

    double dxspan = dxmax - dxmin;
    double dyspan = dymax - dymin;

    double ppu = canvas.GetPPU();

    int32_t x_size = dxspan * ppu;
    int32_t y_size = dyspan * ppu;

    Eigen::MatrixXd threat_matrix = Eigen::MatrixXd::Zero(y_size, x_size);
    int32_t meter_per_pixel = 1 / ppu;

    int64_t pixel_num = x_size * y_size;

    /* other implementations of the value assignment
    // serial version
    for (int32_t i = 0; i < x_size; ++i)
        for (int32_t j = 0; j < y_size; ++j)
        {
            // convert to cartisian coordinate
            double x = dxmin + i / ppu;
            double y = dymin + j / ppu;

            threat_matrix(j, i) = dist_fun(x, y);
        }

    // OpenMP version
    #pragma omp parallel for
    for (int64_t k = 0; k < pixel_num; ++k)
    {
        threat_matrix(k % y_size, k / y_size) = threat(dxmin + (k / y_size) / ppu, dymin + (k % y_size) / ppu, t_k);
    }
    */

    // TBB parallel version
    const auto &fill_threat_matrix2 = [&dist_fun, &threat_matrix, x_size, y_size, dxmin, dymin, ppu](size_t k) {
        threat_matrix(k % y_size, k / y_size) = dist_fun(dxmin + (k / y_size) / ppu, dymin + (k % y_size) / ppu);
    };
    tbb::parallel_for(size_t(0), size_t(pixel_num), fill_threat_matrix2);

    cv::Mat threat_vis = MatrixDraw::CreateColorMapFromEigenMatrix(threat_matrix, true);

    // merge threat distribution to canvas
    auto top_left_pixel = canvas.ConvertGeometryPointToPixel(dxmin, dymax); // y inverted in cartesian coordinate
    threat_vis.copyTo(canvas.GetPaintArea()(cv::Rect(top_left_pixel.x, top_left_pixel.y, threat_vis.cols, threat_vis.rows)));
}
