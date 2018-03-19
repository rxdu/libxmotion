/* 
 * map_plot.cpp
 * 
 * Created on: Dec 29, 2017 14:38
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "fastplot/map_plot.hpp"
#include "fastplot/image_plot.hpp"
#include "fastplot/cvplot_utils.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace librav;
using namespace cv;

void FastPlot::PlotSquareGrid(std::shared_ptr<SquareGrid> grid, bool show_id)
{
    cv::Mat vis_img;
    CreateSquareGridLayer(grid, vis_img, show_id);
    FastPlot::ShowImage(vis_img, "SquareGrid");
}

void FastPlot::CreateSquareGridLayer(std::shared_ptr<SquareGrid> grid, cv::OutputArray _dst, bool show_id)
{
    int32_t vis_side_size = grid->cell_size_ * grid->pixel_per_meter_;
    _dst.create(Size(grid->col_size_ * vis_side_size, grid->row_size_ * vis_side_size), CV_8UC3);
    Mat dst = _dst.getMat();
    dst = FastplotColors::bg_color;

    // fill cell color
    for (auto &col : grid->grid_cells_)
        for (auto &cell : col)
        {
            if (cell->occupancy_ == OccupancyType::OCCUPIED)
                PlotUtils::DrawSquare(dst, cell->bounding_box_, FastplotColors::obs_color);
            else if (cell->occupancy_ == OccupancyType::INTERESTED)
                PlotUtils::DrawSquare(dst, cell->bounding_box_, FastplotColors::aoi_color);

            if (show_id)
            {
                int64_t x, y;
                x = cell->bounding_box_.x.min + (cell->bounding_box_.x.max - cell->bounding_box_.x.min) / 2;
                x = x + (cell->bounding_box_.x.max - cell->bounding_box_.x.min) / 6;
                y = cell->bounding_box_.y.min + (cell->bounding_box_.y.max - cell->bounding_box_.y.min) / 2;
                y = y + (cell->bounding_box_.y.max - cell->bounding_box_.y.min) * 3 / 7;

                std::string id = std::to_string(cell->id_);
                putText(dst, id, Point(x, y), CV_FONT_NORMAL, 1.0, Scalar(0, 0, 0), 1, 1);
            }
        }
    // draw grid lines
    PlotUtils::DrawLine(dst, Point(0, 0), Point(0, grid->row_size_ * vis_side_size - 1), FastplotColors::ln_color);
    for (int i = 1; i <= grid->col_size_; i++)
        PlotUtils::DrawLine(dst, Point(i * vis_side_size - 1, 0), Point(i * vis_side_size - 1, grid->row_size_ * vis_side_size - 1), FastplotColors::ln_color);

    PlotUtils::DrawLine(dst, Point(0, 0), Point(grid->col_size_ * vis_side_size - 1, 0), FastplotColors::ln_color);
    for (int i = 1; i <= grid->row_size_; i++)
        PlotUtils::DrawLine(dst, Point(0, i * vis_side_size - 1), Point(grid->col_size_ * vis_side_size - 1, i * vis_side_size - 1), FastplotColors::ln_color);
}

void FastPlot::AddSquareGridLayer(std::shared_ptr<SquareGrid> grid, cv::InputArray _src, cv::OutputArray _dst)
{
    Mat src_img_color;
    cvtColor(_src, src_img_color, CV_GRAY2BGR);
    _dst.create(src_img_color.size(), src_img_color.type());
    Mat dst = _dst.getMat();

    int32_t vis_side_size = grid->cell_size_ * grid->pixel_per_meter_;
    PlotUtils::DrawLine(dst, Point(0, 0), Point(0, grid->row_size_ * vis_side_size - 1), FastplotColors::ln_color);
    for (int i = 1; i <= grid->col_size_; i++)
        PlotUtils::DrawLine(dst, Point(i * vis_side_size - 1, 0), Point(i * vis_side_size - 1, grid->row_size_ * vis_side_size - 1), FastplotColors::ln_color);

    PlotUtils::DrawLine(dst, Point(0, 0), Point(grid->col_size_ * vis_side_size - 1, 0), FastplotColors::ln_color);
    for (int i = 1; i <= grid->row_size_; i++)
        PlotUtils::DrawLine(dst, Point(0, i * vis_side_size - 1), Point(grid->col_size_ * vis_side_size - 1, i * vis_side_size - 1), FastplotColors::ln_color);

    src_img_color.copyTo(dst);
}