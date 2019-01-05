/* 
 * square_grid_draw.hpp
 * 
 * Created on: Apr 09, 2018 23:13
 * Description: this file defines a set of grid drawing 
 *          primitives that can be used in grid_viz.hpp
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef SQUARE_GRID_DRAW_HPP
#define SQUARE_GRID_DRAW_HPP

#include <cstdint>

#include "decomp/square_grid.hpp"
#include "graph/graph.hpp"

#include "cvdraw/cvdraw.hpp"

namespace librav
{
struct SquareGridDraw
{
    template <typename GridType>
    static CvCanvas CreateCanvas(GridType *grid, int32_t ppu = 100)
    {
        int32_t vis_side_size = grid->GetCellSize() * ppu;
        CvCanvas canvas(grid->SizeX() * vis_side_size, grid->SizeY() * vis_side_size);
        canvas.SetMode(CvCanvas::DrawMode::Geometry);
        return canvas;
    }

    template <typename GridType>
    static void DrawGridNet(CvCanvas &canvas, GridType *grid)
    {
        // draw horizontal lines
        for (int32_t y = 1; y < grid->SizeY(); ++y)
        {
            auto first_vertex = grid->GetCell(0, y)->vertices[0];
            auto last_vertex = grid->GetCell(grid->SizeX() - 1, y)->vertices[1];

            // CPoint pt1(first_vertex.x * canvas.GetPPU(), first_vertex.y * canvas.GetPPU());
            // CPoint pt2(last_vertex.x * canvas.GetPPU(), last_vertex.y * canvas.GetPPU());
            CPoint pt1(first_vertex.x, first_vertex.y);
            CPoint pt2(last_vertex.x, last_vertex.y);

            canvas.DrawLine(pt1, pt2);
        }

        // draw vertical lines
        for (int32_t x = 1; x < grid->SizeX(); ++x)
        {
            auto first_vertex = grid->GetCell(x, 0)->vertices[0];
            auto last_vertex = grid->GetCell(x, grid->SizeY() - 1)->vertices[2];

            // CPoint pt1(first_vertex.x * canvas.GetPPU(), first_vertex.y * canvas.GetPPU());
            // CPoint pt2(last_vertex.x * canvas.GetPPU(), last_vertex.y * canvas.GetPPU());
            CPoint pt1(first_vertex.x, first_vertex.y);
            CPoint pt2(last_vertex.x, last_vertex.y);

            canvas.DrawLine(pt1, pt2);
        }
    }

    template <typename GridType>
    static void DrawGridCell(CvCanvas &canvas, GridType *grid)
    {
        // draw cells
        for (int32_t y = 0; y < grid->SizeY(); ++y)
            for (int32_t x = 0; x < grid->SizeX(); ++x)
            {
                auto cell = grid->GetCell(x, y);
                if (cell->label == SquareCellLabel::OCCUPIED)
                {
                    cv::Range rngx(cell->vertices[0].x * canvas.GetPPU(), cell->vertices[1].x * canvas.GetPPU());
                    cv::Range rngy(cell->vertices[0].y * canvas.GetPPU(), cell->vertices[2].y * canvas.GetPPU());
                    canvas.GetPaintArea()(rngy, rngx) = CvColors::obs_color;
                }
            }
    }

    // template <typename GridType>
    // void DrawGridCost(GridType *grid)
    // {
    //     // draw cells
    //     for (int32_t y = 0; y < grid->SizeY(); ++y)
    //         for (int32_t x = 0; x < grid->SizeX(); ++x)
    //         {
    //             auto cell = grid->GetCell(x, y);
    //             if (cell->label != SquareCellLabel::OCCUPIED)
    //             {
    //                 cv::Range rngx(cell->vertices[0].x * canvas.GetPPU(), cell->vertices[1].x * canvas.GetPPU());
    //                 cv::Range rngy(cell->vertices[0].y * canvas.GetPPU(), cell->vertices[2].y * canvas.GetPPU());
    //                 canvas.GetPaintArea()(rngy, rngx) = CvDraw::JetPaletteTransform(cell->cost_map);
    //             }
    //         }
    // }

    // template <typename GridType>
    // void DrawGridCostOnly(GridType *grid)
    // {
    //     // draw cells
    //     for (int32_t y = 0; y < grid->SizeY(); ++y)
    //         for (int32_t x = 0; x < grid->SizeX(); ++x)
    //         {
    //             auto cell = grid->GetCell(x, y);
    //             if (cell->label != SquareCellLabel::OCCUPIED)
    //             {
    //                 cv::Range rngx(cell->vertices[0].x * canvas.GetPPU(), cell->vertices[1].x * canvas.GetPPU());
    //                 cv::Range rngy(cell->vertices[0].y * canvas.GetPPU(), cell->vertices[2].y * canvas.GetPPU());
    //                 if (cell->cost_map != 0.0)
    //                     canvas.GetPaintArea()(rngy, rngx) = CvDraw::JetPaletteTransform(cell->cost_map);
    //             }
    //         }
    // }

    // template <typename GridCellType>
    // void DrawGridPathStartGoal(const std::vector<GridCellType *> &path)
    // {
    //     cv::Range srngx(path.front()->vertices[0].x * canvas.GetPPU(), path.front()->vertices[1].x * canvas.GetPPU());
    //     cv::Range srngy(path.front()->vertices[0].y * canvas.GetPPU(), path.front()->vertices[2].y * canvas.GetPPU());
    //     canvas.GetPaintArea()(srngy, srngx) = CvDrawColors::start_color;

    //     cv::Range grngx(path.back()->vertices[0].x * canvas.GetPPU(), path.back()->vertices[1].x * canvas.GetPPU());
    //     cv::Range grngy(path.back()->vertices[0].y * canvas.GetPPU(), path.back()->vertices[2].y * canvas.GetPPU());
    //     canvas.GetPaintArea()(grngy, grngx) = CvDrawColors::finish_color;
    // }

    // template <typename GridCellType>
    // void DrawGridPath(const std::vector<GridCellType *> &path)
    // {
    //     for (int i = 0; i < path.size() - 1; ++i)
    //     {
    //         auto first_cell = path[i]->center;
    //         auto next_cell = path[i + 1]->center;

    //         cv::Point pt1(first_cell.x * canvas.GetPPU(), first_cell.y * canvas.GetPPU());
    //         cv::Point pt2(next_cell.x * canvas.GetPPU(), next_cell.y * canvas.GetPPU());

    //         CvDraw::DrawLine(canvas.GetPaintArea(), pt1, pt2, CvDrawColors::intermediate_color);
    //     }
    // }

    // // graph related drawing
    // template <typename GridType, typename GridCellType>
    // void DrawGridGraph(GridType *grid, Graph<GridCellType *> *graph)
    // {
    //     // draw all edges
    //     auto edges = graph->GetAllEdges();
    //     for (auto &edge_it : edges)
    //     {
    //         auto edge = *edge_it;
    //         int64_t loc_x1 = edge.src_->state_->center.x * canvas.GetPPU();
    //         int64_t loc_y1 = edge.src_->state_->center.y * canvas.GetPPU();
    //         int64_t loc_x2 = edge.dst_->state_->center.x * canvas.GetPPU();
    //         int64_t loc_y2 = edge.dst_->state_->center.y * canvas.GetPPU();

    //         CvDraw::DrawLine(canvas.GetPaintArea(), cv::Point(loc_x1, loc_y1), cv::Point(loc_x2, loc_y2), cv::Scalar(237, 149, 100));
    //     }

    //     // draw all vertices
    //     for (auto vertex = graph->vertex_begin(); vertex != graph->vertex_end(); ++vertex)
    //     {
    //         // current vertex center coordinate
    //         int32_t loc_x = vertex->state_->center.x * canvas.GetPPU();
    //         int32_t loc_y = vertex->state_->center.y * canvas.GetPPU();

    //         cv::Point center(loc_x, loc_y);
    //         CvDraw::DrawPoint(canvas.GetPaintArea(), center);

    //         // if (show_id && vertex->state_->GetUniqueID() % 5 == 0)
    //         // {
    //         //     std::string id = std::to_string(vertex->state_->GetUniqueID());
    //         //     cv::putText(dst, id, cv::Point(loc_x, loc_y), CV_FONT_NORMAL, 0.5, cv::Scalar(204, 204, 102), 1, 1);
    //         // }
    //     }
    // }
};
} // namespace librav

#endif /* SQUARE_GRID_DRAW_HPP */
