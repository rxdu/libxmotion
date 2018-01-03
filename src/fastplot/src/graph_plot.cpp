/* 
 * graph_plot.cpp
 * 
 * Created on: Dec 29, 2017 21:03
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "fastplot/graph_plot.hpp"
#include "fastplot/plot_utils.hpp"

using namespace librav;

// template <typename StateType, typename std::enable_if<std::is_pointer<StateType>::value>::type * = nullptr>
void FastPlot::AddGraphLayer(std::shared_ptr<Graph_t<SquareCell*>> graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
{
    cv::Mat src, dst;
    int src_type = _src.getMat().type();
    if (src_type == CV_8UC1)
    {
        cv::cvtColor(_src, src, CV_GRAY2BGR);
        _dst.create(src.size(), src.type());
        dst = _dst.getMat();
    }
    else
    {
        src = _src.getMat();
        _dst.create(_src.size(), _src.type());
        dst = _dst.getMat();
        src.copyTo(dst);
    }

    // draw all vertices
    auto vertices = graph->GetGraphVertices();
    for (auto &vertex : vertices)
    {
        // current vertex center coordinate
        int32_t loc_x, loc_y;
        loc_x = (vertex->state_->bounding_box_.x.min + vertex->state_->bounding_box_.x.max) / 2;
        loc_y = (vertex->state_->bounding_box_.y.min + vertex->state_->bounding_box_.y.max) / 2;

        cv::Point center(loc_x, loc_y);
        PlotUtils::DrawPoint(dst, center);
        
        if (show_id && vertex->state_->GetUniqueID() % 5 == 0)
        {
            std::string id = std::to_string(vertex->state_->GetUniqueID());
            cv::putText(dst, id, cv::Point(loc_x, loc_y), CV_FONT_NORMAL, 0.5, cv::Scalar(204, 204, 102), 1, 1);
        }
    }

    // draw all edges
    auto edges = graph->GetGraphUndirectedEdges();
    for (auto &edge : edges)
    {
        uint64_t loc_x1, loc_y1, loc_x2, loc_y2;
        loc_x1 = (edge.src_->state_->bounding_box_.x.min + edge.src_->state_->bounding_box_.x.max) / 2;
        loc_y1 = (edge.src_->state_->bounding_box_.y.min + edge.src_->state_->bounding_box_.y.max) / 2;
        loc_x2 = (edge.dst_->state_->bounding_box_.x.min + edge.dst_->state_->bounding_box_.x.max) / 2;
        loc_y2 = (edge.dst_->state_->bounding_box_.y.min + edge.dst_->state_->bounding_box_.y.max) / 2;;

        PlotUtils::DrawLine(dst, cv::Point(loc_x1, loc_y1), cv::Point(loc_x2, loc_y2), cv::Scalar(237, 149, 100));
    }
}

// template <typename StateType, typename std::enable_if<!std::is_pointer<StateType>::value>::type * = nullptr>
// void FastPlot::AddGraphLayer(std::shared_ptr<Graph_t<StateType, double>> graph, cv::InputArray _src, cv::OutputArray _dst, bool show_id)
// {
//     cv::Mat src, dst;
//     int src_type = _src.getMat().type();
//     if (src_type == CV_8UC1)
//     {
//         cv::cvtColor(_src, src, CV_GRAY2BGR);
//         _dst.create(src.size(), src.type());
//         dst = _dst.getMat();
//     }
//     else
//     {
//         src = _src.getMat();
//         _dst.create(_src.size(), _src.type());
//         dst = _dst.getMat();
//         src.copyTo(dst);
//     }

//     // draw all vertices
//     auto vertices = graph->GetGraphVertices();
//     for (auto &vertex : vertices)
//     {
//         // current vertex center coordinate
//         int32_t loc_x, loc_y;
//         loc_x = (vertex->state_.bounding_box_.x.min + vertex->state_.bounding_box_.x.max) / 2;
//         loc_y = (vertex->state_.bounding_box_.y.min + vertex->state_.bounding_box_.y.max) / 2;

//         cv::Point center(loc_x, loc_y);
//         PlotUtils::DrawPoint(dst, center);
        
//         if (show_id && vertex->state_.data_id_ % 5 == 0)
//         {
//             std::string id = std::to_string(vertex->state_.GetUniqueID());
//             cv::putText(dst, id, cv::Point(loc_x, loc_y), CV_FONT_NORMAL, 0.5, cv::Scalar(204, 204, 102), 1, 1);
//         }
//     }

//     // draw all edges
//     auto edges = graph->GetGraphUndirectedEdges();
//     for (auto &edge : edges)
//     {
//         uint64_t loc_x1, loc_y1, loc_x2, loc_y2;
//         loc_x1 = (edge.src_->state_.bounding_box_.x.min + edge.src_->state_.bounding_box_.x.max) / 2;
//         loc_y1 = (edge.src_->state_.bounding_box_.y.min + edge.src_->state_.bounding_box_.y.max) / 2;
//         loc_x2 = (edge.dst_->state_.bounding_box_.x.min + edge.dst_->state_.bounding_box_.x.max) / 2;
//         loc_y2 = (edge.dst_->state_.bounding_box_.y.min + edge.dst_->state_.bounding_box_.y.max) / 2;;

//         PlotUtils::DrawLine(dst, cv::Point(loc_x1, loc_y1), cv::Point(loc_x2, loc_y2));
//     }
// }

template <typename StateType>
void FastPlot::AddGraphPathLayer(const Path_t<StateType> &path, cv::InputArray _src, cv::OutputArray _dst, cv::Scalar line_color = cv::Scalar(255, 153, 51))
{
    // cv::Mat src, dst;
    // int src_type = _src.getMat().type();
    // if (src_type == CV_8UC1)
    // {
    //     cv::cvtColor(_src, src, CV_GRAY2BGR);
    //     _dst.create(src.size(), src.type());
    //     dst = _dst.getMat();
    // }
    // else
    // {
    //     src = _src.getMat();
    //     _dst.create(_src.size(), _src.type());
    //     dst = _dst.getMat();
    //     src.copyTo(dst);
    // }

    // // draw starting and finishing cell
    // auto cell_s = path[0]->state_;
    // uint64_t x, y;
    // x = cell_s->location_.x;
    // x = x - (cell_s->bbox_.x.max - cell_s->bbox_.x.min) / 8;
    // y = cell_s->location_.y;
    // y = y + (cell_s->bbox_.y.max - cell_s->bbox_.y.min) / 8;

    // VisUtils::FillRectangularArea(dst, cell_s->bbox_, VisUtils::start_color_);
    // cv::putText(dst, "S", cv::Point(x, y), CV_FONT_NORMAL, 1, cv::Scalar(0, 0, 0), 1, 1);

    // auto cell_f = (*(path.end() - 1))->state_;
    // x = cell_f->location_.x;
    // x = x - (cell_f->bbox_.x.max - cell_f->bbox_.x.min) / 8;
    // y = cell_f->location_.y;
    // y = y + (cell_f->bbox_.y.max - cell_f->bbox_.y.min) / 8;

    // VisUtils::FillRectangularArea(dst, cell_f->bbox_, VisUtils::finish_color_);
    // cv::putText(dst, "F", cv::Point(x, y), CV_FONT_NORMAL, 1, cv::Scalar(0, 0, 0), 1, 1);

    // // draw path
    // uint64_t x1, y1, x2, y2;
    // int thickness = 3;
    // int lineType = 8;
    // int pathline_thickness = 2;

    // for (auto it = path.begin(); it != path.end() - 1; it++)
    // {
    //     // consecutive cells
    //     auto cell1 = (*it)->state_;
    //     auto cell2 = (*(it + 1))->state_;

    //     // center coordinates
    //     x1 = cell1->location_.x;
    //     y1 = cell1->location_.y;

    //     x2 = cell2->location_.x;
    //     y2 = cell2->location_.y;

    //     cv::line(dst,
    //              cv::Point(x1, y1),
    //              cv::Point(x2, y2),
    //              //Scalar( 237, 149, 100 ),
    //              line_color,
    //              pathline_thickness,
    //              lineType);
    // }
}
