/* 
 * traffic_viewer.hpp
 * 
 * Created on: Nov 21, 2018 01:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef TRAFFIC_VIEWER_HPP
#define TRAFFIC_VIEWER_HPP

#include <memory>
#include <cstdint>

#include "opencv2/opencv.hpp"

#include "lightview/lightviewer.hpp"
#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"

namespace librav
{
class TrafficViewer : public LightViewer
{
  public:
    TrafficViewer() = default;
    TrafficViewer(std::string map_file, int32_t ppu = 10);
    void Start() override;

  private:
    std::shared_ptr<RoadMap> road_map_;
    std::shared_ptr<TrafficMap> traffic_map_;

    // drawing parameters
    int32_t ppu_ = 10;
    double xmin_ = 0;
    double xmax_ = 200;
    double ymin_ = 0;
    double ymax_ = 200;

    void CalcCanvasSize(std::shared_ptr<RoadMap> road_map);
    void DrawOpenCVImage(cv::Mat img);
};
} // namespace librav

#endif /* TRAFFIC_VIEWER_HPP */