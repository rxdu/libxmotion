/* 
 * trafficsim_viewer.hpp
 * 
 * Created on: Feb 17, 2019 03:31
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */

#ifndef TRAFFICSIM_VIEWER_HPP
#define TRAFFICSIM_VIEWER_HPP

#include <memory>
#include <cstdint>

#include "opencv2/opencv.hpp"

#include "datalink/lcm_link.hpp"

#include "cav_motion/vehicle_state.hpp"
#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"

#include "stopwatch/stopwatch.h"

namespace autodrive
{
class TrafficSimViewer
{
  public:
    TrafficSimViewer() = delete;
    TrafficSimViewer(std::string map_file, int32_t ppu = 10);

    double aspect_ratio_ = 1280.0 / 720.0;

    void Start();

  private:
    stopwatch::StopWatch loop_stopwatch_;
    std::shared_ptr<LCMLink> data_link_;
    bool data_link_ready_ = false;

    std::shared_ptr<RoadMap> road_map_;
    std::shared_ptr<TrafficMap> traffic_map_;

    // drawing parameters
    int32_t ppu_ = 5;
    double xmin_ = 0;
    double xmax_ = 200;
    double ymin_ = 0;
    double ymax_ = 200;
    double x_span_ = 200;
    double y_span_ = 200;

    cv::Mat background_img_;

    bool ego_state_updated_ = false;
    VehicleState ego_vehicle_state_;
    std::vector<VehicleState> surrounding_vehicles_;

    void CalcCanvasSize(std::shared_ptr<RoadMap> road_map);
    cv::Mat CropImageToROI(cv::Mat img, double cx, double cy, double xspan, double yspan);

    void HandleEgoVehicleStateMsg(const autodrive::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleState *msg);
    void HandleVehicleEstimationsMsg(const autodrive::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleEstimations *msg);
};
} // namespace autodrive

#endif /* TRAFFICSIM_VIEWER_HPP */
