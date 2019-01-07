/* 
 * pose_viewer.hpp
 * 
 * Created on: Jan 07, 2019 04:45
 * Description: 
 * 
 * Copyright (c) 2019 Ruixiang Du (rdu)
 */ 

#ifndef POSE_VIEWER_HPP
#define POSE_VIEWER_HPP

#include <memory>
#include <cstdint>

#include "opencv2/opencv.hpp"

#include "lightview/lightviewer.hpp"
#include "lightview/lightwidgets.hpp"

#include "datalink/lcm_link.hpp"
#include "cav_common/vehicle_state.hpp"
#include "road_map/road_map.hpp"
#include "traffic_map/traffic_map.hpp"

namespace librav
{
class PoseViewer : public LightViewer
{
  public:
    PoseViewer();

    double aspect_ratio_ = 1280.0 / 720.0;

    void Start() override;

  private:
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

    bool ego_state_updated_ = false;
    VehicleState ego_vehicle_state_;
    std::vector<VehicleState> surrounding_vehicles_;

    void CalcCanvasSize(std::shared_ptr<RoadMap> road_map);
    cv::Mat CropImageToROI(cv::Mat img, double cx, double cy, double xspan, double yspan);

    void HandleEgoVehicleStateMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleState *msg);
    void HandleVehicleEstimationsMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleEstimations *msg);
};
} // namespace librav

#endif /* POSE_VIEWER_HPP */
