/* 
 * pose_viewer.hpp
 * 
 * Created on: Dec 19, 2018 07:53
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef POSE_VIEWER_HPP
#define POSE_VIEWER_HPP

#include <memory>
#include <cstdint>

#include "opencv2/opencv.hpp"

#include "datalink/lcm_link.hpp"
#include "lightview/lightviewer.hpp"

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

    // drawing parameters
    void HandleEgoVehicleStateMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleState *msg);
    void HandleVehicleEstimationsMsg(const librav::ReceiveBuffer *rbuf, const std::string &chan, const librav_lcm_msgs::VehicleEstimations *msg);
};
} // namespace librav

#endif /* POSE_VIEWER_HPP */
