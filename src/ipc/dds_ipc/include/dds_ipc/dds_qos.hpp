/*
 * dds_qos.hpp
 *
 * Created on 8/2/23 10:00 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef XMOTION_DDS_QOS_HPP
#define XMOTION_DDS_QOS_HPP

#include "dds/dds.h"

namespace xmotion {
class DdsQos {
 public:
  DdsQos();
  ~DdsQos();

 public:
  dds_qos_t* entity() const { return qos_; }

 private:
  dds_qos_t* qos_;
};
}  // namespace xmotion

#endif  // XMOTION_DDS_QOS_HPP
