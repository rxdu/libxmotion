/*
 * dds_topic.hpp
 *
 * Created on 8/2/23 9:35 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef ROBOSW_DDS_TOPIC_HPP
#define ROBOSW_DDS_TOPIC_HPP

#include "dds/dds.h"

namespace xmotion {
class DdsTopic {
 public:
  dds_entity_t entity() const { return topic_; }

 private:
  dds_entity_t topic_;
};
}  // namespace xmotion

#endif  // ROBOSW_DDS_TOPIC_HPP
