/*
 * dds_publisher.hpp
 *
 * Created on 7/25/23 11:09 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef XMOTION_LIB_DDS_PUBLISHER_HPP
#define XMOTION_LIB_DDS_PUBLISHER_HPP

#include "dds/dds.h"

namespace xmotion {
class DdsPublisher {
 public:
  DdsPublisher(dds_entity_t participant, dds_entity_t topic,
               const dds_qos_t *qos, const dds_listener_t *listener) {
    writer_ = dds_create_writer(participant, topic, qos, listener);
    if (writer_ < 0) {
      DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer_));
    }
  }
  ~DdsPublisher() = default;

  // do not allow copy
  DdsPublisher(const DdsPublisher &) = delete;
  void operator=(const DdsPublisher &) = delete;
  DdsPublisher(DdsPublisher &&) = delete;
  DdsPublisher &operator=(DdsPublisher &&) = delete;

  // public interface
  template <typename MsgType>
  void Publish(const MsgType *msg) {
    dds_write(writer_, msg);
  }

 private:
  dds_entity_t writer_;
};
}  // namespace xmotion

#endif  // XMOTION_LIB_DDS_PUBLISHER_HPP
