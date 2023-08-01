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

namespace xmotion {
class DdsPublisher {
 public:
  DdsPublisher();
  ~DdsPublisher();

  // do not allow copy
  DdsPublisher(const DdsPublisher&) = delete;
  void operator=(const DdsPublisher&) = delete;
  DdsPublisher(DdsPublisher&&) = delete;
  DdsPublisher& operator=(DdsPublisher&&) = delete;

  // public interface
  template <typename MsgType>
  void Publish(const MsgType& msg);
};
}  // namespace xmotion

#endif  // XMOTION_LIB_DDS_PUBLISHER_HPP
