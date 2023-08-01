/*
 * dds_participant.hpp
 *
 * Created on 7/25/23 10:46 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#ifndef XMOTION_LIB_DDS_PARTICIPANT_HPP
#define XMOTION_LIB_DDS_PARTICIPANT_HPP

#include <cstdint>

namespace xmotion {
class DdsParticipant {
 public:
  DdsParticipant(uint32_t domain_id);
  ~DdsParticipant();

  // do not allow copy
  DdsParticipant(const DdsParticipant&) = delete;
  void operator=(const DdsParticipant&) = delete;
  DdsParticipant(DdsParticipant&&) = delete;
  DdsParticipant& operator=(DdsParticipant&&) = delete;

  // public interface
//  template <typename MsgType>
//  std::shared_ptr<DdsPublisher> CreatePublisher(
//      std::string topic_name, DdsPublisherQos qos = DdsPublisherQos());

 private:
  uint32_t domain_id_;
  dds_entity_t participant_;
};
}  // namespace xmotion

#endif  // XMOTION_LIB_DDS_PARTICIPANT_HPP
