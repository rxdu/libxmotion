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
#include <string>
#include <vector>
#include <memory>

#include "dds/dds.h"

#include "dds_ipc/dds_publisher.hpp"
#include "dds_ipc/dds_qos.hpp"

namespace xmotion {
class DdsParticipant {
 public:
  DdsParticipant(uint32_t domain_id = DDS_DOMAIN_DEFAULT);
  ~DdsParticipant();

  // do not allow copy
  DdsParticipant(const DdsParticipant&) = delete;
  void operator=(const DdsParticipant&) = delete;
  DdsParticipant(DdsParticipant&&) = delete;
  DdsParticipant& operator=(DdsParticipant&&) = delete;

  // public interface
  dds_entity_t entity() const { return participant_; }

  std::shared_ptr<DdsPublisher> CreatePublisher(
      std::string topic_name, const dds_topic_descriptor_t* topic_desc,
      DdsQos qos = DdsQos());

 private:
  uint32_t domain_id_;
  dds_entity_t participant_;

  std::vector<dds_entity_t> topics_;
};
}  // namespace xmotion

#endif  // XMOTION_LIB_DDS_PARTICIPANT_HPP
