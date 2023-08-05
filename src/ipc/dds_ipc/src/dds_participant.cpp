/*
 * dds_participant.cpp
 *
 * Created on 7/25/23 10:46 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "dds_ipc/dds_participant.hpp"

#include <stdexcept>
#include <exception>

namespace xmotion {
DdsParticipant::DdsParticipant(uint32_t domain_id) : domain_id_(domain_id) {
  participant_ = dds_create_participant(domain_id_, NULL, NULL);

  if (participant_ < 0) {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant_));
    throw std::runtime_error("Failed to create DDS participant");
  }
}

DdsParticipant::~DdsParticipant() { dds_delete(participant_); }

std::shared_ptr<DdsPublisher> DdsParticipant::CreatePublisher(
    std::string topic_name, const dds_topic_descriptor_t* topic_desc,
    DdsQos qos) {
  dds_entity_t topic;
  topic = dds_create_topic(participant_, topic_desc, topic_name.c_str(),
                           qos.entity(), NULL);
  if (topic < 0) {
    DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));
    throw std::runtime_error("Failed to create DDS topic");
  }

  auto publisher = std::make_shared<DdsPublisher>(participant_, topic,
                                                  qos.entity(), nullptr);

  return nullptr;
}
}  // namespace xmotion