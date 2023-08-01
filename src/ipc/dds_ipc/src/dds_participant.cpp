/*
 * dds_participant.cpp
 *
 * Created on 7/25/23 10:46 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "dds_participant.hpp"

#include <exception>

namespace xmotion {
DdsParticipant::DdsParticipant(uint32_t domain_id):domain_id_(domain_id)) {
  participant_ = dds_create_participant(domain_id_, NULL, NULL);

  if (participant < 0) {
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));
    std::throw std::runtime_error("Failed to create DDS participant");
  }
}

DdsParticipant::~DdsParticipant() {}
}  // namespace xmotion