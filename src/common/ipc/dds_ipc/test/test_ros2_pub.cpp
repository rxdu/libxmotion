/*
 * test_dds_helloworld.cpp
 *
 * Created on 6/7/23 11:36 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "dds/dds.h"

// #include "HelloWorldData.h"
#include "String.h"

#include <stdio.h>
#include <stdlib.h>

#include <thread>
#include <chrono>

int main(int argc, char** argv) {
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t writer;
  dds_return_t rc;
  std_msgs_msg_dds__String_ msg;
  uint32_t status = 0;
  (void)argc;
  (void)argv;

  /* Create a Participant. */
  participant = dds_create_participant(0, NULL, NULL);
  if (participant < 0)
    DDS_FATAL("dds_create_participant: %s\n", dds_strretcode(-participant));

  /* Create a Topic. */
  topic = dds_create_topic(participant, &std_msgs_msg_dds__String__desc,
                           "rt/teststring", NULL, NULL);
  if (topic < 0) DDS_FATAL("dds_create_topic: %s\n", dds_strretcode(-topic));

  /* Create a Writer. */
  writer = dds_create_writer(participant, topic, NULL, NULL);
  if (writer < 0) DDS_FATAL("dds_create_writer: %s\n", dds_strretcode(-writer));

  printf("=== [Publisher]  Waiting for a reader to be discovered ...\n");
  fflush(stdout);

//  rc = dds_set_status_mask(writer, DDS_PUBLICATION_MATCHED_STATUS);
//  if (rc != DDS_RETCODE_OK)
//    DDS_FATAL("dds_set_status_mask: %s\n", dds_strretcode(-rc));
//
//  while (!(status & DDS_PUBLICATION_MATCHED_STATUS)) {
//    rc = dds_get_status_changes(writer, &status);
//    if (rc != DDS_RETCODE_OK)
//      DDS_FATAL("dds_get_status_changes: %s\n", dds_strretcode(-rc));
//
//    /* Polling sleep. */
//    dds_sleepfor(DDS_MSECS(20));
//  }

  /* Create a message to write. */
  msg.data = "Hello World - ROS2";

  printf("=== [Publisher]  Writing : ");
  printf("Message (%s)\n", msg.data);
  fflush(stdout);

  while (true) {
    rc = dds_write(writer, &msg);
    if (rc != DDS_RETCODE_OK) DDS_FATAL("dds_write: %s\n", dds_strretcode(-rc));
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  /* Deleting the participant will delete all its children recursively as well.
   */
  rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK) DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  return EXIT_SUCCESS;
}
