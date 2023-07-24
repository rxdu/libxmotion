/*
 * test_dds_sub.c
 *
 * Created on 6/8/23 9:33 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include "dds/dds.h"
// #include "HelloWorldData.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "String.h"

/* An array of one message (aka sample in dds terms) will be used. */
#define MAX_SAMPLES 1

int main(int argc, char **argv) {
  dds_entity_t participant;
  dds_entity_t topic;
  dds_entity_t reader;
  std_msgs_msg_dds__String_ *msg;
  void *samples[MAX_SAMPLES];
  dds_sample_info_t infos[MAX_SAMPLES];
  dds_return_t rc;
  dds_qos_t *qos;
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

  /* Create a reliable Reader. */
  qos = dds_create_qos();
  dds_qset_reliability(qos, DDS_RELIABILITY_RELIABLE, DDS_SECS(10));
  reader = dds_create_reader(participant, topic, qos, NULL);
  if (reader < 0) DDS_FATAL("dds_create_reader: %s\n", dds_strretcode(-reader));
  dds_delete_qos(qos);

  printf("\n=== [Subscriber] Waiting for a sample ...\n");
  fflush(stdout);

  /* Initialize sample buffer, by pointing the void pointer within
   * the buffer array to a valid sample memory location. */
  samples[0] = std_msgs_msg_dds__String___alloc();

  /* Poll until data has been read. */
  while (true) {
    /* Do the actual read.
     * The return value contains the number of read samples. */
    rc = dds_read(reader, samples, infos, MAX_SAMPLES, MAX_SAMPLES);
    if (rc < 0) DDS_FATAL("dds_read: %s\n", dds_strretcode(-rc));

    /* Check if we read some data and it is valid. */
    if ((rc > 0) && (infos[0].valid_data)) {
      /* Print Message. */
      msg = (std_msgs_msg_dds__String_ *)samples[0];
      printf("=== [Subscriber] Received : ");
      printf("Message (%s)\n", msg->data);
      fflush(stdout);
      break;
    } else {
      /* Polling sleep. */
      dds_sleepfor(DDS_MSECS(20));
    }
  }

  /* Free the data location. */
  std_msgs_msg_dds__String__free(samples[0], DDS_FREE_ALL);

  /* Deleting the participant will delete all its children recursively as well.
   */
  rc = dds_delete(participant);
  if (rc != DDS_RETCODE_OK) DDS_FATAL("dds_delete: %s\n", dds_strretcode(-rc));

  return EXIT_SUCCESS;
}