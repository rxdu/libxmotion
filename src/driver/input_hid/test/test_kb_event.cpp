/*
 * test_kb_event.cpp
 *
 * Created on 7/15/24 8:18 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <linux/input.h>
#include <event2/event.h>

#define KEYBOARD_DEVICE \
  "/dev/input/event3"  // Adjust this path to your actual keyboard device

void handle_keyboard_event(evutil_socket_t fd, short events, void *arg) {
  struct input_event ev;
  ssize_t bytes_read = read(fd, &ev, sizeof(ev));
  if (bytes_read == (ssize_t)-1) {
    perror("read");
    return;
  } else if (bytes_read != sizeof(ev)) {
    fprintf(stderr, "Unexpected bytes read: %zd\n", bytes_read);
    return;
  }

  if (ev.type == EV_KEY) {
    printf("Key event: time %ld.%06ld, type %d, code %d, value %d\n",
           ev.time.tv_sec, ev.time.tv_usec, ev.type, ev.code, ev.value);
  }
}

int main() {
  // Open the keyboard device file
  int keyboard_fd = open(KEYBOARD_DEVICE, O_RDONLY | O_NONBLOCK);
  if (keyboard_fd < 0) {
    perror("open keyboard device");
    return 1;
  }

  // Initialize libevent
  struct event_base *base = event_base_new();
  if (!base) {
    fprintf(stderr, "Could not initialize libevent!\n");
    close(keyboard_fd);
    return 1;
  }

  // Create an event for the keyboard
  struct event *keyboard_event = event_new(
      base, keyboard_fd, EV_READ | EV_PERSIST, handle_keyboard_event, NULL);
  if (!keyboard_event) {
    fprintf(stderr, "Could not create keyboard event!\n");
    event_base_free(base);
    close(keyboard_fd);
    return 1;
  }

  // Add the event to the event base
  if (event_add(keyboard_event, NULL) < 0) {
    fprintf(stderr, "Could not add keyboard event!\n");
    event_free(keyboard_event);
    event_base_free(base);
    close(keyboard_fd);
    return 1;
  }

  // Dispatch events
  if (event_base_dispatch(base) < 0) {
    fprintf(stderr, "Error running event loop!\n");
    event_free(keyboard_event);
    event_base_free(base);
    close(keyboard_fd);
    return 1;
  }

  // Cleanup
  event_free(keyboard_event);
  event_base_free(base);
  close(keyboard_fd);
  return 0;
}
