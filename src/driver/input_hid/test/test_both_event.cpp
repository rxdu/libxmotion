/*
 * test_both_event.cpp
 *
 * Created on 7/15/24 8:20 PM
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
#include <pthread.h>
#include <event2/event.h>

#define KEYBOARD_DEVICE "/dev/input/event3"  // Adjust this path to your actual keyboard device
#define JOYSTICK_DEVICE "/dev/input/event22"  // Adjust this path to your actual joystick device

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
    printf("Keyboard event: time %ld.%06ld, type %d, code %d, value %d\n",
           ev.time.tv_sec, ev.time.tv_usec, ev.type, ev.code, ev.value);
  }
}

void handle_joystick_event(evutil_socket_t fd, short events, void *arg) {
  struct input_event ev;
  ssize_t bytes_read = read(fd, &ev, sizeof(ev));
  if (bytes_read == (ssize_t)-1) {
    perror("read");
    return;
  } else if (bytes_read != sizeof(ev)) {
    fprintf(stderr, "Unexpected bytes read: %zd\n", bytes_read);
    return;
  }

  printf("Joystick event: time %ld.%06ld, type %d, code %d, value %d\n",
         ev.time.tv_sec, ev.time.tv_usec, ev.type, ev.code, ev.value);
}

void *event_loop_thread(void *arg) {
  struct event_base *base = (struct event_base *)arg;
  event_base_dispatch(base);
  return NULL;
}

int main() {
  // Open the keyboard device file
  int keyboard_fd = open(KEYBOARD_DEVICE, O_RDONLY | O_NONBLOCK);
  if (keyboard_fd < 0) {
    perror("open keyboard device");
    return 1;
  }

  // Open the joystick device file
  int joystick_fd = open(JOYSTICK_DEVICE, O_RDONLY | O_NONBLOCK);
  if (joystick_fd < 0) {
    perror("open joystick device");
    close(keyboard_fd);
    return 1;
  }

  // Initialize libevent
  struct event_base *base = event_base_new();
  if (!base) {
    fprintf(stderr, "Could not initialize libevent!\n");
    close(keyboard_fd);
    close(joystick_fd);
    return 1;
  }

  // Create an event for the keyboard
  struct event *keyboard_event = event_new(base, keyboard_fd, EV_READ | EV_PERSIST, handle_keyboard_event, NULL);
  if (!keyboard_event) {
    fprintf(stderr, "Could not create keyboard event!\n");
    event_base_free(base);
    close(keyboard_fd);
    close(joystick_fd);
    return 1;
  }

  // Create an event for the joystick
  struct event *joystick_event = event_new(base, joystick_fd, EV_READ | EV_PERSIST, handle_joystick_event, NULL);
  if (!joystick_event) {
    fprintf(stderr, "Could not create joystick event!\n");
    event_free(keyboard_event);
    event_base_free(base);
    close(keyboard_fd);
    close(joystick_fd);
    return 1;
  }

  // Add the events to the event base
  if (event_add(keyboard_event, NULL) < 0) {
    fprintf(stderr, "Could not add keyboard event!\n");
    event_free(keyboard_event);
    event_free(joystick_event);
    event_base_free(base);
    close(keyboard_fd);
    close(joystick_fd);
    return 1;
  }

  if (event_add(joystick_event, NULL) < 0) {
    fprintf(stderr, "Could not add joystick event!\n");
    event_free(keyboard_event);
    event_free(joystick_event);
    event_base_free(base);
    close(keyboard_fd);
    close(joystick_fd);
    return 1;
  }

  // Create a thread to run the event loop
  pthread_t thread;
  if (pthread_create(&thread, NULL, event_loop_thread, base) != 0) {
    perror("pthread_create");
    event_free(keyboard_event);
    event_free(joystick_event);
    event_base_free(base);
    close(keyboard_fd);
    close(joystick_fd);
    return 1;
  }

  // Main loop doing other work
  while (1) {
    // Simulate doing other work
    printf("Main thread doing other work...\n");
    sleep(1);
  }

  // Cleanup (not reachable in this example)
  pthread_join(thread, NULL);
  event_free(keyboard_event);
  event_free(joystick_event);
  event_base_free(base);
  close(keyboard_fd);
  close(joystick_fd);

  return 0;
}
