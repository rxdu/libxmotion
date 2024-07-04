/*
 * test_quadruped_mode.cpp
 *
 * Created on 7/3/24 11:34 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <csignal>
#include <atomic>
#include <iostream>

#include "time/stopwatch.hpp"

#include "quadruped/control_mode_fsm.hpp"

using namespace xmotion;

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
  if (signal == SIGINT) {
    keep_running = false;
  }
}

int main(int argc, char **argv) {
  // register signal handler
  std::signal(SIGINT, [](int signal) -> void {
    if (signal == SIGINT) {
      keep_running = false;
    }
  });

  std::cout << "Hello, Quadruped!" << std::endl;

  ControlContext context;
  PassiveMode initial_state;
  ControlModeFsm fsm(std::move(initial_state), std::move(context));

  Timer timer;
  while (keep_running) {
    timer.reset();
    fsm.Update();
    //    timer.sleep_until_us(2000);
    timer.sleep_until_ms(1000);
  }

  return 0;
}