/*
 * test_xlogger.cpp
 *
 * Created on 7/9/24 10:45 PM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include <thread>
#include "logging/xlogger.hpp"

int main(int argc, char* argv[]) {
  XLOG_INFO("This is an info message");
  XLOG_WARN("This is a warning message");
  XLOG_ERROR("This is an error message");
  XLOG_FATAL("This is a fatal message");

  std::thread t1([]() {
    //    while (1) {
    XLOG_INFO("This is an info message");
    XLOG_WARN("This is a warning message");
    XLOG_ERROR("This is an error message");
    XLOG_FATAL("This is a fatal message");
    //      std::this_thread::sleep_for(std::chrono::seconds(1));
    //    }
  });
  //
  //  while (1) {
  //    std::this_thread::sleep_for(std::chrono::seconds(1));
  //  }

  t1.join();

  return 0;
}