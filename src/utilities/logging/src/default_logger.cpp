/*
 * default_logger.cpp
 *
 * Created on 4/20/24 10:34 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "logging/details/default_logger.hpp"

#include "logging_utils.hpp"

namespace xmotion {
std::shared_ptr<DefaultLogger> DefaultLogger::GetInstance() {
  static std::atomic<bool> first_time_run{true};
  static std::shared_ptr<DefaultLogger> logger =
      std::shared_ptr<DefaultLogger>(new DefaultLogger());
  if (first_time_run.exchange(false)) {
    logger->Initialize(GetCurrentProcessName(), "%^[%l] [%E.%F] [%n]: %v%$",
                       ".log");
  }
  return logger;
}
}  // namespace xmotion