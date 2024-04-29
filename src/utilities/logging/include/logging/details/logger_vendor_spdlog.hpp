/*
 * @file logger_vendor_spdlog.hpp
 * @date 4/20/24
 * @brief
 *
 * @copyright Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_DETAILS_LOGGER_VENDOR_SPDLOG_HPP_
#define XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_DETAILS_LOGGER_VENDOR_SPDLOG_HPP_

#include <memory>
#include <string>
#include <vector>
#include <functional>
#include <iostream>

#include "spdlog/spdlog.h"

#include "logging/details/logger_interface.hpp"

namespace xmotion {
class LoggerVendorSpdlog : public LoggerInterface {
 public:
  LoggerVendorSpdlog() = default;
  virtual ~LoggerVendorSpdlog() = default;

  // public methods
  void Initialize(std::string logger_name, std::string pattern,
                  std::string file_suffix) override;
  void Deinitialize() override;

  void SetLoggerLevel(LogLevel level) override;
  LogLevel GetLoggerLevel() override;

  template <typename... LogArgs>
  void Log(LogLevel level, LogArgs&&... args) {
    switch (level) {
      case LogLevel::kTrace:
        logger_->trace(std::forward<LogArgs>(args)...);
        break;
      case LogLevel::kDebug:
        logger_->debug(std::forward<LogArgs>(args)...);
        break;
      case LogLevel::kInfo:
        logger_->info(std::forward<LogArgs>(args)...);
        break;
      case LogLevel::kWarn:
        logger_->warn(std::forward<LogArgs>(args)...);
        break;
      case LogLevel::kError:
        logger_->error(std::forward<LogArgs>(args)...);
        break;
      case LogLevel::kFatal:
        logger_->critical(std::forward<LogArgs>(args)...);
        break;
      default:
        break;
    }
  }

 protected:
  std::string process_name_ = "uninitialized";
  std::shared_ptr<spdlog::logger> logger_;
  std::vector<spdlog::sink_ptr> sinks_;
};
}  // namespace xmotion

#endif  // XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_DETAILS_LOGGER_VENDOR_SPDLOG_HPP_
