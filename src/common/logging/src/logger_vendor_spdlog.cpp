/*
 * logger_vendor_spdlog.cpp
 *
 * Created on 4/20/24 10:34 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#include "logging/details/logger_vendor_spdlog.hpp"

#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "logging_utils.hpp"

namespace xmotion {
namespace {
spdlog::level::level_enum ToSpdlogLevel(LogLevel level) {
  switch (level) {
    case LogLevel::kTrace:
      return spdlog::level::trace;
    case LogLevel::kDebug:
      return spdlog::level::debug;
    case LogLevel::kInfo:
      return spdlog::level::info;
    case LogLevel::kWarn:
      return spdlog::level::warn;
    case LogLevel::kError:
      return spdlog::level::err;
    case LogLevel::kFatal:
      return spdlog::level::critical;
    default:
      return spdlog::level::off;
  }
}

LogLevel FromSpdlogLevel(spdlog::level::level_enum level) {
  switch (level) {
    case spdlog::level::trace:
      return LogLevel::kTrace;
    case spdlog::level::debug:
      return LogLevel::kDebug;
    case spdlog::level::info:
      return LogLevel::kInfo;
    case spdlog::level::warn:
      return LogLevel::kWarn;
    case spdlog::level::err:
      return LogLevel::kError;
    case spdlog::level::critical:
      return LogLevel::kFatal;
    default:
      return LogLevel::kOff;
  }
}
}  // namespace

void LoggerVendorSpdlog::Initialize(std::string logger_name,
                                    std::string pattern,
                                    std::string file_suffix) {
  // handle log level
  spdlog::level::level_enum log_level = ToSpdlogLevel(default_log_level);
  if (!GetEnvironmentVariable(log_level_env_var_name).empty()) {
    int log_level_int;
    try {
      log_level_int = std::stoi(GetEnvironmentVariable(log_level_env_var_name));
    } catch (std::invalid_argument& e) {
      log_level_int = ToSpdlogLevel(default_log_level);
    }
    if (log_level_int < 0 || log_level_int > 6)
      log_level_int = ToSpdlogLevel(default_log_level);
    log_level = ToSpdlogLevel(static_cast<LogLevel>(log_level_int));
  }

  bool enable_logfile = false;
  std::string enable_logfile_var =
      GetEnvironmentVariable(log_logfile_env_var_name);
  if (enable_logfile_var == "TRUE" || enable_logfile_var == "true" ||
      enable_logfile_var == "1") {
    enable_logfile = true;
  }

  // set up the logger
  sinks_.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  if (enable_logfile && log_level < spdlog::level::off) {
    sinks_.push_back(std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        CreateLogNameWithFullPath(logger_name, file_suffix)));
  }

  logger_ = std::make_shared<spdlog::logger>(logger_name, sinks_.begin(),
                                             sinks_.end());
  logger_->set_pattern(pattern);

  SetLoggerLevel(FromSpdlogLevel(log_level));
}

void LoggerVendorSpdlog::Terminate() { logger_->flush(); }

void LoggerVendorSpdlog::SetLoggerLevel(LogLevel level) {
  logger_->set_level(ToSpdlogLevel(level));
  logger_->flush_on(ToSpdlogLevel(level));
}

LogLevel LoggerVendorSpdlog::GetLoggerLevel() {
  return FromSpdlogLevel(logger_->level());
}
}  // namespace xmotion