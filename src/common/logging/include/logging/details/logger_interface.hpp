/*
 * logger_interface.hpp
 *
 * Created on 4/20/24 10:35 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_DETAILS_LOGGER_INTERFACE_HPP_
#define XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_DETAILS_LOGGER_INTERFACE_HPP_

#include <string>

namespace xmotion {
enum class LogLevel : int {
  kTrace = 0,
  kDebug = 1,
  kInfo = 2,
  kWarn = 3,
  kError = 4,
  kFatal = 5,
  kOff = 6
};

auto static constexpr log_level_env_var_name = "XLOG_LEVEL";
auto static constexpr log_logfile_env_var_name = "XLOG_ENABLE_LOGFILE";
auto static constexpr log_folder_env_var_name = "XLOG_FOLDER";
auto static constexpr default_log_level = LogLevel::kInfo;

class LoggerInterface {
 public:
  virtual ~LoggerInterface() = default;

 public:
  virtual void Initialize(std::string logger_name, std::string pattern,
                          std::string file_suffix) = 0;
  virtual void Terminate() {};

  virtual void SetLoggerLevel(LogLevel level) = 0;
  virtual LogLevel GetLoggerLevel() = 0;

  // log functions with the following signatures:
  /*
   * template <typename... LogArgs>
   * void Log(LogLevel level, LogArgs &&...args)
   *
   */
};
}  // namespace xmotion

#endif  // XMOTION_SRC_UTILITIES_LOGGING_INCLUDE_LOGGING_DETAILS_LOGGER_INTERFACE_HPP_
