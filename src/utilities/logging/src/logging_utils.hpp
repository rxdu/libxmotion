/*
 * logging_utils.hpp
 *
 * Created on 4/20/24 11:05 AM
 * Description:
 *
 * Copyright (c) 2024 Ruixiang Du (rdu)
 */

#ifndef XMOTION_SRC_UTILITIES_LOGGING_SRC_LOGGING_UTILS_HPP_
#define XMOTION_SRC_UTILITIES_LOGGING_SRC_LOGGING_UTILS_HPP_

#include <ctime>
#include <cstdlib>
#include <string>

#include "logging/details/logger_interface.hpp"

namespace xmotion {
std::string GetEnvironmentVariable(const std::string &variableName);

inline std::string GetDefaultRootPath() {
  char *home_path = std::getenv("HOME");
  std::string hm(home_path);
  return hm + "/.xmotion";
}

inline std::string GetDefaultLogPath() {
  std::string log_path = GetEnvironmentVariable(log_folder_env_var_name);
  if (log_path.empty()) return GetDefaultRootPath() + "/log";
  return log_path;
}

// reference:
// https://stackoverflow.com/questions/22318389/pass-system-date-and-time-as-a-filename-in-c
inline std::string CreateLogNameWithFullPath(std::string prefix,
                                             std::string suffix) {
  time_t t = time(0);  // get time now
  struct tm *now = localtime(&t);

  char buffer[80];
  strftime(buffer, 80, "%Y%m%d-%H%M%S", now);
  std::string time_stamp(buffer);

  strftime(buffer, 80, "%Y%m%d", now);
  std::string subfolder(buffer);

  std::string filename = GetDefaultLogPath() + "/" + subfolder + "/" + prefix +
                         "-" + time_stamp + suffix;
  return filename;
}

#if defined(_WIN32) || defined(_WIN64)
#include <Windows.h>

// For Windows
std::string GetCurrentProcessName() {
  char buffer[MAX_PATH];
  GetModuleFileNameA(NULL, buffer, MAX_PATH);
  std::string fullPath(buffer);
  size_t pos = fullPath.find_last_of("\\/");
  return fullPath.substr(pos + 1);
}
#elif defined(__linux__)
#include <unistd.h>

// For Linux
inline std::string GetCurrentProcessName() {
  char buffer[1024];
  ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (len != -1) {
    buffer[len] = '\0';
    std::string fullPath(buffer);
    size_t pos = fullPath.find_last_of("/");
    return fullPath.substr(pos + 1);
  }
  return "";
}
#elif defined(__APPLE__)
#include <libproc.h>

// For macOS
inline std::string GetCurrentProcessName() {
  char buffer[1024];
  int ret = proc_name(getpid(), buffer, sizeof(buffer));
  if (ret > 0) {
    return buffer;
  }
  return "";
}
#else
// Unsupported platform
inline std::string GetCurrentProcessName() { return "Unsupported Platform"; }
#endif

#if defined(_WIN32)
#include <Windows.h>
#else
#include <unistd.h>  // For POSIX systems
#endif

inline std::string GetEnvironmentVariable(const std::string &variableName) {
#if defined(_WIN32)
  // For Windows
  char buffer[32767];  // The maximum size for environment variables on Windows
  DWORD bufferSize =
      GetEnvironmentVariableA(variableName.c_str(), buffer, sizeof(buffer));
  if (bufferSize > 0 && bufferSize <= sizeof(buffer)) {
    return std::string(buffer);
  }
  return "";
#else
  // For POSIX systems (Linux, macOS, etc.)
  char *envValue = getenv(variableName.c_str());
  if (envValue != nullptr) {
    return std::string(envValue);
  }
  return "";
#endif
}
}  // namespace xmotion

#endif  // XMOTION_SRC_UTILITIES_LOGGING_SRC_LOGGING_UTILS_HPP_
