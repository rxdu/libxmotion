/* 
 * logging_utils.hpp
 * 
 * Created on: Dec 28, 2018 11:30
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef LOGGING_UTILS_HPP
#define LOGGING_UTILS_HPP

#include <cstdlib>
#include <string>

namespace xmotion {
inline std::string GetProjectRootPath() {
  char *home_path;
  home_path = std::getenv("HOME");
  std::string log_path;
  if (home_path != NULL) {
    std::string hm(home_path);
    log_path = hm + "/.xmotion";
  } else {
    // default path
    log_path = "/home/rdu/.xmotion";
  }
  return log_path;
}

inline std::string GetLogFolderPath() {
  return GetProjectRootPath() + "/log";
}

// reference: https://stackoverflow.com/questions/22318389/pass-system-date-and-time-as-a-filename-in-c
inline std::string CreateLogFileName(std::string prefix, std::string path) {
  time_t t = time(0); // get time now
  struct tm *now = localtime(&t);

  char buffer[80];
  strftime(buffer, 80, "%Y%m%d%H%M%S", now);
  std::string time_stamp(buffer);

  std::string filename = path + "/" + prefix + "." + time_stamp + ".csv";
  return filename;
}
} // namespace xmotion

#endif /* LOGGING_UTILS_HPP */
