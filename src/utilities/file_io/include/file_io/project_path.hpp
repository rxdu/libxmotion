/*
 * project_path.hpp
 *
 * Created on: Nov 03, 2018 23:00
 * Description: convenience functions to get path of librav
 *
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef PROJECT_PATH_HPP
#define PROJECT_PATH_HPP

#include <cstdlib>
#include <string>

namespace robosw {
inline std::string GetProjectRootPath() {
  std::string res_path = "..";
  if (const char* env_p = std::getenv("ROBOTNAV_ROOT")) {
    res_path = std::string(env_p);
  }
  return res_path;
}

inline std::string GetDataFolderPath() {
  return GetProjectRootPath() + "/data";
}

inline std::string GetLogFolderPath() { return GetDataFolderPath() + "/log"; }
}  // namespace robosw

#endif /* PROJECT_PATH_HPP */
