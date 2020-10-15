/* 
 * folder_path.hpp
 * 
 * Created on: Nov 03, 2018 23:00
 * Description: convenience functions to get path of librav
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */ 

#ifndef FOLDER_PATH_HPP
#define FOLDER_PATH_HPP

#include <cstdlib>
#include <string>

namespace ivnav
{
namespace FolderPath
{
inline std::string GetDataFolderPath()
{
    char *home_path;
    home_path = std::getenv("HOME");
    std::string log_path;
    if (home_path != NULL)
    {
        std::string hm(home_path);
        log_path = hm + "/Workspace/librav/data";
    }
    else
    {
        // default path
        log_path = "/home/rdu/Workspace/librav/data";
    }
    return log_path;
}

inline std::string GetLogFolderPath()
{
    return GetDataFolderPath() + "/log";
}
} // namespace FolderPath
} // namespace ivnav

#endif /* FOLDER_PATH_HPP */
