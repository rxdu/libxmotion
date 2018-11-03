/* 
 * file_location.hpp
 * 
 * Created on: Nov 03, 2018 11:04
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef FILE_LOCATION_HPP
#define FILE_LOCATION_HPP

#include <cstdlib>
#include <string>

namespace librav
{
namespace Location
{
inline std::string GetDefaultDataFolderPath()
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
} // namespace Folder
} // namespace librav

#endif /* FILE_LOCATION_HPP */
