/* 
 * image_io.hpp
 * 
 * Created on: Mar 23, 2018 17:27
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef IMAGE_IO_HPP
#define IMAGE_IO_HPP

#include <cstdint>

#include <eigen3/Eigen/Dense>

namespace rnav
{
struct MonoImageMatrix
{
    int32_t w = 0;
    int32_t h = 0;
    const int32_t d = 1;
    Eigen::MatrixXd data;
};

namespace ImageIO
{
MonoImageMatrix ReadImage(std::string file_name);
bool SaveToImage(const MonoImageMatrix& matrix, std::string file_name);
}
}

#endif /* IMAGE_IO_HPP */
