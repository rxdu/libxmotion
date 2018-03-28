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

#include <Eigen/Dense>

#define STB_IMAGE_IMPLEMENTATION
#include "image/stb_image.h"

namespace librav
{
namespace ImageIO
{
void DisplayImage();
}
}

#endif /* IMAGE_IO_HPP */
