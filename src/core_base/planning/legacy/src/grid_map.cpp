/* 
 * grid_map.cpp
 * 
 * Created on: Jul 27, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#include "map/grid_map.hpp"

using namespace librav;

bool GridMap::IsMapValid()
{
    bool is_valid = image_loaded_ && dimension_set_ && offset_set_;

    if (is_valid)
    {
        return true;
    }
    else
    {
        std::cout << "Map loaded: " << image_loaded_ << " , dimension set: " << dimension_set_ << " , offset_set: " << offset_set_ << std::endl;
        return false;
    }
}

void GridMap::LoadMapImage()
{
    cv::Mat image;

    // set image_loaded_ to be true only if an image is successfully loaded
    image_loaded_ = false;
    if (!map_path_.empty())
    {
        // read map image
        raw_image_ = imread(map_path_, cv::IMREAD_GRAYSCALE);

        if (raw_image_.data)
        {
            map_size_x_ = raw_image_.cols;
            map_size_y_ = raw_image_.rows;
            image_loaded_ = true;
        }
    }
}

void GridMap::LoadMapImage(std::string map_path)
{
    map_path_ = map_path;
    LoadMapImage();
}

void GridMap::SetWorldSize(double world_x, double world_y)
{
    if (image_loaded_)
    {
        world_size_x_ = world_x;
        world_size_y_ = world_y;

        scale_x_ = world_size_x_ / static_cast<double>(map_size_x_);
        scale_y_ = world_size_y_ / static_cast<double>(map_size_y_);

        dimension_set_ = true;
    }
}