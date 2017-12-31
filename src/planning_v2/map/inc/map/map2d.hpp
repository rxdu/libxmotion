/* 
 * map2d.hpp
 * 
 * Created on: Jul 27, 2016
 * Description: 
 * 
 * Copyright (c) 2017 Ruixiang Du (rdu)
 */

#ifndef MAP2D_HPP
#define MAP2D_HPP

#include <memory>

#include "map/image_utils.hpp"

namespace librav
{
/*
 * 2D Map Coordinate System:
 *		y
 *		^
 *		|
 *		|
 *  	|
 *		|
 *		|
 *		|
 * 	origin ------------------> x
 */
class Map2D
{
  public:
	Map2D(std::string path) : map_path_(path){};
	Map2D() = default;
	~Map2D() = default;

	// Map images
	cv::Mat raw_image_;
	cv::Mat map_image_;

	bool IsMapValid();

	// Helper functions to setup a map
	void LoadMapImage();
	void LoadMapImage(std::string map_path);
	void SetWorldSize(double world_x, double world_y);

	// Map queries


  private:
  	std::string map_path_;
	
	bool image_loaded_ = false;
	bool dimension_set_ = false;
	bool offset_set_ = false;

	// Dimensions
	int32_t map_size_x_; // in pixels
	int32_t map_size_y_;
	double world_size_x_; // in meters
	double world_size_y_;	

	double scale_x_; 	// world/map: meter per pixel
	double scale_y_;

	// Offset relative to bottom left corner
	double origin_offset_x_; // in meters
	double origin_offset_y_;	
};
}

#endif /* MAP2D_HPP */
