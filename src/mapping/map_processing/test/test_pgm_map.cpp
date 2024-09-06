/*
 * test_pgm_map.cpp
 *
 * Created on: Jan 28, 2021 22:01
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include "map_processing/pgm_map.hpp"

using namespace xmotion;

int main(int argc, char *argv[]) {
  std::string file_path = "../data/pgm_map/pgm_scale.pgm";

  PgmMap::Metadata metadata;
  metadata.image = file_path;

  PgmMap map(metadata);
  if (map.LoadData()) {
    std::cout << "Map loaded successfully" << std::endl;
  } else {
    std::cout << "Failed to load map" << std::endl;
    return -1;
  }

  //  std::string save_dir = ".";
  //  std::string save_file = "pgm_map_test.pgm";
  //  if (map.SaveToFile(save_dir, save_file)) {
  //    std::cout << "Map saved successfully" << std::endl;
  //  } else {
  //    std::cout << "Failed to save map" << std::endl;
  //  }

  cv::Mat img = map.ToCvMat();
  cv::imshow("PGM Map", img);
  int k = cv::waitKey(0);  // Wait for a keystroke in the window

  return 0;
}
