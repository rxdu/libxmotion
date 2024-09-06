/*
 * test_pgm_map.cpp
 *
 * Created on: Jan 28, 2021 22:01
 * Description:
 *
 * Copyright (c) 2021 Ruixiang Du (rdu)
 */

#include <iostream>

#include "map_processor/pgm_map.hpp"

using namespace xmotion;

int main(int argc, char *argv[]) {
  std::string file_path = "../data/pgm_map/pgm_scale.pgm";

  PgmMap::Metadata metadata;
  metadata.image = file_path;

  PgmMap map;
  if (map.LoadData()) {
    std::cout << "Map loaded successfully" << std::endl;
  } else {
    std::cout << "Failed to load map" << std::endl;
  }

  return 0;
}
