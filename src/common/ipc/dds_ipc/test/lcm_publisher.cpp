/*
 * lcm_publisher.cpp
 *
 * Created on 6/5/23 10:54 PM
 * Description:
 *
 * Copyright (c) 2023 Ruixiang Du (rdu)
 */

#include <lcm/lcm-cpp.hpp>
#include "exlcm/example_t.hpp"

int main(int argc, char ** argv)
{
  lcm::LCM lcm;
  if(!lcm.good())
    return 1;

  exlcm::example_t my_data;
  my_data.timestamp = 0;

  my_data.position[0] = 1;
  my_data.position[1] = 2;
  my_data.position[2] = 3;

  my_data.orientation[0] = 1;
  my_data.orientation[1] = 0;
  my_data.orientation[2] = 0;
  my_data.orientation[3] = 0;

  my_data.num_ranges = 15;
  my_data.ranges.resize(my_data.num_ranges);
  for(int i = 0; i < my_data.num_ranges; i++)
    my_data.ranges[i] = i;

  my_data.name = "example string";
  my_data.enabled = true;

  lcm.publish("EXAMPLE", &my_data);

  return 0;
}