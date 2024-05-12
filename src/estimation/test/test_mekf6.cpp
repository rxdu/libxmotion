#include <iostream>

#include "estimation/mekf6.hpp"

using namespace xmotion;

int main(int argc, char* argv[]) {
  std::cout << "Hello, World!" << std::endl;

  Mekf6::Params params;
  Mekf6 mekf;
  mekf.Initialize(params);

  return 0;
}
