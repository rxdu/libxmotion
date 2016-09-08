/*
 * test_cubearray.cpp
 *
 *  Created on: Sep 8, 2016
 *      Author: rdu
 */

#include <iostream>

#include "cube_array/cube_array.h"

using namespace srcl_ctrl;

int main(int argc, char* argv[])
{
	CubeArray ca(3,3,3,0.5);

	std::cout << "\n----------------------------\n" << std::endl;
	std::cout << "id: " << ca.GetIDFromPosition(1.2, 1.2, 0.3) << std::endl;

	for(auto& nd : ca.GetNeighbours(13))
		std::cout << "neighbour: " << nd << std::endl;
}


