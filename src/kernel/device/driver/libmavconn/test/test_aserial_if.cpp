/*
 * test_interface.cpp
 *
 *  Created on: Dec 25, 2016
 *      Author: rdu
 */

#include <iostream>

#include "mavconn/async_serial.hpp"

using namespace mavconn;

int main(int argc, char* argv[])
{
	ASyncSerial::Ptr serial;

	serial = ASyncSerial::open_url("/dev/ttyUSB0:115200");

	if(serial->is_open())
		std::cout << "serial port opened" << std::endl;

	while(1) {

	}
}


