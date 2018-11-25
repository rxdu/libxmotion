/*
 * test_interface.cpp
 *
 *  Created on: Dec 25, 2016
 *      Author: rdu
 */

#include <iostream>

#include "mavconn/interface.h"
#include "mavconn/serial.h"

using namespace mavconn;

int main(int argc, char* argv[])
{
	MAVConnInterface::Ptr serial;
	MAVConnSerial *serial_p;

	serial = MAVConnInterface::open_url("/dev/ttyUSB0:115200");
	serial_p = dynamic_cast<MAVConnSerial*>(serial.get());

	if(serial->is_open())
		std::cout << "serial port opened" << std::endl;

	while(1) {

	}
}


