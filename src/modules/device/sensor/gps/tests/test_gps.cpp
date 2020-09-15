/*
 * test_interface.cpp
 *
 *  Created on: Dec 25, 2016
 *      Author: rdu
 */

#include <iostream>

#include "async_serial/async_serial.hpp"
#include "nmeaparse/nmea.h"

using namespace librav;
using namespace nmea;

NMEAParser* parser;

void parse_buffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // std::cout << "parser called" << std::endl;

    // mavlink::mavlink_status_t status;
    // mavlink::mavlink_message_t message;

    parser->readBuffer(buf, bytes_received);

    // for (; bytes_received > 0; bytes_received--)
    // {
    //     auto c = *buf++;

    //     // 	// based on mavlink_parse_char()
    //     // 	auto msg_received = static_cast<Framing>(mavlink::mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
    //     // 	if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
    //     // 		mavlink::_mav_parse_error(&m_status);
    //     // 		m_status.msg_received = mavlink::MAVLINK_FRAMING_INCOMPLETE;
    //     // 		m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_IDLE;
    //     // 		if (c == MAVLINK_STX) {
    //     // 			m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_GOT_STX;
    //     // 			m_buffer.len = 0;
    //     // 			mavlink::mavlink_start_checksum(&m_buffer);
    //     // 		}
    //     // 	}

    //     // 	if (msg_received != Framing::incomplete) {
    //     // 		log_recv(pfx, message, msg_received);

    //     // 		if (message_received_cb)
    //     // 			message_received_cb(&message, msg_received);
    //     // 	}
    // }
}

int main(int argc, char *argv[])
{
    // ------------  CONFIGURE GPS SERVICE  ------------------- //

    // Create a GPS service that will keep track of the fix data.
    parser = new NMEAParser();
    parser->log = false;
    GPSService gps(*parser);

    // Handle events when the lock state changes
    gps.onLockStateChanged += [](bool newlock) {
        if (newlock)
        {
            std::cout << "\t\t\tGPS aquired LOCK!" << std::endl;
        }
        else
        {
            std::cout << "\t\t\tGPS lost lock. Searching..." << std::endl;
        }
    };

    // Handle any changes to the GPS Fix... This is called after onSentence
    gps.onUpdate += [&gps]() {
        std::cout << "\t\t\tPosition: " << gps.fix.latitude << "'N, " << gps.fix.longitude << "'E" << std::endl
                  << std::endl;
    };

    // (optional) - Handle events when the parser receives each sentence
    parser->onSentence += [&gps](const NMEASentence &n) {
        std::cout << "Received " << (n.checksumOK() ? "good" : "bad") << " GPS Data: " << n.name << std::endl;
    };

    //-----------------------------------------------------------------------------//

    // ASyncSerial::Ptr serial = ASyncSerial::open_url("/dev/ttyUSB0:115200");
    std::shared_ptr<ASyncSerial> serial = std::make_shared<ASyncSerial>("/dev/ttyUSB0", 115200);

    serial->set_receive_callback(parse_buffer);

    if (serial->is_open())
        std::cout << "serial port opened" << std::endl;

    while (1)
    {
    }
}
