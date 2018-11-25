/* 
 * gps_receiver.cpp
 * 
 * Created on: Nov 24, 2018 03:42
 * Description: 
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#include "gps/gps_receiver.hpp"

#include <string>
#include <functional>
#include <iostream>

using namespace librav;
using namespace nmea;

GPSReceiver::GPSReceiver(std::string device, unsigned int baudrate) : serial_port_(std::make_shared<ASyncSerial>(device, baudrate)),
                                                                      gps_service_(parser_)
{
    ConfigureSensor();
}

GPSReceiver::GPSReceiver(std::string device, unsigned int baudrate, std::shared_ptr<LCMLink> lcm) : lcm_(lcm),
                                                                                                    serial_port_(std::make_shared<ASyncSerial>(device, baudrate)),
                                                                                                    gps_service_(parser_)
{
    ConfigureSensor();
}

void GPSReceiver::ConfigureSensor()
{
    // connect serial port and parser
    serial_port_->set_receive_callback(std::bind(&GPSReceiver::ParseMessageBuffer, this,
                                                 std::placeholders::_1,
                                                 std::placeholders::_2,
                                                 std::placeholders::_3));

    // disable log of parser
    parser_.log = false;

    // handle events when the lock state changes
    gps_service_.onLockStateChanged += [](bool new_lock_state) {
        if (new_lock_state)
            std::cout << "GPS aquired LOCK!" << std::endl;
        else
            std::cout << "GPS lost lock. Searching..." << std::endl;
    };

    // register handlers
    parser_.setSentenceHandler("GNGGA", std::bind(&GPSReceiver::GNGGASentenceHandler, this,
                                                  std::placeholders::_1));
    parser_.setSentenceHandler("GNRMC", std::bind(&GPSReceiver::GNRMCSentenceHandler, this,
                                                  std::placeholders::_1));
}

void GPSReceiver::ParseMessageBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    parser_.readBuffer(buf, bytes_received);
}

void GPSReceiver::GNGGASentenceHandler(const NMEASentence &msg)
{
    // std::cout << msg << "\n----------" << std::endl;
}

void GPSReceiver::GNRMCSentenceHandler(const NMEASentence &msg)
{
    std::cout << msg << "\n----------" << std::endl;

    if (lcm_ != nullptr)
    {
        librav_lcm_msgs::NavSatFix nav_msg;

        nav_msg.latitude = ConvertDMSToDecimal(msg.parameters[2]);
        nav_msg.longitude = ConvertDMSToDecimal(msg.parameters[4]);

        // std::cout << "lat: " << nav_msg.latitude << " , lon: " << nav_msg.longitude << std::endl;

        lcm_->publish("NAV_SAT_FIX", &nav_msg);
    }
}

double GPSReceiver::ConvertDMSToDecimal(std::string str)
{
    double dms = std::stod(str);
    double degree = static_cast<int64_t>(dms / 100.0);
    double min = dms - degree * 100.0;

    double dec = degree + (min / 60);

    // std::cout << "degree: " << degree << " , min: " << min << " , res = " << dec << std::endl;

    return dec;
}
