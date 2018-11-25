/* 
 * gps_receiver.hpp
 * 
 * Created on: Nov 23, 2018 23:17
 * Description: a gps receiver class that talks with a GPS
 *              sensor via serial port and parses NEMA msgs
 * 
 * Reference: https://www.gpsinformation.org/dale/nmea.htm
 * 
 * Copyright (c) 2018 Ruixiang Du (rdu)
 */

#ifndef GPS_RECEIVER_HPP
#define GPS_RECEIVER_HPP

#include <memory>

#include "datalink/lcm_link.hpp"

#include "async_serial/async_serial.hpp"
#include "nmeaparse/nmea.h"

namespace librav
{
class GPSReceiver
{
  public:
    GPSReceiver() = delete;
    GPSReceiver(std::string device, unsigned int baudrate);
    GPSReceiver(std::string device, unsigned int baudrate, std::shared_ptr<LCMLink> lcm);
    ~GPSReceiver() = default;

    // do not allow copy
    GPSReceiver(const GPSReceiver &other) = delete;
    GPSReceiver &operator=(const GPSReceiver &other) = delete;

    bool Connected() { return serial_port_->is_open(); }
    void ParseMessageBuffer(uint8_t *buf, const size_t bufsize, size_t bytes_received);

  private:
    std::shared_ptr<LCMLink> lcm_;
    std::shared_ptr<ASyncSerial> serial_port_;
    nmea::NMEAParser parser_;
    nmea::GPSService gps_service_;

    void ConfigureSensor();

    void GNGGASentenceHandler(const nmea::NMEASentence &msg);
    void GNRMCSentenceHandler(const nmea::NMEASentence &msg);

    double ConvertDMSToDecimal(std::string str);
};
} // namespace librav

#endif /* GPS_RECEIVER_HPP */
