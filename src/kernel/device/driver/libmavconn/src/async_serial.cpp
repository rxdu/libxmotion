/**
 * @brief MAVConn Serial link class
 * @file serial.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2013,2014,2015,2016,2018 Vladimir Ermakov, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <cassert>
#include <iostream>

// #include "mavconn/thread_utils.h"
#include "mavconn/async_serial.hpp"

#if defined(__linux__)
#include <linux/serial.h>
#endif

namespace mavconn
{

namespace
{
template <typename... Args>
std::string format(const std::string &fmt, Args... args)
{
    // C++11 specify that string store elements continously
    std::string ret;

    auto sz = std::snprintf(nullptr, 0, fmt.c_str(), args...);
    ret.reserve(sz + 1);
    ret.resize(sz); // to be sure there have room for \0
    std::snprintf(&ret.front(), ret.capacity() + 1, fmt.c_str(), args...);
    return ret;
}

template <typename... Args>
bool set_this_thread_name(const std::string &name, Args &&... args)
{
    auto new_name = format(name, std::forward<Args>(args)...);

#ifdef __APPLE__
    return pthread_setname_np(new_name.c_str()) == 0;
#else
    pthread_t pth = pthread_self();
    return pthread_setname_np(pth, new_name.c_str()) == 0;
#endif
}

/**
 * Parse host:port pairs
 */
void url_parse_host(std::string host,
                    std::string &host_out, int &port_out,
                    const std::string def_host, const int def_port)
{
    std::string port;

    auto sep_it = std::find(host.begin(), host.end(), ':');
    if (sep_it == host.end())
    {
        // host
        if (!host.empty())
        {
            host_out = host;
            port_out = def_port;
        }
        else
        {
            host_out = def_host;
            port_out = def_port;
        }
        return;
    }

    if (sep_it == host.begin())
    {
        // :port
        host_out = def_host;
    }
    else
    {
        // host:port
        host_out.assign(host.begin(), sep_it);
    }

    port.assign(sep_it + 1, host.end());
    port_out = std::stoi(port);
}

/**
 * Parse ?ids=sid,cid
 */
void url_parse_query(std::string query)
{
    const std::string ids_end("ids=");
    std::string sys, comp;

    if (query.empty())
        return;

    auto ids_it = std::search(query.begin(), query.end(),
                              ids_end.begin(), ids_end.end());
    if (ids_it == query.end())
    {
        // CONSOLE_BRIDGE_logWarn(PFX "URL: unknown query arguments");
        return;
    }

    std::advance(ids_it, ids_end.length());
    auto comma_it = std::find(ids_it, query.end(), ',');
    if (comma_it == query.end())
    {
        // CONSOLE_BRIDGE_logError(PFX "URL: no comma in ids= query");
        return;
    }

    sys.assign(ids_it, comma_it);
    comp.assign(comma_it + 1, query.end());

    // sysid = std::stoi(sys);
    // compid = std::stoi(comp);

    // CONSOLE_BRIDGE_logDebug(PFX "URL: found system/component id = [%u, %u]", sysid, compid);
}
} // namespace

using asio::buffer;
using asio::io_service;
using std::error_code;

#define PFX "mavconn: serial"
#define PFXd PFX "%zu: "

std::atomic<size_t> ASyncSerial::conn_id_counter{0};

ASyncSerial::ASyncSerial(std::string device, unsigned baudrate, bool hwflow) : tx_total_bytes(0),
                                                                               rx_total_bytes(0),
                                                                               last_tx_total_bytes(0),
                                                                               last_rx_total_bytes(0),
                                                                               last_iostat(steady_clock::now()),
                                                                               tx_in_progress(false),
                                                                               tx_q{},
                                                                               rx_buf{},
                                                                               io_service(),
                                                                               serial_dev(io_service)
{
    conn_id = conn_id_counter.fetch_add(1);

    using SPB = asio::serial_port_base;

    // CONSOLE_BRIDGE_logInform(PFXd "device: %s @ %d bps", conn_id, device.c_str(), baudrate);

    try
    {
        serial_dev.open(device);

        // Set baudrate and 8N1 mode
        serial_dev.set_option(SPB::baud_rate(baudrate));
        serial_dev.set_option(SPB::character_size(8));
        serial_dev.set_option(SPB::parity(SPB::parity::none));
        serial_dev.set_option(SPB::stop_bits(SPB::stop_bits::one));

        // Flow control setting in older versions of Boost.ASIO is broken, use workaround (below) for now.
        serial_dev.set_option(SPB::flow_control((hwflow) ? SPB::flow_control::hardware : SPB::flow_control::none));

#if defined(__linux__)
        // Enable low latency mode on Linux
        {
            int fd = serial_dev.native_handle();

            struct serial_struct ser_info;
            ioctl(fd, TIOCGSERIAL, &ser_info);

            ser_info.flags |= ASYNC_LOW_LATENCY;

            ioctl(fd, TIOCSSERIAL, &ser_info);
        }
#endif
    }
    catch (std::system_error &err)
    {
        throw DeviceError("serial", err);
    }

    // NOTE: shared_from_this() should not be used in constructors

    // give some work to io_service before start
    io_service.post(std::bind(&ASyncSerial::do_read, this));

    // run io_service for async io
    io_thread = std::thread([this]() {
        set_this_thread_name("mserial%zu", conn_id);
        io_service.run();
    });
}

ASyncSerial::~ASyncSerial()
{
    close();
}

void ASyncSerial::close()
{
    lock_guard lock(mutex);
    if (!is_open())
        return;

    serial_dev.cancel();
    serial_dev.close();

    io_service.stop();

    if (io_thread.joinable())
        io_thread.join();

    io_service.reset();

    if (port_closed_cb)
        port_closed_cb();
}

ASyncSerial::IOStat ASyncSerial::get_iostat()
{
    std::lock_guard<std::recursive_mutex> lock(iostat_mutex);
    IOStat stat;

    stat.tx_total_bytes = tx_total_bytes;
    stat.rx_total_bytes = rx_total_bytes;

    auto d_tx = stat.tx_total_bytes - last_tx_total_bytes;
    auto d_rx = stat.rx_total_bytes - last_rx_total_bytes;
    last_tx_total_bytes = stat.tx_total_bytes;
    last_rx_total_bytes = stat.rx_total_bytes;

    auto now = steady_clock::now();
    auto dt = now - last_iostat;
    last_iostat = now;

    float dt_s = std::chrono::duration_cast<std::chrono::seconds>(dt).count();

    stat.tx_speed = d_tx / dt_s;
    stat.rx_speed = d_rx / dt_s;

    return stat;
}

void ASyncSerial::iostat_tx_add(size_t bytes)
{
    tx_total_bytes += bytes;
}

void ASyncSerial::iostat_rx_add(size_t bytes)
{
    rx_total_bytes += bytes;
}

void ASyncSerial::send_bytes(const uint8_t *bytes, size_t length)
{
    if (!is_open())
    {
        // CONSOLE_BRIDGE_logError(PFXd "send: channel closed!", conn_id);
        return;
    }

    {
        lock_guard lock(mutex);

        if (tx_q.size() >= MAX_TXQ_SIZE)
            throw std::length_error("ASyncSerial::send_bytes: TX queue overflow");

        tx_q.emplace_back(bytes, length);
    }
    io_service.post(std::bind(&ASyncSerial::do_write, shared_from_this(), true));
}

void ASyncSerial::parse_buffer(const char *pfx, uint8_t *buf, const size_t bufsize, size_t bytes_received)
{
    // mavlink::mavlink_status_t status;
    // mavlink::mavlink_message_t message;

    // assert(bufsize >= bytes_received);

    // iostat_rx_add(bytes_received);
    // for (; bytes_received > 0; bytes_received--) {
    // 	auto c = *buf++;

    // 	// based on mavlink_parse_char()
    // 	auto msg_received = static_cast<Framing>(mavlink::mavlink_frame_char_buffer(&m_buffer, &m_status, c, &message, &status));
    // 	if (msg_received == Framing::bad_crc || msg_received == Framing::bad_signature) {
    // 		mavlink::_mav_parse_error(&m_status);
    // 		m_status.msg_received = mavlink::MAVLINK_FRAMING_INCOMPLETE;
    // 		m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_IDLE;
    // 		if (c == MAVLINK_STX) {
    // 			m_status.parse_state = mavlink::MAVLINK_PARSE_STATE_GOT_STX;
    // 			m_buffer.len = 0;
    // 			mavlink::mavlink_start_checksum(&m_buffer);
    // 		}
    // 	}

    // 	if (msg_received != Framing::incomplete) {
    // 		log_recv(pfx, message, msg_received);

    // 		if (message_received_cb)
    // 			message_received_cb(&message, msg_received);
    // 	}
    // }
}

void ASyncSerial::do_read(void)
{
    auto sthis = shared_from_this();
    serial_dev.async_read_some(
        buffer(rx_buf),
        [sthis](error_code error, size_t bytes_transferred) {
            if (error)
            {
                // CONSOLE_BRIDGE_logError(PFXd "receive: %s", sthis->conn_id, error.message().c_str());
                sthis->close();
                return;
            }

            sthis->parse_buffer(PFX, sthis->rx_buf.data(), sthis->rx_buf.size(), bytes_transferred);
            sthis->do_read();
        });

    std::cout << rx_buf.data() << std::endl;
}

void ASyncSerial::do_write(bool check_tx_state)
{
    if (check_tx_state && tx_in_progress)
        return;

    lock_guard lock(mutex);
    if (tx_q.empty())
        return;

    tx_in_progress = true;
    auto sthis = shared_from_this();
    auto &buf_ref = tx_q.front();
    serial_dev.async_write_some(
        buffer(buf_ref.dpos(), buf_ref.nbytes()),
        [sthis, &buf_ref](error_code error, size_t bytes_transferred) {
            assert(bytes_transferred <= buf_ref.len);

            if (error)
            {
                // CONSOLE_BRIDGE_logError(PFXd "write: %s", sthis->conn_id, error.message().c_str());
                sthis->close();
                return;
            }

            sthis->iostat_tx_add(bytes_transferred);
            lock_guard lock(sthis->mutex);

            if (sthis->tx_q.empty())
            {
                sthis->tx_in_progress = false;
                return;
            }

            buf_ref.pos += bytes_transferred;
            if (buf_ref.nbytes() == 0)
            {
                sthis->tx_q.pop_front();
            }

            if (!sthis->tx_q.empty())
                sthis->do_write(false);
            else
                sthis->tx_in_progress = false;
        });
}

static ASyncSerial::Ptr url_parse_serial(
    std::string path, std::string query, bool hwflow)
{
    std::string file_path;
    int baudrate;

    // /dev/ttyACM0:57600
    url_parse_host(path, file_path, baudrate, ASyncSerial::DEFAULT_DEVICE, ASyncSerial::DEFAULT_BAUDRATE);
    url_parse_query(query);

    return std::make_shared<ASyncSerial>(file_path, baudrate, hwflow);
}

ASyncSerial::Ptr ASyncSerial::open_url(std::string url)
{
    /* Based on code found here:
	 * http://stackoverflow.com/questions/2616011/easy-way-to-parse-a-url-in-c-cross-platform
	 */

    const std::string proto_end("://");
    std::string proto;
    std::string host;
    std::string path;
    std::string query;

    auto proto_it = std::search(
        url.begin(), url.end(),
        proto_end.begin(), proto_end.end());
    if (proto_it == url.end())
    {
        // looks like file path
        // CONSOLE_BRIDGE_logDebug(PFX "URL: %s: looks like file path", url.c_str());
        return url_parse_serial(url, "", false);
    }

    // copy protocol
    proto.reserve(std::distance(url.begin(), proto_it));
    std::transform(url.begin(), proto_it,
                   std::back_inserter(proto),
                   std::ref(tolower));

    // copy host
    std::advance(proto_it, proto_end.length());
    auto path_it = std::find(proto_it, url.end(), '/');
    std::transform(proto_it, path_it,
                   std::back_inserter(host),
                   std::ref(tolower));

    // copy path, and query if exists
    auto query_it = std::find(path_it, url.end(), '?');
    path.assign(path_it, query_it);
    if (query_it != url.end())
        ++query_it;
    query.assign(query_it, url.end());

    if (proto == "serial")
        return url_parse_serial(path, query, false);
    else if (proto == "serial-hwfc")
        return url_parse_serial(path, query, true);
    else
        throw DeviceError("url", "Unknown URL type");
}
} // namespace mavconn
