#include <iostream>

#include "socketcan/socketcan.h"

/**
 * Transmits all frames from the TX queue, receives up to one frame.
 */
void processTxRxOnce(SocketCANInstance *socketcan, int timeout_msec)
{
    // Transmitting
    CanardCANFrame tx_frame;
    tx_frame.id = 1234;
    tx_frame.data[0] = 1;
    tx_frame.data[1] = 2;
    tx_frame.data[2] = 3;
    tx_frame.data_len = 3;
    const int tx_res = socketcanTransmit(socketcan, &tx_frame, 0);
    if (tx_res < 0) // Failure - drop the frame and report
        std::cout << "Transmit error " << tx_res << ", frame dropped" << std::endl;
    else if (tx_res > 0) // Success - just drop the frame
        std::cout << "Transmit successfully" << std::endl;
    // Timeout - just exit and try again later

    // Receiving
    CanardCANFrame rx_frame;
    const int rx_res = socketcanReceive(socketcan, &rx_frame, timeout_msec);
    if (rx_res < 0) // Failure - report
    {
        std::cout << "Receive error " << tx_res << ", frame dropped " << std::endl;
    }
    else if (rx_res > 0) // Success - process the frame
    {
        std::cout << "Receive successfully" << std::endl;
    }
    else
    {
        ; // Timeout - nothing to do
    }
}

int main(int argc, char *argv[])
{
    SocketCANInstance socketcan;
    std::string ifacename = "can0";
    int res = socketcanInit(&socketcan, ifacename.c_str());
    if (res < 0)
    {
        std::cerr << "Failed to open CAN iface " << ifacename << std::endl;
        return 1;
    }

    while(true)
    {
        processTxRxOnce(&socketcan, 10);
    }

    return 0;
}