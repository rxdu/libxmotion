#!/bin/bash

# bring up can interface
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 10000
