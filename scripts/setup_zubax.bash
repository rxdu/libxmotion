#!/bin/bash

sudo slcand -o -s8 -t hw -S 3000000 /dev/zubax_can
sudo ip link set up slcan0