#!/bin/bash

sudo slcand -o -s8 -t hw -S 3000000 /dev/canable
sudo ip link set up slcan0