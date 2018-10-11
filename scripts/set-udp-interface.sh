#!/bin/bash

sudo ifconfig $1 multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev $1

echo "set routing UDP traffic through interface: $1"
