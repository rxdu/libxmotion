#!/usr/bin/env python

import os
import sys
import time
import argparse
from time import sleep

import numpy as np

import lcm
from librav_lcm_msgs import *

class CarCommander(object):
    def __init__(self, lcm_h):
        self.lcm_h = lcm_h

    def publish_car_command(self):
        print "publish car command"
        cmd_msg = CarCommand_t()
        cmd_msg.servo = 0.1
        cmd_msg.motor = 0.2

        self.lcm_h.publish("car_command", cmd_msg.encode())

        print "command published"

    # def map_request_handler(self, channel, data):
        # msg = MapRequest_t.decode(data)        
        # print("Received message on channel \"%s\"" % channel)
        
        # self.set_env_size(msg.map_size_x, msg.map_size_y, msg.map_size_z)
        
        # self.generate_space()
        # self.publish_map()

def main():
    print("started env_gen")

    # create a LCM instance
    lc = lcm.LCM()    
    car_cmd = CarCommander(lc)
    car_cmd.publish_car_command()

    # subscription = lc.subscribe("envsim/map_request", gen.map_request_handler)
    
    # try:
    #     while True:
    #         lc.handle()
    # except KeyboardInterrupt:
    #     pass

    # lc.unsubscribe(subscription)


# Standard boilerplate to call the main() function to begin
# the program.
if __name__ == '__main__':
    main()
