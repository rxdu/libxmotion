# import the necessary packages
import argparse
import cv2
import numpy as np
import os
import time
from dronekit import *
import droneapi
import socket
import sys

#arm the vehicle if armable
def checkArm(vehicle):
    #uncomment for real world test
##    while not vehicle.is_armable:
##        print "waiting for vehicle to become armable"
##        time.sleep(1)
    print "Contact!"
    vehicle.armed = True
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)
    print " Armed"
    return

#wait until the vehicle is in guided mode
def wait4guided(vehicle):
    while (vehicle.mode.name != 'GUIDED'):
        print " Waiting for GUIDED mode..."
        time.sleep(2)
    print "In GUIDED mode"
    return

def main():
    # Import DroneKit-Python
    from dronekit import connect, VehicleMode

    # Connect to the Vehicle.
    #print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect('/dev/ttyUSB0', baud = 57600, rate=6)

    # Get some vehicle attributes (state)
    print "\nGet some vehicle attribute values:"
    print " Autopilot Firmware version: %s" % vehicle.version
    print "   Major version number: %s" % vehicle.version.major
    print "   Minor version number: %s" % vehicle.version.minor
    print "   Patch version number: %s" % vehicle.version.patch
    print "   Release type: %s" % vehicle.version.release_type()
    print "   Release version: %s" % vehicle.version.release_version()
    print "   Stable release?: %s" % vehicle.version.is_stable()

    print " Attitude: %s" % vehicle.attitude
    print " Velocity: %s" % vehicle.velocity
    print " GPS: %s" % vehicle.gps_0
    print " Gimbal status: %s" % vehicle.gimbal
    print " Battery: %s" % vehicle.battery
    print " EKF OK?: %s" % vehicle.ekf_ok
    print " Last Heartbeat: %s" % vehicle.last_heartbeat
    print " Rangefinder: %s" % vehicle.rangefinder
    print " Rangefinder distance: %s" % vehicle.rangefinder.distance
    print " Rangefinder voltage: %s" % vehicle.rangefinder.voltage
    print " Heading: %s" % vehicle.heading
    print " Is Armable?: %s" % vehicle.is_armable
    #print " System status: %s" % vehicle.system_status.state
    print " Groundspeed: %s" % vehicle.groundspeed    # settable
    print " Airspeed: %s" % vehicle.airspeed    # settable
    print " Mode: %s" % vehicle.mode.name    # settable
    print " Armed: %s" % vehicle.armed    # settable

    print " Armed: (check before task): %s" % vehicle.armed

    checkArm(vehicle)
    wait4guided(vehicle)

    time.sleep(2)

    vehicle.armed = False
    while vehicle.armed:
        print " Waiting for disarming..."
        time.sleep(1)

    print " Armed: (check before close communication): %s" % vehicle.armed    # settable

    # Close vehicle object before exiting script
    vehicle.close()

    # Shut down simulator
    print("Completed")

    return

if __name__ == "__main__":
    main()
