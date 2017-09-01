#!/usr/bin/env python

import os
import sys
import time
import argparse

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D

import lcm
from srcl_lcm_msgs import Graph_t
from space_vis import *
from voxel_space import *

class EnvGen(object):
    def __init__(self):
        self.space = Space(0,0,0)
        self.space_size = np.array([0, 0, 0])

    def set_env_size(self, x, y, z):
        self.space = Space(x,y,z)
        self.space_size = np.array([x, y, z])

    def generate_space(self):
        # add obstacles to space
        print 'generate space'
        
        # get projection as 2d map
        self.space.add_obstacles()

    def publish_space(self):
        print 'publish'
        # lc = lcm.LCM()

        # msg = example_t()
        # msg.timestamp = int(time.time() * 1000000)
        # msg.position = (1, 2, 3)
        # msg.orientation = (1, 0, 0, 0)
        # msg.ranges = range(15)
        # msg.num_ranges = len(msg.ranges)
        # msg.name = "example string - python"
        # msg.enabled = True

        # lc.publish("EXAMPLE", msg.encode())

    def show_map(self):
        print self.space_size[0]

        random_image = np.random.random(
            [self.space_size[0], self.space_size[1]])

        plt.imshow(random_image, cmap='gray',
                   interpolation='None', aspect='equal')
        plt.show()

    def show_space(self):
        # create 3d space
        N1 = self.space_size[0]
        N2 = self.space_size[1]
        N3 = self.space_size[2]
        
        # collect voxel data
        ma = np.zeros((self.space_size[0],self.space_size[1],self.space_size[2]))
        ma[14,14,14] = 1
        ma[0,0,0] = 1
        # ma = self.space.get_occupancy_matrix()

        # plot space
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_xlim(0, self.space_size[0])
        ax.set_ylim(0, self.space_size[1])
        ax.set_zlim(0, self.space_size[2])
        ax.set_aspect('equal')

        plotMatrix(ax, ma)

        plt.show()


def main():
    print("started env_gen")

    gen = EnvGen()
    gen.set_env_size(2, 5, 2)
    gen.show_space()


# Standard boilerplate to call the main() function to begin
# the program.
if __name__ == '__main__':
    main()
