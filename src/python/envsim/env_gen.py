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


class EnvGen(object):
    def __init__(self):
        self.space = np.array([])
        self.space_size = np.array([0, 0, 0])

    def set_env_size(self, x, y, z):
        self.space.resize((x, y, z))
        self.space_size = np.array([x, y, z])

        for x in range(0, self.space_size[0]):
            for y in range(0, self.space_size[1]):
                for z in range(0, self.space_size[2]):
                    self.space[x, y, z] = 0

        print self.space

    def generate_space(self):
        # add obstacles to space
        print self.space
        # get projection as 2d map

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
        # prepare some coordinates
        x, y, z = np.indices((8, 8, 8))

        # draw cuboids in the top left and bottom right corners, and a link between them
        cube1 = (x < 3) & (y < 3) & (z < 3)
        cube2 = (x >= 5) & (y >= 5) & (z >= 5)
        link = abs(x - y) + abs(y - z) + abs(z - x) <= 2

        # combine the objects into a single boolean array
        voxels = cube1 | cube2 | link

        # set the colors of each object
        colors = np.empty(voxels.shape, dtype=object)
        colors[link] = 'red'
        colors[cube1] = 'blue'
        colors[cube2] = 'green'

        # and plot everything
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.voxels(voxels, facecolors=colors, edgecolor='k')

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
