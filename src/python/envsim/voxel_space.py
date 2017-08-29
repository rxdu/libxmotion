#!/usr/bin/env python

import os
import sys
import copy

import numpy as np

from cell_map import *


class Voxel(object):
    def __init__(self, x, y, z):
        self.id = -1
        self.index = np.array([x, y, z])
        self.occupied = False

    def print_info(self):
        print "{}: {}, {}, {}".format("Voxel info", self.id, self.index, self.occupied)


class Space(object):
    def __init__(self, x, y, z):
        self.size = np.array([x, y, z])
        self.voxels = np.array([], dtype=object)
        self.voxels.resize((x, y, z))

        for zi in range(0, z):
            for yi in range(0, y):
                for xi in range(0, x):
                    id = zi * x * y + yi * x + xi
                    self.voxels[xi, yi, zi] = Voxel(xi, yi, zi)
                    self.voxels[xi, yi, zi].id = id

        self.map = Map(x, y)

    def add_obstacles(self):
        total_num = self.size[0] * self.size[1] * self.size[2]
        percentage = 0.5
        obs_prob = np.random.random([self.size[0], self.size[1], self.size[2]])
        print obs_prob
        for zi in range(0, self.size[2]):
            for yi in range(0, self.size[1]):
                for xi in range(0, self.size[0]):
                    if obs_prob[xi, yi, zi] > 0.5:
                        self.voxels[xi, yi, zi].occupied = True
                    else:
                        self.voxels[xi, yi, zi].occupied = False
        self.generate_2d_map()

    def generate_2d_map(self):
        for xi in range(0, self.size[0]):
            for yi in range(0, self.size[1]):
                for zi in range(0, self.size[2]):
                    if self.voxels[xi, yi, zi].occupied == True:
                        self.map.cells[xi, yi].occupied = True
                        break
                    else:
                        self.map.cells[xi, yi].occupied = False

    def get_2d_map(self):
        return self.map

    def print_info(self):
        print "{}: {}".format("Space info", self.size)

        for zi in range(0, self.size[2]):
            for yi in range(0, self.size[1]):
                for xi in range(0, self.size[0]):
                    self.voxels[xi, yi, zi].print_info()
