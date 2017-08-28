#!/usr/bin/env python

import os
import sys
import copy

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


class Voxel(object):
    def __init__(self, x, y, z):
        self.id = -1
        self.index = np.array([x, y, z])
        self.occupied = False

    def print_info(self):
        print "{}: {}, {}, {}".format("Voxel info", self.id, self.index, self.occupied)

class Map(object):
    def __init__(self, x, y):
        self.size = np.array([x, y])
        self.voxels = np.array([], dtype=object)
        self.map.resize((x, y))


class Space(object):
    def __init__(self, x, y, z):
        self.size = np.array([x, y, z])
        self.voxels = np.array([], dtype=object)
        self.voxels.resize((x, y, z))

        self.map = np.array([], dtype=object)
        self.map.resize((x, y))

        for zi in range(0, z):
            for yi in range(0, y):
                for xi in range(0, x):
                    id = zi * x * y + yi * x + xi
                    self.voxels[xi, yi, zi] = Voxel(xi, yi, zi)
                    self.voxels[xi, yi, zi].id = id

    def get_2d_projection(self):
        for xi in range(0, self.size[0]):
            for yi in range(0, self.size[1]):
                self.map[xi, yi] = copy.deepcopy(self.voxels[xi, yi, 0])
                for zi in range(0, self.size[2]):
                    if self.voxels[xi, yi, zi].occupied == True:
                        self.map[xi, yi].occupied = True
                        break
                    else:
                        self.map[xi, yi].occupied = False
        return self.map

    def print_info(self):
        print "{}: {}".format("Space info", self.size)

        for zi in range(0, self.size[2]):
            for yi in range(0, self.size[1]):
                for xi in range(0, self.size[0]):
                    self.voxels[xi, yi, zi].print_info()
