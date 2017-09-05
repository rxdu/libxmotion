#!/usr/bin/env python

import os
import sys
import copy

import numpy as np

from cell_map import *


class Voxel(object):
    def __init__(self, x, y, z):
        self.id = -1
        self.position = np.array([x, y, z])
        self.occupied = False

    def print_info(self):
        print "{}: {}, {}, {}".format("Voxel info", self.id, self.position, self.occupied)


class Space(object):
    def __init__(self, x, y, z):
        self.size = np.array([x, y, z])
        self.voxels = np.array([], dtype=object)
        self.voxels.resize((x, y, z))

        for zi in range(0, z):
            for yi in range(0, y):
                for xi in range(0, x):
                    vid = zi * x * y + yi * x + xi
                    self.voxels[xi, yi, zi] = Voxel(xi, yi, zi)
                    self.voxels[xi, yi, zi].id = vid

        self.map2d = Map(x, y)

    def __generate_2d_map(self):
        """Generate 2D map of the space
        This function should be called whenever the space is changed
        """
        for xi in range(0, self.size[0]):
            for yi in range(0, self.size[1]):
                for zi in range(0, self.size[2]):
                    if self.voxels[xi, yi, zi].occupied == True:
                        self.map2d.cells[xi, yi].occupied = True
                        break
                    else:
                        self.map2d.cells[xi, yi].occupied = False

    def add_obstacles(self):
        total_num = self.size[0] * self.size[1] * self.size[2]
        percentage = 0.995
        obs_prob = np.random.random([self.size[0], self.size[1], self.size[2]])
        # print obs_prob
        for zi in range(0, self.size[2]):
            for yi in range(0, self.size[1]):
                for xi in range(0, self.size[0]):
                    if obs_prob[xi, yi, zi] > percentage:
                        self.voxels[xi, yi, zi].occupied = True
                    else:
                        self.voxels[xi, yi, zi].occupied = False

        # regenerate 2d map
        self.__generate_2d_map()

    def add_objects(self, objs, max_percentage):
        for obj in objs:
            for vi in range(0, obj.size[0] * obj.size[1] * obj.size[2]):
                idx = obj.occupied_voxels[vi]
                if idx[0] >= 0 and idx[0] < self.size[0] and idx[1] >= 0 and idx[1] < self.size[1] and idx[2] >= 0 and idx[2] < self.size[2]:
                    self.voxels[idx[0], idx[1], idx[2]].occupied = True
            # regenerate 2d map
            self.__generate_2d_map()
            if self.map2d.get_occupied_percentage() > max_percentage:
                break        
        print "occupied percentage: {}".format(self.map2d.get_occupied_percentage())

    def get_2d_map(self):
        return self.map2d

    def get_occupancy_matrix(self):
        mat = np.zeros((self.size[0], self.size[1], self.size[2]))
        for xi in range(0, self.size[0]):
            for yi in range(0, self.size[1]):
                for zi in range(0, self.size[2]):
                    if self.voxels[xi, yi, zi].occupied == True:
                        mat[xi, yi, zi] = 1
        return mat

    def print_info(self):
        print "{}: {}".format("Space info", self.size)

        for zi in range(0, self.size[2]):
            for yi in range(0, self.size[1]):
                for xi in range(0, self.size[0]):
                    self.voxels[xi, yi, zi].print_info()
