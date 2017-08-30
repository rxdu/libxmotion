#!/usr/bin/env python

import os
import sys
import copy

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


class ObjectBase(object):
    def __init__(self, x, y):
        self.position = np.array([x, y])


class Cuboid(ObjectBase):
    def __init__(self, x, y, sx, sy, sz):
        ObjectBase.__init__(self, x, y)
        self.size = np.array([sx, sy, sz])
        self.occupied_voxels = np.array([])
        # self.occupied_voxels.resize((sx * sy * sz))
        
        for xi in range(0, sx):
            for yi in range(0, sy):
                for zi in range(0, sz):
                    print "element: ({}, {}, {})".format(xi, yi, zi)
                    pos = np.array(
                        [[self.position[0] + xi, self.position[1] + yi, zi]])
                    if self.occupied_voxels.size == 0:
                        self.occupied_voxels = pos
                    else:
                        self.occupied_voxels = np.concatenate((self.occupied_voxels, pos), axis=0)
                    print self.occupied_voxels
        # np.reshape(self.occupied_voxels,(sx*sy*sz,3))
        print self.occupied_voxels

    def print_info(self):
        # for v in np.nditer(self.occupied_voxels, order='C'):
        for i in range(0, self.size[0]*self.size[1]*self.size[2])
            print self.occupied_voxels[i]
            # print "pos: ({}, {}, {}) \n".format(v[0], v[1], v[2])
