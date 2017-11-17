#!/usr/bin/env python

import os
import sys
import time
import argparse
from time import sleep

import numpy as np
import scipy.stats

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

import lcm
from librav_lcm_msgs import *

class FieldVis(object):
    def __init__(self, lcm_h):
        self.lcm_h = lcm_h
    
    def publish_map(self):
        print "publish 2d map"

        # map2d = self.space.get_2d_map()
        # self.lcm_h.publish("envsim/map", map_msg.encode())

        print "map published"

    def plot_map(self):
        print "map size: {} * {}".format(self.space_size[0], self.space_size[1])

        fig = plt.figure()
        # random_image = np.random.random(
        #     [self.space_size[0], self.space_size[1]])
        map_img = self.space.get_2d_map().get_image()

        plt.imshow(map_img, cmap='gray',
                   interpolation='None', aspect='equal')

    def plot_space(self):
        # # create 3d space
        # N1 = self.space_size[0]
        # N2 = self.space_size[1]
        # N3 = self.space_size[2]

        # # collect voxel data
        # mat = np.zeros((self.space_size[0],self.space_size[1],self.space_size[2]))
        # mat[14,14,14] = 1
        # mat[0,0,0] = 1
        mat = self.space.get_occupancy_matrix()

        # plot space
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_xlim(0, self.space_size[0])
        ax.set_ylim(0, self.space_size[1])
        ax.set_zlim(0, self.space_size[2])
        # ax.set_aspect('equal')

        plotMatrix(ax, mat)

    def show_all_plots(self):
        plt.show()

    def scalar_field_msg_handler(self, channel, data):
        msg = ScalarField_t.decode(data)        
        print("Received message on channel \"%s\"" % channel)

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # Make data.
        X = np.arange(0, msg.size_x, 1)
        Y = np.arange(0, msg.size_y, 1)
        X, Y = np.meshgrid(X, Y)
        Z = np.zeros((msg.size_x, msg.size_y))
        print X.size
        print Y.size
        print Z.size
        for i in range(0, msg.size_x):
            for j in range(0,msg.size_y):
                Z[i, j] = msg.value[i][j]
                # print "map: {} , {} : {}".format(i,j,msg.value[i][j])

        # Plot the surface.
        surf = ax.plot_surface(X, Y, Z, cmap=cm.coolwarm,
                            linewidth=0, antialiased=False)

        # Customize the z axis.
        # ax.set_zlim(-1.01, 1.01)
        # ax.zaxis.set_major_locator(LinearLocator(10))
        # ax.zaxis.set_major_formatter(FormatStrFormatter('%.02f'))

        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.5, aspect=5)

        plt.show()
        

def main():
    print("started field_vis")

    # create a LCM instance
    lc = lcm.LCM()
    
    # create a environment generator
    vis = FieldVis(lc)

    subscription = lc.subscribe("ScalarField", vis.scalar_field_msg_handler)
    
    try:
        while True:
            lc.handle()
    except KeyboardInterrupt:
        pass

    lc.unsubscribe(subscription)

    ## for development 
    # for i in range(0,50):
    # gen.generate_rand_pos_space()
    # gen.publish_map()
    # sleep(2.0)

    # gen.plot_map()
    # gen.plot_space()
    # gen.show_all_plots()


# Standard boilerplate to call the main() function to begin
# the program.
if __name__ == '__main__':
    main()
