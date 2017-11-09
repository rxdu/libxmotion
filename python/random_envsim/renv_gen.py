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

import lcm
from librav_lcm_msgs import *
from space_vis import *
from voxel_space import *
from object_lib import *


class RandEnvGen(object):
    def __init__(self, lcm_h):
        self.lcm_h = lcm_h
        self.space = Space(0, 0, 0)
        self.space_size = np.array([0, 0, 0])

        # individual object constraints
        self.min_side = 3
        self.max_side = 8
        self.norm_side = 5

        self.min_height = 1
        self.max_height = 4
        self.norm_height = 3

        # space obstacle configurations
        self.obj_num = 500
        self.obs_perc = 0.35

    def set_env_size(self, x, y, z):
        self.space_size = np.array([x, y, z])

    def generate_rand_pos_space(self):
        # create empty space
        print 'generate random position space'
        self.space = Space(
            self.space_size[0], self.space_size[1], self.space_size[2])

        # add obstacles to space
        # self.space.add_obstacles()
        objs = self.randomly_pick_objects()
        self.space.add_objects(objs, self.obs_perc)

        # get projection as 2d map
        self.space.get_2d_map().get_occupied_percentage()

    def generate_poisson_space(self):
        # create empty space
        print 'generate spatial poisson space'
        self.space = Space(
            self.space_size[0], self.space_size[1], self.space_size[2])

        # add obstacles to space
        # self.space.add_obstacles()
        objs = self.gen_poisson_pos_objects()
        self.space.add_objects(objs, self.obs_perc)

        # get projection as 2d map
        self.space.get_2d_map().get_occupied_percentage()

    def randomly_pick_objects(self):
        # random locations
        loc_x = np.random.randint(
            0, self.space_size[0], size=(1, self.obj_num))
        loc_y = np.random.randint(
            0, self.space_size[1], size=(1, self.obj_num))

        rand_loc = np.array([])
        rand_loc.resize((self.obj_num, 2))

        for i in range(0, self.obj_num):
            rand_loc[i, 0] = loc_x[0, i]
            rand_loc[i, 1] = loc_y[0, i]

        # add random objects to random locations
        objs = []
        for loc in rand_loc:
            # random object size
            sz_xy = np.random.randint(
                self.min_side, self.max_side, size=(1, 2))
            sz_z = np.random.randint(
                self.min_height, self.max_height, size=(1, 1))
            sz_xyz = np.array([sz_xy[0, 0], sz_xy[0, 1], sz_z])
            obj = Cuboid(loc[0], loc[1], sz_xyz[0], sz_xyz[1], sz_xyz[2])
            objs.append(obj)
            # print loc
            # print sz_xyz

        # obj1 = Cuboid(5,5,2,5,3)
        # objs.append(obj1)

        # obj2 = Cuboid(5,5,5,2,3)
        # objs.append(obj2)

        return objs

    def PoissonPP(self, rate, Dx, Dy=None ):
        '''
        Determines the number of events `N` for a rectangular region,
        given the rate `rate` and the dimensions, `Dx`, `Dy`.
        Returns a <2xN> NumPy array.
        Source: http://connor-johnson.com/2014/02/25/spatial-point-processes/
        '''
        if Dy == None:
            Dy = Dx
        N = scipy.stats.poisson( rate*Dx*Dy ).rvs()
        x = scipy.stats.uniform.rvs(0,Dx,((N,1)))
        y = scipy.stats.uniform.rvs(0,Dy,((N,1)))
        P = np.hstack((x,y))
        return P

    def gen_poisson_pos_objects(self):
        # random poission locations
        rate = self.obj_num/(self.space_size[0] * self.space_size[1])
        loc = self.PoissonPP(rate, self.space_size[0], self.space_size[1])
     
        rand_loc = np.array([])
        rand_loc.resize((loc.size/2, 2))
        for i in range(0, loc.size/2):
            rand_loc[i, 0] = int(loc[i, 0])
            rand_loc[i, 1] = int(loc[i, 1])

        # add random objects to random locations
        objs = []
        ymu,ysigma = 2, 0.8
        zmu,zsigma = self.norm_height, 0.8
        for loc in rand_loc:
            # random object size
            sz_x = 8
            sz_y = 3          
            # sz_y = int(np.random.normal(ymu, ysigma))
            sz_z = int(np.random.normal(zmu, zsigma))
            obj = PoissonCuboid(loc[0], loc[1], sz_x, sz_y, sz_z)
            objs.append(obj)
            # print loc
            # print sz_xyz

        # obj1 = Cuboid(5,5,2,5,3)
        # objs.append(obj1)

        # obj2 = Cuboid(5,5,5,2,3)
        # objs.append(obj2)

        return objs

    def publish_map(self):
        print "publish 2d map"

        map2d = self.space.get_2d_map()

        map_msg = Map_t()

        map_msg.size_x = self.space_size[0]
        map_msg.size_y = self.space_size[1]
        map_msg.size_z = self.space_size[2]
        
        map_msg.cell_num = map2d.cells.size
        map_msg.voxel_num = self.space.voxels.size

        for yi in range(0, map2d.size[1]):
            for xi in range(0, map2d.size[0]):
                cell_msg = Cell_t()
                cell_msg.id = map2d.cells[xi, yi].id
                cell_msg.pos_x = map2d.cells[xi, yi].position[0]
                cell_msg.pos_y = map2d.cells[xi, yi].position[1]
                cell_msg.occupied = map2d.cells[xi, yi].occupied
                map_msg.cells.append(cell_msg)

        for zi in range(0, self.space_size[2]):
            for yi in range(0, self.space_size[1]):
                for xi in range(0, self.space_size[0]):
                    voxel_msg = Voxel_t()
                    voxel_msg.id = self.space.voxels[xi, yi, zi].id
                    voxel_msg.pos_x = self.space.voxels[xi, yi, zi].position[0]
                    voxel_msg.pos_y = self.space.voxels[xi, yi, zi].position[1]
                    voxel_msg.pos_z = self.space.voxels[xi, yi, zi].position[2]
                    voxel_msg.occupied = self.space.voxels[xi, yi, zi].occupied
                    map_msg.voxels.append(voxel_msg)

        self.lcm_h.publish("envsim/map", map_msg.encode())

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

    def map_request_handler(self, channel, data):
        msg = MapRequest_t.decode(data)        
        print("Received message on channel \"%s\"" % channel)
        
        self.set_env_size(msg.map_size_x, msg.map_size_y, msg.map_size_z)
        
        if msg.map_type == 2:
            self.generate_poisson_space()
        else:
            self.generate_rand_pos_space()
        self.publish_map()

def main():
    print("started renv_gen")

    # create a LCM instance
    lc = lcm.LCM()
    
    # create a environment generator
    gen = RandEnvGen(lc)
    # set random environment size: 1 unit = 1 meter
    # gen.set_env_size(30, 50, 5)
    # gen.set_env_size(5, 5, 5)

    gen.gen_poisson_pos_objects()

    subscription = lc.subscribe("envsim/map_request", gen.map_request_handler)
    
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
