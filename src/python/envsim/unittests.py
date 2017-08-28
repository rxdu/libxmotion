#!/usr/bin/env python

import os
import sys

import numpy as np
from voxel_space import *


def test_voxel():
    print "test voxel"
    v = Voxel(1, 2, 3)
    v.print_info()
    print "---------------------\n"


def test_space():
    print "test space"
    s = Space(2, 2, 2)
    s.print_info()
    print s.get_2d_projection()
    print "---------------------\n"


def main():
    print "started unit tests: \n"

    test_voxel()
    test_space()


# Standard boilerplate to call the main() function to begin
# the program.
if __name__ == '__main__':
    main()
