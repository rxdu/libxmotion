#!/usr/bin/env python

import os
import sys
import copy

import numpy as np

import matplotlib.pyplot as plt
import matplotlib.image as mpimg

class ObjectBase(object):
    def __init__(self, x, y):