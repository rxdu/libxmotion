#!/usr/bin/env python

import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import scipy.stats

def PoissonPP( rt, Dx, Dy=None ):
    '''
    Determines the number of events `N` for a rectangular region,
    given the rate `rt` and the dimensions, `Dx`, `Dy`.
    Returns a <2xN> NumPy array.
    '''
    if Dy == None:
        Dy = Dx
    N = scipy.stats.poisson( rt*Dx*Dy ).rvs()
    x = scipy.stats.uniform.rvs(0,Dx,((N,1)))
    y = scipy.stats.uniform.rvs(0,Dy,((N,1)))
    P = np.hstack((x,y))
    return P

rate, Dx = 0.2, 20
P = PoissonPP( rate, Dx ).T
fig, ax = plt.subplots()
ax = fig.add_subplot(111)
ax.scatter( P[0], P[1], edgecolor='b', facecolor='none', alpha=0.5 )
# lengths of the axes are functions of `Dx`
plt.xlim(0,Dx) ; plt.ylim(0,Dx)
# label the axes and force a 1:1 aspect ratio
plt.xlabel('X') ; plt.ylabel('Y') ; ax.set_aspect(1)
plt.title('Poisson Process')
plt.savefig( 'poisson_lambda_0p2.png', fmt='png', dpi=100 )