"""
Lookup Table generation for model predictive trajectory generator

author: Atsushi Sakai
        Ruixiang Du
"""
from matplotlib import pyplot as plt
import numpy as np
import math
import trajectory_generator as planner
import motion_model
import pandas as pd


def calc_states_list():
    maxyaw = math.radians(-30.0)

    # x = np.arange(1.0, 30.0, 5.0)
    # y = np.arange(0.0, 20.0, 2.0)
    x = np.arange(1.0, 5.0, 1.0)
    y = np.arange(0.0, 5.0, 1.0)
    theta = np.arange(-maxyaw, maxyaw, maxyaw/15)
    kappa = 0.0

    states = []

    # for itheta in theta:
    #     for iy in y:
    #         for ix in x:
    #             states.append([ix, iy, itheta, kappa])

    for itheta in theta:
        states.append([1.0, 0, itheta, kappa])
    
    # states.append([1.0, 0.0, math.radians(10), 0])
    # states.append([1.0, 0.0, math.radians(-10), 0])

    # states.append([1.0, 0.0, math.radians(20), 0])
    # states.append([1.0, 0.0, math.radians(-20), 0])

    # states.append([1.0, 1.0, math.radians(0), 0])

    # states.append([1.0, 1.0, math.radians(10), 0])
    # states.append([1.0, 1.0, math.radians(-10), 0])

    return states


def search_nearest_one_from_lookuptable(tx, ty, ttheta, tkappa, lookuptable):
    mind = float("inf")
    minid = -1

    for (i, table) in enumerate(lookuptable):

        dx = tx - table[0]
        dy = ty - table[1]
        dtheta = ttheta - table[2]
        dkappa = tkappa - table[3]
        d = math.sqrt(dx ** 2 + dy ** 2 + dtheta ** 2 + dkappa ** 2)
        if d <= mind:
            minid = i
            mind = d

    # print(minid)

    return lookuptable[minid]


# def save_lookup_table(fname, table):
#     mt = np.array(table)
#     print(mt)
#     # save csv
#     df = pd.DataFrame()
#     df["x"] = mt[:, 0]
#     df["y"] = mt[:, 1]
#     df["yaw"] = mt[:, 2]
#     df["s"] = mt[:, 3]
#     df["km"] = mt[:, 4]
#     df["kf"] = mt[:, 5]
#     df.to_csv(fname, index=None)

#     print("lookup table file is saved as " + fname)


def generate_lookup_table():

    states = calc_states_list()

    # x, y, theta, kappa, a, b, c, d, sf
    lookuptable = [[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]

    for state in states:
        bestq = search_nearest_one_from_lookuptable(
            state[0], state[1], state[2], state[3], lookuptable)

        st0 = motion_model.State(0, 0, 0, 0)
        stf = motion_model.State(state[0], state[1], state[2], state[3])

        init_q = np.array([bestq[4], bestq[5], bestq[6], bestq[7], bestq[8]])

        q = planner.optimize_trajectory(st0, stf, init_q, 50, False)

        if q is not None:
            print("found good path")
            lookuptable.append(
                [state[0], state[1], state[2], state[3], q[0], q[1], q[2], q[3], q[4]])
   
    print "Table:"
    for table in lookuptable:
        print table

    print "\nsize of state list: {0}".format(len(states))
    print "finish lookup table generation, table size: {0}".format(len(lookuptable))

    # save_lookup_table("lookuptable.csv", lookuptable)

    # k0 = 0.0

    for table in lookuptable:
        planner.show_trajectory(table[4:8],table[8])

    plt.grid(True)
    plt.axis("equal")
    plt.show()

    # print("Done")


def main():
    generate_lookup_table()


if __name__ == '__main__':
    main()
