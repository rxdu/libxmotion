import math
import numpy as np
import scipy.integrate as integrate

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import motion_model

# System model
# System state: x = {x, y, theta, kappa}
# Control input: u = {u, v}
# - x_dot = v(t) * cos(theta(t))
# - y_dot = v(t) * sin(theta(t))
# - theta_dot = kappa(t)
# - kappa_dot = u(t)
# Given x_0, x_f, find u(t) and v(t).
#
# To solve the BVP, transcribe the above problem to be
# - kappa(s) = a + bs + cs^2 + ds^3
# - theta(s) = as + bs^2/2 + cs^3/3 + ds^4/4
# - x(s) = \int_{0}^{s} cos(theta(s))
# - y(s) = \int_{0}^{s} sin(theta(s))
# Given x_0, x_f, find kappa(s), s_f
#
# For the constraint satisfaction case, we have 5 BCs:
# (1) initial condition
#  s_0 = 0: x(0) = y(0) = theta(0) = 0, kappa(0) = kappa_0
# => k(0) = a   ... [1]
# (2) final condition
#  x_f = {x_f, y_f, theta_f, kappa_f}
# => g(q) = h(q) - x_f = 0  ... [2]
# In total, 5 equations [1][2] and 5 unkowns {a,b,c,d,s_f}.
# [1] is trivial to solve and we can easily get a = kappa(0)
# [2] is a 4-D nonlinear system of equations
#
# Use Euler's method to solve:
# [\partial/\partial q g(q)]\delta q = - g(q)
# where q = [a, b, c, d, s_f]^T
# => q_{n+1} = q_{n} + \delta q
# => q_{n+1} = q_{n} - [\partial/\partial q g(q)]^{-1} g(q)
#
# [\partial/\partial q g(q)] = [ ... ] from Appendix

# Abbreviations:
# s: arc length
# st: state
# 0,f: initial, final

# optimization parameters
terminal_residual = 0.01
residual_weight = np.array([1.0, 1.0, 1.0, 0.2])
index_set = []
res_set = []


def calc_g(q, stf):
    print "Evaluating state with q = {0}".format(q)
    st = get_state_at(q)
    st.print_info()

    d = np.array([st.x - stf.x,
                  st.y - stf.y,
                  st.theta - stf.theta,
                  st.kappa - stf.kappa])

    return d


def calc_residual(gq):
    res = np.multiply(gq, residual_weight)
    print "residual: {0}".format(res)
    # weighted_res = math.sqrt(res[0]**2 + res[1]**2 + res[2]**2 + res[3]**2)
    weighted_res = np.linalg.norm(res)

    return weighted_res


def evaluate_partial_derivatives(q):

    # extract variables for convenience
    a = q[0]
    b = q[1]
    c = q[2]
    d = q[3]
    sf = q[4]

    # define integrands
    def C2(s): return (s**2) * np.cos(a * s + b * (s**2) /
                                      2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    def C3(s): return (s**3) * np.cos(a * s + b * (s**2) /
                                      2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    def C4(s): return (s**4) * np.cos(a * s + b * (s**2) /
                                      2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    def S2(s): return (s**2) * np.sin(a * s + b * (s**2) /
                                      2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    def S3(s): return (s**3) * np.sin(a * s + b * (s**2) /
                                      2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    def S4(s): return (s**4) * np.sin(a * s + b * (s**2) /
                                      2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    # integrate using Simpson's rule
    pts = np.linspace(0.0, sf, 5)
    x0_samples = [S2(pt) for pt in pts]
    x1_samples = [S3(pt) for pt in pts]
    x2_samples = [S4(pt) for pt in pts]
    y0_samples = [C2(pt) for pt in pts]
    y1_samples = [C3(pt) for pt in pts]
    y2_samples = [C4(pt) for pt in pts]

    x_p0 = integrate.simps(x0_samples, pts)
    x_p1 = integrate.simps(x1_samples, pts)
    x_p2 = integrate.simps(x2_samples, pts)

    y_p0 = integrate.simps(y0_samples, pts)
    y_p1 = integrate.simps(y1_samples, pts)
    y_p2 = integrate.simps(y2_samples, pts)

    return np.array([[-x_p0/2.0, -x_p1/3.0, -x_p2/4.0],
                     [-y_p0/2.0, -y_p1/3.0, -y_p2/4.0]])

    # x_p0 = integrate.quad(S2, 0, sf)
    # x_p1 = integrate.quad(S3, 0, sf)
    # x_p2 = integrate.quad(S4, 0, sf)

    # y_p0 = integrate.quad(C2, 0, sf)
    # y_p1 = integrate.quad(C3, 0, sf)
    # y_p2 = integrate.quad(C4, 0, sf)

    # return np.array([[-x_p0[0]/2.0, -x_p1[0]/3.0, -x_p2[0]/4.0],
    #                  [-y_p0[0]/2.0, -y_p1[0]/3.0, -y_p2[0]/4.0]])


def calc_jacobian(q):
    J = np.zeros((4, 4))

    sf = q[4]
    stq = get_state_at(q)
    theta_f = stq.theta
    kappa_f = stq.kappa
    kappa_f_prime = q[1] + 2 * q[2] * sf + 3 * q[3] * (sf**2)

    # x, y
    xy_partial = evaluate_partial_derivatives(q)

    J[0, :3] = xy_partial[0, :]
    J[0, 3] = np.cos(theta_f)

    J[1, :3] = xy_partial[1, :]
    J[1, 3] = np.sin(theta_f)

    # theta
    J[2, :] = [(sf**2)/2.0, (sf**3)/3.0, (sf**4)/4.0, kappa_f]

    # kappa
    J[3, :] = [sf, sf**2, sf**3, kappa_f_prime]

    print "J: {0}".format(J)

    return J


def calculate_initial_guess(st0, stf):

    d = np.sqrt(stf.x**2 + stf.y**2)
    delta_theta = np.abs(stf.theta)
    s = d * (delta_theta**2/5.0 + 1) + delta_theta*2.0/5.0
    c = 0
    a = 6 * stf.theta/(s**2) - 2*st0.kappa /s + 4 * stf.kappa/s
    b = 3 * (st0.kappa + stf.kappa)/(s**2) + 6 * stf.theta / (s**3)

    return np.array([a, b, c, d, s])


def optimize_trajectory(st0, stf, q0, max_iter=5):
    print "Initial state: "
    st0.print_info()
    print "Final state: "
    stf.print_info()
    print "-----------------"

    # initial condition
    q = q0
    q[0] = st0.kappa

    show_trajectory_frame(q, stf)

    # final condition
    for i in range(max_iter):
        print "\n********************* i = {0}\n".format(i)

        gq = calc_g(q, stf)
        res = calc_residual(gq)

        # save for numerical analysis
        index_set.append(i)
        res_set.append(res)

        print "\ng(q): {0}".format(gq)
        print "r: {0}\n".format(res)

        if res <= terminal_residual:
            print("path found with residual error:" + str(res))
            break

        # calculate Jacobian
        J = calc_jacobian(q)
        try:
            delta_q = - np.linalg.inv(J) * np.matrix(gq).T
        except np.linalg.linalg.LinAlgError:
            print("failed to invert Jacobian matrix: LinAlgError")
            da, db, dc, dd, dsf = None, None, None, None, None
            break
        print "\nq(n): {0}".format(q)
        print "delta_q: {0}".format(np.array(delta_q)[:, 0])
        q[1:5] += np.array(delta_q)[:, 0]
        print "q(n+1): {0}".format(q)
        show_trajectory_frame(q, stf)

    print "------"
    print "q = {0}".format(q)

    return q


def get_state_at(q):
    return evaluate_state(q[:4], q[4])


def evaluate_state(poly_coeffs, sf):

    a = poly_coeffs[0]
    b = poly_coeffs[1]
    c = poly_coeffs[2]
    d = poly_coeffs[3]

    kappa = a + b * sf + c * (sf**2) + d * (sf**3)
    theta = a * sf + b * (sf**2) / 2.0 + c * (sf**3) / 3.0 + d * (sf**4) / 4.0

    def fx(s): return np.cos(a * s + b * (s**2) /
                             2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    def fy(s): return np.sin(a * s + b * (s**2) /
                             2.0 + c * (s**3) / 3.0 + d * (s**4) / 4.0)

    # xint = integrate.quad(fx, 0, sf)
    # yint = integrate.quad(fy, 0, sf)

    if sf != 0.0:
        pts = np.linspace(0.0, sf, 5)
        # pts = np.arange(0, sf)
        # print "sf: {0}, sampling points: {1}".format(sf, pts)
        x_samples = [fx(pt) for pt in pts]
        y_samples = [fy(pt) for pt in pts]

        x = integrate.simps(x_samples, pts)
        y = integrate.simps(y_samples, pts)
    else:
        x = 0
        y = 0

    return motion_model.State(x, y, theta, kappa)


def setup_plot():
    # plt.axis("equal")
    plt.axes().set_aspect('equal', 'datalim')
    plt.grid(True)


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y)
    plt.plot(0, 0)


def show_trajectory(poly_coeffs, s_f, show_3d=False):

    ds = 0.01
    svals = np.arange(0.0, s_f, ds)
    states = [evaluate_state(poly_coeffs, s) for s in svals]

    x, y, yaw = [], [], []
    for s in states:
        x.append(s.x)
        y.append(s.y)
        yaw.append(s.theta)

    plt.plot(x, y, "-r")

    # plt.figure()
    # plt.plot(svals, yaw)

    # plt.figure()
    # plt.plot(x, svals, "-g")

    # plt.figure()
    # plt.plot(y, svals, "-b")

    if show_3d == True:
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.plot(x, y, svals)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")


def show_trajectory_frame(q, target, show_3d=False):

    ds = 0.01
    svals = np.arange(0.0, q[4], ds)
    states = [evaluate_state(q[:4], s) for s in svals]

    x, y, yaw = [], [], []
    for s in states:
        x.append(s.x)
        y.append(s.y)
        yaw.append(s.theta)

    plt.clf()
    plot_arrow(target.x, target.y, target.theta)

    plt.plot(x, y, "-r")

    # plt.figure()
    # plt.plot(svals, yaw)

    # plt.figure()
    # plt.plot(x, svals, "-g")

    # plt.figure()
    # plt.plot(y, svals, "-b")

    plt.pause(2.0)


def test_trajectory_plot():
    # plot_arrow(10,10,math.radians(-90))

    # poly_coeffs = np.array([0.0, 33, -82, 41.5])
    poly_coeffs = np.array([0.0, 1, 1, -0.8])
    plot_arrow(1.5, 1.5, math.radians(30))

    show_trajectory(poly_coeffs, 2.5)
    plt.show()


def test_trajectory_optimization(show_traj=False):
    # q0 = np.array([0.0, 1, 1, -0.8, 2.5])
    # st0 = motion_model.State(0, 0, 0, 0)
    # stf = motion_model.State(1.5, 1.2, math.radians(30), 0)

    # q0 = np.array([0.0, -2.0, 4.0, -1.5, 2.5])
    st0 = motion_model.State(0, 0, 0, 0)
    stf = motion_model.State(1.0, 0.0, math.radians(0), 0)
    q0 = calculate_initial_guess(st0,stf)
    print "initial guess: {0}".format(q0)

    q = optimize_trajectory(st0, stf, q0)

    plt.plot(index_set, res_set)
    plt.title('residual')
    plt.show()

    if show_traj == True:
        plot_arrow(stf.x, stf.y, stf.theta)
        show_trajectory(q[:4], q[4])
        plt.show()


def main():
    print("trajectory generator started: " + __file__)

    setup_plot()
    # test_trajectory_plot()
    test_trajectory_optimization()


if __name__ == '__main__':
    main()
