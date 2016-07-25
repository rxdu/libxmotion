from mpl_toolkits.mplot3d import Axes3D
import numpy
import matplotlib.pyplot as plt

data = numpy.loadtxt('/home/rdu/Workspace/srcl_robot_suite/srcl_ctrl/build/bin/output.txt')
fig = plt.figure()
#ax = fig.gca(projection='3d')
plt.plot(data[:,0],data[:,0],'.-')
plt.show()
