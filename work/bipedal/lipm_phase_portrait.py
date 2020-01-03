#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from scipy.integrate import odeint
import scipy.spatial as ssp

import sys
sys.path.append('../../scripts/')
from bipedal_robot.lipm import LinearInvertedPendulum

def point_in_hull(point, hull, tolerance=1e-6):
  return all(
    (np.dot(eq[:-1], point) + eq[-1] <= tolerance)
    for eq in hull.equations)

if __name__ == '__main__':
    x_ref = np.array([[0.5],[0.]])
    u_ref = np.array([[0.5]])
    x_start = np.array([[0.],[0.]])
    dt = 0.01
    height = 1.5
    plant = LinearInvertedPendulum(height)

    #mesh
    limit = 6
    x_max = limit
    x_min = -limit
    x_d_max = limit
    x_d_min = -limit
    X1, X2 = np.meshgrid(np.arange(x_min, x_max, 0.5), np.arange(x_d_min, x_d_max, 0.5))
    X = np.array([X1, X2])
    dX = plant.calc_dX(X)

    #normalize
    dX0_nr = dX[0] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
    dX1_nr = dX[1] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)

    #plot vetor
    plt.quiver(X[0], X[1], dX0_nr, dX1_nr, color='b')

    # xv = [1, -1,  1, -1]
    # yv = [1,  1, -1, -1]
    # bases = [xv, yv]
    # hull = ssp.ConvexHull(bases)

    #ode
    time = np.arange(0, 2, 0.01)
    step = 40 
    for xi in np.linspace(x_min, x_max, step):
        for xj in np.linspace(x_d_min, x_d_max, step):
            if xj == x_d_min or xj == x_d_max:
                x_data = odeint(plant.calc_dX, [xi, xj], time)
                plt.plot(x_data[:,0], x_data[:,1], color='k', linewidth = 0.6)
                # if x_data[-1,0] < 0.1 or x_data[-1,1] < 0.1:
                #     point_in_hull([xi, xj], hull)

    #counter
    # plt.contour(X[0], X[1], dX[1], levels=[0], colors="red")
    # plt.contour(X[0], X[1], dX[0], levels=[0], colors="Blue")

    plt.xlim([x_min, x_max])
    plt.ylim([x_d_min, x_d_max])
    plt.grid()
    plt.show()