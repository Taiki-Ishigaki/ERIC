#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from scipy.integrate import odeint
import scipy.spatial as ssp

class LinearInvertedPendulum(object):
    gravity = 9.8 # gravity

    def __init__(self, height  = 1.5, weight = 50):
        self.height = height
        self.omega2 = self.gravity / self.height
        self.omega = np.sqrt(self.omega2)

        self.A = np.array([[0, 1],[self.omega2, 0]]) # state matrix
        self.B = np.array([[0],[-self.omega2]]) # input matrix

    
    def init_state(self, x, x_ref, u_ref, dt = 0.01, dxot = 0., dxdot = 0):
        self.X = np.array([[x], [dxot]])
        self.dxot = np.array([[dxot], [dxdot]])
        self.u = self.bestCOG_Regulator(x_ref, u_ref)
        self.dT = dt

    def do_action(self, x_ref, u_ref):
        self.u = self.bestCOG_Regulator(x_ref, u_ref)
        self.update_state()

    def update_state(self):
        self.dxot = self.A @ self.X + self.B @ self.u
        self.X =self.X + self.dxot * self.dT 

    def bestCOG_Regulator(self, x_ref, u_ref):
        self.alpha = 3.0
        self.F = self.alpha * np.array([[1.0, 1.0/self.omega2]])
        return u_ref - self.F @ (x_ref - self.X)

    def get_X(self):
        return self.X
    
    def get_u(self):
        return self.u

    def calc_dX(self, X, t = 0):
        if X.shape == np.zeros((self.A[0].size, 1)).shape:# normal
            return self.A @ X
        elif  X.ndim == 1: # for odeint
            return self.A @ X.T
        else: # for meshgrid
            dxot = np.zeros((X[:,0,0].size,X[0,0,:].size, X[0,:,0].size), dtype=np.float64)
            for i in range(X[:,0,0].size):
                x_tmp =  np.empty((0,X[0,0,:].size, X[0,:,0].size), dtype=np.float64)
                for j in range(X[:,0,0].size):
                    x_tmp = np.append(x_tmp, np.array([self.A[j,i] * X[i]]), axis = 0)    
                dxot += x_tmp
            return dxot

def point_in_hull(point, hull, tolerance=1e-6):
  return all(
    (np.dot(eq[:-1], point) + eq[-1] <= tolerance)
    for eq in hull.equations)

if __name__ == '__main__':
    x_ref = np.array([[0.5],[0.]])
    u_ref = np.array([[0.5]])
    x_start = np.array([[0.],[0.]])
    dt = 0.01
    height = 9.8
    plant = LinearInvertedPendulum(height)
    fig = plt.figure()
    ax = fig.add_subplot(111)

    #mesh
    limit = 6
    x_max = limit
    x_min = -limit
    dx_max = limit
    dx_min = -limit
    X1, X2 = np.meshgrid(np.arange(x_min, x_max+1, 0.5), np.arange(dx_min, dx_max+1, 0.5))
    X = np.array([X1, X2])
    dX = plant.calc_dX(X)

    #normalize
    dX0_nr = dX[0] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
    dX1_nr = dX[1] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)

    #plot vetor
    # plt.quiver(X[0], X[1], dX0_nr, dX1_nr, color='b')

    # xv = [1, -1,  1, -1]
    # yv = [1,  1, -1, -1]
    # bases = [xv, yv]
    # hull = ssp.ConvexHull(bases)

    #ode
    time = np.arange(0, 16, 0.01)
    step = 20 
    count = 0

    for xi in np.linspace(x_min, x_max, step):
        for xj in np.linspace(dx_min, dx_max, step):
            if (xj == dx_min and xi > 0)  or (xj == dx_max and xi < 0) or\
               (xi ==  x_min and xj > 0)  or (xi ==  x_max and xj < 0):
                count += 1
                x_data = odeint(plant.calc_dX, [xi, xj], time)
                plt.plot(x_data[:,0], x_data[:,1], color='k', linewidth = 0.6)
                # if x_data[-1,0] < 0.1 or x_data[-1,1] < 0.1:
                #     point_in_hull([xi, xj], hull)
                if (count%2 == 0 and xi > 0) or (count%2 == 1 and xi < 0):
                    ss = []
                    for s in range(time.size-1):
                        if (x_data[s,1]-dx_max)*(x_data[s+1,1]-dx_max) <= 0 or (x_data[s,1]-dx_min)*(x_data[s+1,1]-dx_min) <= 0 or \
                           (x_data[s,0]- x_max)*(x_data[s+1,0]- x_max) <= 0 or (x_data[s,0]- x_min)*(x_data[s+1,0]- x_min) <= 0:
                            ss.append(s)
                    l = ss[1] - ss[0]
                    k = 0#int(l/15)
                    if  (abs(x_data[ss[0],1] + x_data[ss[1],1])/2 > 1 and abs(x_data[ss[0],0] - x_data[ss[1],0]) > 4) or\
                        (abs(x_data[ss[0],0] + x_data[ss[1],0])/2 > 1 and abs(x_data[ss[0],1] - x_data[ss[1],1]) > 4):
                        X = np.array([x_data[ss[0]+k,0], x_data[ss[0]+k,1]])
                        dX = plant.calc_dX(X)
                        dX0_nr = dX[0] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        dX1_nr = dX[1] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        plt.quiver(X[0]-dX0_nr/3+dX0_nr/2, X[1]-dX1_nr/3+dX1_nr/2, dX0_nr, dX1_nr, color='b')
                        X = np.array([x_data[ss[1]-k,0], x_data[ss[1]-k,1]])
                        dX = plant.calc_dX(X)
                        dX0_nr = dX[0] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        dX1_nr = dX[1] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        plt.quiver(X[0]-dX0_nr/3-dX0_nr/2, X[1]-dX1_nr/3-dX1_nr/2, dX0_nr, dX1_nr, color='b')
                    if (abs(x_data[ss[0],1] + x_data[ss[1],1])/2 > 1 and abs(x_data[int((ss[0]+ss[1])/2),1]) < 5) or \
                       (abs(x_data[ss[0],0] + x_data[ss[1],0])/2 > 1 and abs(x_data[int((ss[0]+ss[1])/2),0]) < 5):
                        X = np.array([x_data[int((ss[0]+ss[1])*1/4),0], x_data[int((ss[0]+ss[1])*1/4),1]])
                        dX = plant.calc_dX(X)
                        dX0_nr = dX[0] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        dX1_nr = dX[1] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        plt.quiver(X[0]-dX0_nr/3, X[1]-dX1_nr/3, dX0_nr, dX1_nr, color='b')
                        X = np.array([x_data[int((ss[0]+ss[1])*3/4),0], x_data[int((ss[0]+ss[1])*3/4),1]])
                        dX = plant.calc_dX(X)
                        dX0_nr = dX[0] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        dX1_nr = dX[1] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        plt.quiver(X[0]-dX0_nr/3, X[1]-dX1_nr/3, dX0_nr, dX1_nr, color='b')
                    if abs(x_data[ss[0],1] - x_data[ss[1],1]) > 1 or abs(x_data[ss[0],0] - x_data[ss[1],0]) > 1:
                        X = np.array([x_data[int((ss[0]+ss[1])/2),0], x_data[int((ss[0]+ss[1])/2),1]])
                        dX = plant.calc_dX(X)
                        dX0_nr = dX[0] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        dX1_nr = dX[1] / (np.sqrt(dX[0]**2 + dX[1]**2) + 1e-6)
                        plt.quiver(X[0]-dX0_nr/3, X[1]-dX1_nr/3, dX0_nr, dX1_nr, color='b')
    x_data = odeint(plant.calc_dX, [+1e-3, +1e-3], time)
    plt.plot(x_data[:,0], x_data[:,1], color='k', linewidth = 0.6)
    x_data = odeint(plant.calc_dX, [-1e-3, -1e-3], time)
    plt.plot(x_data[:,0], x_data[:,1], color='k', linewidth = 0.6)

    #contour
    # plt.contour(X[0], X[1], dX[1], levels=[0], colors="red")
    # plt.contour(X[0], X[1], dX[0], levels=[0], colors="Blue")

    # plt.xlim([x_min-1, x_max+1])
    # plt.ylim([dx_min-1, dx_max+1])
    plt.xlim([x_min, x_max])
    plt.ylim([dx_min, dx_max])
    ax.set_title("LIPM phase portrait", fontsize = 16)
    ax.set_xlabel('position $x$', size = 14, weight = "light")
    ax.set_ylabel('velocity $\dot{x}$', size = 14, weight = "light")
    plt.grid()
    plt.show()