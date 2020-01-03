#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class LinearInvertedPendulum(object):
    gravity = 9.8 # gravity

    def __init__(self, height  = 1.5, weight = 50):
        self.height = height
        self.omega2 = self.gravity / self.height
        self.omega = np.sqrt(self.omega2)

        self.A = np.array([[0, 1],[self.omega2, 0]]) # state matrix
        self.B = np.array([[0],[-self.omega2]]) # input matrix

    
    def init_state(self, x, x_ref, u_ref, dt = 0.01, x_dot = 0., x_ddot = 0):
        self.X = np.array([[x], [x_dot]])
        self.X_dot = np.array([[x_dot], [x_ddot]])
        self.u = self.bestCOG_Regulator(x_ref, u_ref)
        self.dT = dt

    def do_action(self, x_ref, u_ref):
        self.u = self.bestCOG_Regulator(x_ref, u_ref)
        self.update_state()

    def update_state(self):
        self.X_dot = self.A @ self.X + self.B @ self.u
        self.X =self.X + self.X_dot * self.dT 

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
            x_dot = np.zeros((X[:,0,0].size,X[0,0,:].size, X[0,:,0].size), dtype=np.float64)
            for i in range(X[:,0,0].size):
                x_tmp =  np.empty((0,X[0,0,:].size, X[0,:,0].size), dtype=np.float64)
                for j in range(X[:,0,0].size):
                    x_tmp = np.append(x_tmp, np.array([self.A[j,i] * X[i]]), axis = 0)    
                x_dot += x_tmp
            return x_dot
