#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class LinearInvertedPendulum(object):
    h = 1.5 # height of cog
    g = 9.8 # gravity
    omega2 = g/h
    omega = np.sqrt(omega2)

    A = np.array([[0, 1],[omega2, 0]]) # state matrix
    B = np.array([[0],[-omega2]]) # input matrix

    def __init__(self, x, x_ref, u_ref, dt = 0.01, x_dot = 0., x_ddot = 0):
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
