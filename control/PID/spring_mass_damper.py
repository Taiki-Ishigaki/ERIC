#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

class SpringMassDamper(object):
    m = 50.0
    k = 50.0
    d = 25.0
    A = np.array([[0, 1],[-k/m, -d/m]]) # state matrix
    B = np.array([[0],[1/m]]) # input matrix

    def __init__(self, x, x_dot = 0.,  x_ddot = 0., dt = 0.001):
        self.X = np.array([[x], [x_dot]])
        self.X_dot = np.array([[x_dot], [x_ddot]])
        self.dT = dt
        self.u = 0
    
    def init_system(self, mass, elasticity, viscocity):
        self.m = mass
        self.k = elasticity
        self.d = viscocity
        self.A = np.array([[0, 1],[-self.k/self.m, -self.d/self/m]]) # state matrix
        self.B = np.array([[0],[1/self.m]]) # input matrix

    def do_action(self, u):
        self.u = u

    def update_state(self):
        self.X_dot = self.A @ self.X + self.B @ self.u
        self.X = self.X + self.X_dot * self.dT 

    def get_X(self):
        return self.X
    
    def get_u(self):
        return self.u

class PID(object):
    def __init__(self, kp, ki, kd):
        self.k = np.array([[kp, kd]])

    def controller(self, x, x_ref, u_ref):
        return u_ref - self.k @ (x - x_ref)

if __name__ == '__main__':
    x = np.array([[0.5],[0.5]])
    u = np.array([[0.0]])
    x_ref = np.array([[0.0]])
    x_start = 0.0
    dt = 0.01
    period = 300
    time_step = (int) (period / dt)
    plant = SpringMassDamper(0.5, 0.5)
    pid = PID(50, 20, 25)
    X_history = np.array(plant.get_X())
    u_history = np.array(plant.get_u())
    for i in range(time_step-1):
        plant.do_action(pid.controller(plant.get_X(), x_ref, u))
        plant.update_state()
        u_history = np.append(u_history, plant.get_u())
        X_history = np.append(X_history, plant.get_X(), axis=1)

    t = np.linspace(0, time_step*dt, time_step)

    fig = plt.figure()

    plt.plot(t, X_history[0], label="position")
    plt.plot(t, X_history[1], label="velosity")
    # plt.plot(t, u_history, label="control")
    plt.legend()
    plt.show()