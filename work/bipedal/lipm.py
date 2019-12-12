#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class LinearInvertedPendulum(object):
    # actions = [0, 1, 2]
    h = 1.5 # height of cog
    g = 9.8 # gravity
    omega2 = g/h
    omega = np.sqrt(omega2)

    A = np.array([[0, 1],[omega2, 0]]) # state matrix
    B = np.array([[0],[-omega2]]) # input matrix
    dT = 0.01

    def __init__(self, x, u = 0, x_dot = 0., x_ddot = 0):
        self.X = np.array([[x], [x_dot]])
        self.X_dot = np.array([[x_dot], [x_ddot]])
        self.u = np.array([[u]])

    def do_action(self, x_ref):
        self.u = self.bestCOG_Regulator(x_ref)
        self.update_state()

    def update_state(self):
        self.X_dot = self.A.dot(self.X) + self.B.dot(self.u)
        self.X =self.X +  np.dot(self.dT,self.X_dot)

    def bestCOG_Regulator(self, X_ref):
        self.alpha = 2.0
        self.F = np.array([[1.0, 1.0/self.omega2]])*self.alpha
        return -self.F.dot((X_ref - self.X))

    def get_X(self):
        return self.X
    
    def get_u(self):
        return self.u

def video(x_hs, u_hs, h, t):
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect='equal', autoscale_on=False, xlim=(-2, 2), ylim=(-0.5, 3.5))
    ax.grid()
    line, = ax.plot([], [], '-r', lw=2)
    time_text = ax.text(0.02, 0.95, 'time = 0.0', transform=ax.transAxes)

    # circle
    circle, = ax.plot([], [], '-r', lw=2)
    radius = 0.1
    angles = np.arange(0.0, np.pi * 2.0, np.radians(3.0))
    ox = radius * np.cos(angles) 
    oy = radius * np.sin(angles)

    def init():
        line.set_data([], [])
        circle.set_data([], [])
        time_text.set_text('')
        return line, time_text

    def animate(i):
        line.set_data([u_hs[i], x_hs[i]],[0, h])
        circle.set_data([ox + x_hs[i]],[ oy + h])
        time_text.set_text('time = {0:.1f}'.format(i*t))
        return line, time_text

    ani = animation.FuncAnimation(fig, animate, frames=range(len(x_hs)),
                                  interval=t, blit=False, init_func=init)
    plt.show()



if __name__ == '__main__':
    plant = LinearInvertedPendulum(0)
    angle_history = [np.pi/12]
    X_history = np.array([[0.],[0.]])
    u_history = np.array([[0.]])
    for i in range(1000):
        plant.do_action(np.array([[0.5],[0.]]))
        u_history = np.append(u_history, plant.get_u(), axis=1)
        X_history = np.append(X_history, plant.get_X(), axis=1)

    dt = 0.01
    t = np.linspace(0, 1000*dt, 1001)

    #print(u_history.shape)
    
    plt.plot(t, X_history[0], t, X_history[1], t, u_history[0])
    plt.show()
    
    video(X_history[0], u_history[0], plant.h, 1)