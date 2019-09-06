#!/usr/bin/env python

import rospy
import threading
import time
from robot_sim.msg import RobotState
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
from math import sin
from math import cos
from math import pi


class CartPoleGUI(object):
    def __init__(self):
        self.sub = rospy.Subscriber("robot_state", RobotState, self.update_state, queue_size = 100)
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111)
        self.ax.axis('equal')
        plt.xlim(-1.5, 1.5)
        plt.ylim(-1, 2)
        self.fig.canvas.draw()
        plt.show(block=False)
        self.x = 0 
        self.q = 0

    def update_state(self,msg):
        self.x = msg.robot_state[0]
        self.q = msg.robot_state[1]

    def render(self):
        plt.figure(self.fig.number)
        self.ax.clear()
        self.ax.axhline(y=-0.05, color='k')
        self.ax.add_patch(plt.Rectangle(xy=[-0.15 + self.x, -0.1], height=0.2, width=0.3, fill=True, facecolor='slategray'))
        l = 1.0
        self.ax.add_line(mlines.Line2D((self.x, self.x - l*sin(self.q)), (0, l*cos(self.q)), color='firebrick', linewidth=10))
        self.fig.canvas.draw()

def my_thread():
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('cartpole_gui', anonymous=True)
    gui = CartPoleGUI()
    my_thread = threading.Thread(target=my_thread)
    my_thread.daemon = True
    my_thread.start()

    while not rospy.is_shutdown():
        gui.render()
        time.sleep(0.005)