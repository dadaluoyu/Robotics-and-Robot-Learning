#!/usr/bin/env python

import numpy as np
import time
import rospy

from inverted_pendulum_cart_numpy_simulated_dynamics import InvertedPendulumCartNumpySimulatedDynamics
from simulated_robot import SimulatedRobot

from robot_sim.msg import RobotState
from robot_sim.srv import RobotAction
from robot_sim.srv import RobotActionRequest
from robot_sim.srv import RobotActionResponse


class CartPoleRobot(object):
    def __init__(self):
        self.dynamics = InvertedPendulumCartNumpySimulatedDynamics(delta_t=0.02, cart_mass=1.0, pendulum_mass=0.1,
                                                                   length=1.0, linearize=False)
        self.robot = SimulatedRobot(dynamics=self.dynamics,
                                    data_names=None,
                                    steps_per_action=1)
        #State = [X_cart, Q_pole, X_dot_cart, Q_dot_pole] Q is the pole angle in rads, X is position of the cart
        self.initial_state = np.zeros((4,1))  
        self.reset_robot(0.0)
        self.action_service = rospy.Service('cartpole_robot', RobotAction, self.robot_action_service)
        self.pub = rospy.Publisher("robot_state", RobotState, queue_size=100)

    def reset_robot(self, pole_angle):
        state = np.asarray(self.initial_state)
        state[1] = pole_angle
        self.robot.set_state(state.reshape((-1, 1)))


    def robot_action_service(self, req):
      response = RobotActionResponse()
      message = RobotState()
      if req.reset_robot:
        self.reset_robot(req.reset_pole_angle)
      else:
        self.robot.take_action(np.asarray(req.action).reshape((-1, 1)))
      response.robot_state = self.robot.get_state()[0]
      message.robot_state = self.robot.get_state()[0]
      self.pub.publish(message)
      return response

if __name__ == '__main__':
    rospy.init_node('cartpole_robot', anonymous=True)
    r = CartPoleRobot()
    print "Cartpole robot now spinning"
    rospy.spin()



