#!/usr/bin/env python

import numpy as np
import time
import rospy
import random

from robot_sim.msg import RobotState
from robot_sim.srv import RobotAction
from robot_sim.srv import RobotActionRequest
from robot_sim.srv import RobotActionResponse
from robot_sim.srv import RobotPolicy
from robot_sim.srv import RobotPolicyRequest
from robot_sim.srv import RobotPolicyResponse


class RobotExecutive(object):
    def __init__(self):
        self.num_random_tests = 11
        self.steps_max = 200
        self.q_max = 6*np.pi/180
        self.start_q_max = 3*np.pi/180
        self.x_max = 1.2
        self.cartpole_action_service = rospy.ServiceProxy('cartpole_robot', RobotAction, persistent=True)
        self.cartpole_policy_service = rospy.ServiceProxy('cartpole_policy', RobotPolicy,  persistent=True)
        self.steps_score=np.zeros(self.num_random_tests)


    def print_final_score(self, ):
        score1 = np.median(self.score1)
        score2 = np.median(self.score2)
        score3 = np.median(self.score3)
        print "Median balanced steps for Test 1 = ",score1
        print "Median balanced steps for Test 2 = ",score2
        print "Median balanced steps for Test 3 = ",score3
        grade = 0
        if score1>=150:
            grade+=10
        elif score1>=100:
            grade+=8
        elif score1>=50:
            grade+=5

        if score2>=150:
            grade+=4
        elif score2>=100:
            grade+=2
        elif score2>=50:
            grade+=1

        if score3>=150:
            grade+=6
        elif score3>=100:
            grade+=3
        elif score3>=50:
            grade+=1
        print "Final score:",grade,"/ 20"


    def get_random_sign(self):
        return 1.0 if random.random() < 0.5 else -1.0

    def state_out_of_bounds(self, state):
        return abs(state[0]) > self.x_max or abs(state[1]) > self.q_max 

    def test(self, test_interval_min, test_interval_max):
        steps_score=np.zeros(self.num_random_tests)
        for k in range(0,self.num_random_tests):
            #Pick an initial state to start from 
            req = RobotActionRequest()
            req.reset_robot = True
            req.reset_pole_angle = np.random.uniform(np.deg2rad(test_interval_min), np.deg2rad(test_interval_max))*self.get_random_sign()
            #print "Initial pole angle is", np.rad2deg(req.reset_pole_angle), "degrees"
            response = self.cartpole_action_service(req)
            for n in range(0, self.steps_max):
                pol = RobotPolicyRequest()
                pol.robot_state = response.robot_state
                dqn_response = self.cartpole_policy_service(pol)
                req = RobotActionRequest()
                req.action = dqn_response.action
                req.reset_robot=False
                response = self.cartpole_action_service(req)
                if self.state_out_of_bounds(response.robot_state):
                    #print "Cartpole was balanced for",n,"number of steps"
                    steps_score[k]=n
                    time.sleep(1)
                    break
                if n==self.steps_max-1:
                    #print "Episode ended successfully!"
                    steps_score[k]=self.steps_max
                    time.sleep(1)
                time.sleep(0.01)
        return steps_score

    def run(self):
        print "Test 1: pole with initial angle between 0 and 1 degrees. Will test for",self.num_random_tests,"trials"
        self.score1 = self.test(0.0, 1.0)     #Init angle 0 to 1 degrees
        print "Number of balanced steps for each trial =", self.score1
        print "Test 2: pole with initial angle between 1 and 2 degrees. Will test for",self.num_random_tests,"trials"
        self.score2 = self.test(1.0, 2.0)     #Init angle 1 to 2 degrees
        print "Number of balanced steps for each trial =", self.score2
        print "Test 3: pole with initial angle between 2 and 3 degrees. Will test for",self.num_random_tests,"trials"
        self.score3 = self.test(2.0, 3.0)     #Init angle 2 to 3 degrees
        print "Number of balanced steps for each trial =", self.score3
    
 
if __name__ == '__main__':
    rospy.init_node('robot_executive', anonymous=True)
    np.random.seed(rospy.get_rostime().secs)
    executive = RobotExecutive()
    executive.run()
    time.sleep(1)
    executive.print_final_score()
    print "Executive done!"








