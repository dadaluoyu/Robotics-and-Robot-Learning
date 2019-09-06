#!/usr/bin/env python

import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from torch.utils.data.dataset import Dataset
from torch.utils.data import DataLoader

import numpy as np
import math
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

class my_robot(object):
    def __init__(self):
	rospy.wait_for_service('cartpole_robot')
		
	self.cartpole_action_service = rospy.ServiceProxy('cartpole_robot', RobotAction, persistent=True)
     
	#self.pub = rospy.Publisher("/robot_states", RobotState, queue_size=100)
	
        self.steps_max = 400
	#training steps
        self.q_max = 6*np.pi/180
        self.start_q_max = 3*np.pi/180
        self.x_max = 1.2
	#same as executive.py
		
	print "trying to get training data"
	
	self.DQN = DQN()
        self.score = []
	#DQN is the agent, and score records rewards

	self.run()
	
	print "Finished part 1"
	
	print "Start Training"
	
	self.policy_service = rospy.Service('cartpole_policy', RobotPolicy, self.callback)
		
	print "Finished part 2"
    	

    def get_random_sign(self):
        return 1.0 if random.random() < 0.5 else -1.0

    def state_out_of_bounds(self, state):
        return abs(state[0]) > self.x_max or abs(state[1]) > self.q_max 


    def test(self, test_interval_min, test_interval_max):
	#same as executive.py
	
	for episode in range(300):
            #Pick an initial state to start from 
            req = RobotActionRequest()
            req.reset_robot = True
            req.reset_pole_angle = np.random.uniform(np.deg2rad(test_interval_min), np.deg2rad(test_interval_max))*self.get_random_sign()
            #print "Initial pole angle is", np.rad2deg(req.reset_pole_angle), "degrees"
            s_t = self.cartpole_action_service(req)
	    final_reward=0
            for n in range(0, self.steps_max):
		a_t=self.DQN.execute(s_t.robot_state)
		
                
                req = RobotActionRequest()
                req.action = [a_t]
                req.reset_robot=False

                s_t1 = self.cartpole_action_service(req)
                r_t1=1
   		
                if self.state_out_of_bounds(s_t1.robot_state):
                    r_t1=-1
                self.DQN.store(s_t.robot_state, a_t, r_t1, s_t1.robot_state)
		
		if self.state_out_of_bounds(s_t1.robot_state):
		    break
		final_reward+=1
		s_t=s_t1
		#store training data then learn from it
              	self.DQN.learn()

	    self.score.append(final_reward)
	    if np.median(self.score[-21:])>195:
		#if score is good, break
                break
 
    def run(self):   
        self.test(0.0, 3.0) 
	#run the test process in degrees of 0-3
       
    def callback(self, req):
	
	self.response_dqn=RobotPolicyResponse()
	self.response_dqn.action = [self.DQN.execute(req.robot_state)]		
	#callback same as project2	    
	return self.response_dqn

#first wirte the DNN and DQN
#then fix the input data use DQN function
#the order can be changed

class MyDNN(nn.Module):
    def __init__(self, input_dim,output_dim):
        super(MyDNN, self).__init__()
	para1=128 
	#set the network of (4,128,128,2)
        self.fc1 = nn.Linear(input_dim, para1)
	self.fc1.weight.data.normal_(0,0.1)
        self.fc2 = nn.Linear(para1, para1)
	self.fc2.weight.data.normal_(0,0.1)
        self.fc3 = nn.Linear(para1, output_dim)
	self.fc3.weight.data.normal_(0,0.1)
	#normalize is important

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class DQN(object):
    def __init__(self):
        self.Gamma=0.8
    
    	self.decay=200
    	
    	self.batch=[]
    	self.capacity=2000
    	self.batch_size=64
    	self.Q_A_net=MyDNN(4,2)
    	self.target_net=MyDNN(4,2)
	
    	self.optimizer=optim.Adam(self.Q_A_net.parameters(),lr=0.001)
        #use optimizer method adam
    	self.steps=0


    def execute(self,s_t):
	self.steps+=1
	ep=0.01+(0.2-0.01)*(math.exp(-1.0*self.steps/self.decay))
	#use decaying epsilon
	if random.random()<ep:
	    a_t=random.randrange(2)
	else:
	    a_t=torch.argmax(self.Q_A_net(torch.tensor(s_t,dtype=torch.float))).item()
	return 20*(a_t-0.5)

    def store(self, s_t, a_t, r_t1, s_t1):
        a_t = a_t/(20)+0.5
        if len(self.batch)==self.capacity:
            self.batch.pop(0)
        self.batch.append([s_t, a_t, r_t1, s_t1])
	#Experience replay
	#store transition in D

    def learn(self):
	k=100
        if self.steps % k == 0:
            self.target_net.load_state_dict(self.Q_A_net.state_dict())
	    #every k episodes:set target_net=Q_A

        if (len(self.batch)) < self.batch_size:
            return

        samples = random.sample(self.batch, self.batch_size)
        s_t, a_t, r_t1, s_t1 = zip(*samples)
        s_t = torch.tensor( s_t, dtype=torch.float)
        a_t = torch.tensor( a_t, dtype=torch.long).view(self.batch_size, -1)
        r_t1 = torch.tensor( r_t1, dtype=torch.float).view(self.batch_size, -1)
        s_t1 = torch.tensor( s_t1, dtype=torch.float)
        y_from_T = r_t1 + self.Gamma * torch.max( self.Q_A_net(s_t1).detach(), dim=1)[0].view(self.batch_size, -1)
        y_from_A = self.Q_A_net(s_t).gather(1, a_t)
        loss = F.smooth_l1_loss(y_from_A, y_from_T)
	#compute the loss function as showed in the slides

        self.optimizer.zero_grad()
        loss.backward()
        for param in self.Q_A_net.parameters():
            param.grad.data.clamp_(-1, 1)
        self.optimizer.step()

	#use the select_action function in the pytorch tutorial
'''

policy_net = DQN(screen_height, screen_width, n_actions).to(device)
target_net = DQN(screen_height, screen_width, n_actions).to(device)
target_net.load_state_dict(policy_net.state_dict())

optimizer = optim.RMSprop(policy_net.parameters())
memory = ReplayMemory(10000)

target_network.load_state_dict(act_network.state_dict())

terminal state

torch.manual_seed(seed)


ep=max(ep*decay_const,0.2) 




def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * \
        math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            # t.max(1) will return largest column value of each row.
            # second column on max result is index of where max element was
            # found, so we pick action with the larger expected reward.
            return policy_net(state).max(1)[1].view(1, 1)
    else:
        return torch.tensor([[random.randrange(n_actions)]], device=device, dtype=torch.long)

'''



def main():
    
    rospy.init_node('learn_dqn', anonymous=True) 
    random.seed(1)
    np.random.seed(1)
    torch.manual_seed(1)
    #use fixed random seed
    start_time = time.time()
    
    my_robot()
    elapsed_time = time.time() - start_time
    print "time cost: " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
    print "dqn ready"
    rospy.spin()

if __name__ == '__main__':
    main()


