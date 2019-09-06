#!/usr/bin/env python

#by Haiping Jiang
#all the common def and classes are getting from pytorch frame(dnn.py in the class)
import torch
import torch.nn as nn
import torch.nn.functional as F

from torch.utils.data.dataset import Dataset
from torch.utils.data import DataLoader

import numpy as np
import math
import time
import rospy

from robot_sim.msg import RobotState
from robot_sim.srv import RobotAction
from robot_sim.srv import RobotActionRequest
from robot_sim.srv import RobotActionResponse

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

class my_robot(object):
    def __init__(self):
	rospy.wait_for_service('real_robot')
		
	self.get_action = rospy.ServiceProxy('real_robot', RobotAction)
	self.pub = rospy.Publisher("/robot_states", RobotState, queue_size=100)
	self.num_random_tests = 600		
	self.num_steps = 200    
	
	print "trying to get training data"
	self.features = [];
	self.labels = [];
	
	self.run()
	
	print "Finished 1"
	print "Start Training"
	
	self.network = MyDNN(self.features.shape[1], self.labels.shape[1])
	#fake robot service
	self.fake_robot_service = rospy.Service('fake_robot', RobotAction, self.callback)
	self.response_fake = RobotActionResponse()
	# reset the state
	self.response_fake.robot_state = self.features[0, :6].tolist
	# train the network
	self.dnn = MyDNNTrain(self.network)
	self.dnn.train(self.labels, self.features)

	print "Finished 2"

    def run(self):	
	for k in range(0, self.num_random_tests):
	    action = np.random.rand(1,3)
	    action[0,0] = (2 * action[0,0] - 1.0) * 1.0
	    action[0,1] = (2 * action[0,1] - 1.0) * 0.5
	    action[0,2] = (2 * action[0,2] - 1.0) * 0.25
	    self.single_test(action.reshape(3))
	    
	self.features = np.array(self.features)
	self.labels = np.array(self.labels)
		
    def single_test(self, action):
	req = RobotActionRequest()
	req.reset = True
	# send request to reset 
	resp = self.get_action(req)
	
	
	for i in range(self.num_steps):
	    
	    req = RobotActionRequest()
	    req.reset = False
	    req.action = action
	    
	    # get training data
	    self.features.append((np.append(resp.robot_state, req.action)).tolist())
	    # send request to move real_robot
	    resp = self.get_action(req)
	    self.labels.append(resp.robot_state)
	     
    def callback(self, req):
	# test reset
	if (not req.reset):
	    self.response_fake.robot_state = self.network.predict(np.append(self.response_fake.robot_state, req.action)).tolist()
	else:			
	    self.response_fake.robot_state = self.features[0, :6].tolist()	    
	return self.response_fake

'''
notes
class get_info():
self.xx=rospy.ServiceProxy('real_robot', RobotAction)
yy=self.xx(request)
request=RobotActionRequest()
request.reset=True/False
request.action=self.getaction
request.shape

self.pp=rospy.Service('fake_robot',RobotAction,self.callback)

def callback(self,request)
	return RobotActionResponse(numpy.array(1*6)
'''

class MyDNN(nn.Module):
    def __init__(self, input_dim,output_dim):
        super(MyDNN, self).__init__()
	para1=6
	#real_feature is 6, so I set parameter as 6 then times parameter to get nodes of the hidden layer
	#6 states and 3 actions(torque) 
        self.fc1 = nn.Linear(input_dim, 2*para1)
        self.fc2 = nn.Linear(2*para1, 2*para1)
        self.fc3 = nn.Linear(2*para1, output_dim)
	#(2*p,2*p) is the best,gets about 0.2
	#(4*p,2*p) is not so good
	#(2*p,4*p) is better than 4,2
	#(p,2*p) gets about 0.33
	#(2*p,p) 1.2
	#(4*p,4*p) 0.63 over parameter may be not so good
	#(3*p,3*p) 1.2
	#(9*p,9*p) 0.5, 0.375
	#(p,p) 1.0
	#(256,128) like shit
	#(9,6)is the best i think

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x
	
    def predict(self, features):
	""" Function receives a numpy array, converts to torch, returns numpy again"""
        self.eval()	#Sets network in eval mode (vs training mode)
        features = torch.from_numpy(features).float()
        return self.forward(features).detach().numpy()

class MyDataset(Dataset):
    def __init__(self, labels, features):
        super(MyDataset, self).__init__()
        self.labels = labels
        self.features = features

    def __len__(self):
        return self.features.shape[0]

    def __getitem__(self, idx):		#This tells torch how to extract a single datapoint from a dataset, Torch randomized and needs a way to get the nth-datapoint
        feature = self.features[idx]
        label = self.labels[idx]
        return {'feature': feature, 'label': label}
    
class MyDNNTrain(object):
    def __init__(self, network):	#Networks is of datatype MyDNN
        self.network = network
        self.learning_rate = .005 #0.01 is not good
        self.optimizer = torch.optim.SGD(self.network.parameters(), lr=self.learning_rate)
        self.criterion = nn.MSELoss()
        self.num_epochs = 50 #dont overfit
        self.batchsize = 30 #50 before, now 32,16,8 try each  
        self.shuffle = True #True is better
	
	#set the stop condition to save time
	self.delta_loss = 1 
	self.current_loss = 1		 
	self.loss_th = 0.003
	self.delta_loss_th = 0.0002

    def train(self, labels, features):
        self.network.train()
        dataset = MyDataset(labels, features)
        loader = DataLoader(dataset, shuffle=self.shuffle, batch_size = self.batchsize)
        for epoch in range(self.num_epochs):
	    if self.current_loss < self.loss_th or self.delta_loss < self.delta_loss_th:
	    	print "end training"
	    	break
            self.train_epoch(loader)

    def train_epoch(self, loader):
        total_loss = 0.0
        for i, data in enumerate(loader):
            features = data['feature'].float()
            labels = data['label'].float()
            self.optimizer.zero_grad()
            predictions = self.network(features)
            loss = self.criterion(predictions, labels)
            loss.backward()
            total_loss += loss.item()
            self.optimizer.step()
	self.delta_loss = self.current_loss - total_loss/i
	self.current_loss = total_loss/i
        print 'loss', self.current_loss

'''
def function(x1, x2):
    return math.cos(x1) * math.cos(x2)

def reject(x1, x2):
    #if (x1<0 and x2 > 0): return True

    # s1 = 0
    # s2 = 0
    # dist = 1
    # if ( math.sqrt(np.dot([s1-x1,s2-x2],[s1-x1,s2-x2])) < dist): return True
    return False

def plot_prediction(network):
    X = np.arange(-math.pi, math.pi, 0.1)
    Y = np.arange(-math.pi, math.pi, 0.1)
    X, Y = np.meshgrid(X, Y)

    features = np.concatenate((X.reshape(-1, 1), Y.reshape(-1,1)), axis=1)
    predictions = network.predict(features).reshape(63, 63)

    fig1 = plt.figure()
    ax1 = fig1.gca(projection='3d')
    surf = ax1.plot_surface(X, Y, predictions)

    fig2 = plt.figure()
    ax2 = fig2.gca(projection='3d')
    surf = ax2.plot_surface(X, Y, np.vectorize(function)(X,Y))

    plt.show()
'''
#this part of dnn is useless

def main():
    rospy.init_node('fake_robot',anonymous=True)
    start_time = time.time()
    my_robot()
    elapsed_time = time.time() - start_time
    print "time cost: " + time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
    print "Fake robot ready"
    rospy.spin()
    '''
    network = MyDNN(2)
    trainer = MyDNNTrain(network)
    features = []
    while len(features) < 1000:
        x1 = np.random.uniform(-math.pi, math.pi)
        x2 = np.random.uniform(-math.pi, math.pi)
        if reject(x1,x2): continue
        features.append([x1, x2])
    features = np.asarray(features)
    labels = np.vectorize(function)(features[:,0], features[:,1]).reshape(-1,1)
    trainer.train(labels, features)
    plot_prediction(network)
    '''

if __name__ == '__main__':
    main()
