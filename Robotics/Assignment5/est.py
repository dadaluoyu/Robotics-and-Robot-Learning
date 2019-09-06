#!/usr/bin/env python

# Columbia Engineering
# MECS 4602 - Fall 2018

import math
import numpy
import time

import rospy

from state_estimator.msg import RobotPose
from state_estimator.msg import SensorData

class Estimator(object):
    def __init__(self):

        # Publisher to publish state estimate
        self.pub_est = rospy.Publisher("/robot_pose_estimate", RobotPose, queue_size=1)

        # Initial estimates for the state and the covariance matrix
        self.x = numpy.zeros((3,1))
        self.P = numpy.zeros((3,3))

        # Covariance matrix for process (model) noise
        self.V = numpy.zeros((3,3))
        self.V[0,0] = 0.0025
        self.V[1,1] = 0.0025
        self.V[2,2] = 0.005

        self.step_size = 0.01

        # Subscribe to command input and sensory output of robot
        rospy.Subscriber("/sensor_data", SensorData, self.sensor_callback)
        
    	# This function gets called every time the robot publishes its control 
    	# input and sensory output. You must make use of what you know about 
    	# extended Kalman filters to come up with an estimate of the current
   	# state of the robot and covariance matrix.
    	# The SensorData message contains fields 'vel_trans' and 'vel_ang' for
   	# the commanded translational and rotational velocity respectively. 
    	# Furthermore, it contains a list 'readings' of the landmarks the
    	# robot can currently observe

    def estimate(self, sens):

        #### ----- YOUR CODE GOES HERE ----- ####
	
	#print(sens)
	#sens.vel_trans
	#sens.vel_ang
	#print(sens.vel_ang)#[0],[1],[2]
	#landmark1=sens.readings[0]
	#sens.readings[0].landmark=x:5.0 y:5.0
	#sens.readings[0].landmark.x=5.0
	#sens.readings[0].range
	#sens.readings[0].bearing
	#reading part
	tt=self.step_size
	
	
	#print(self.x)
	
	F=numpy.zeros((3,3))
	F[0,0]=1
	F[1,1]=1
	F[2,2]=1
	F[0,2]=-tt*sens.vel_trans*numpy.sin(self.x[2])
	F[1,2]=tt*sens.vel_trans*numpy.cos(self.x[2])
	
	hat_x = numpy.zeros((3,1))
	hat_x[0]=self.x[0]+tt*sens.vel_trans*numpy.cos(self.x[2])
	hat_x[1]=self.x[1]+tt*sens.vel_trans*numpy.sin(self.x[2])
	hat_x[2]=self.x[2]+tt*sens.vel_ang

	H=numpy.zeros((2,3))
	#print(H)
	Final_H=[]
	yy=[]
	kk=0
	#print(len(sens.readings))
	for ii in range(len(sens.readings)):
	    H=numpy.zeros((2,3))
	    range_est=math.sqrt((hat_x[0]-sens.readings[ii].landmark.x)*(hat_x[0]-sens.readings[ii].landmark.x)+(hat_x[1]-sens.readings[ii].landmark.y)*(hat_x[1]-sens.readings[ii].landmark.y))
	    #print(sens.readings[ii].range)
	    #print(range_est)
	    if (range_est>0.1):
		kk=kk+1
	        #mid=numpy.square((sens.readings[ii].landmark.y-hat_x[1])/(sens.readings[ii].landmark.x-hat_x[0]))
		
		
		#print(range_est)
	        #print(sens.readings[ii].range)
		aa=hat_x[0]-sens.readings[ii].landmark.x
		bb=hat_x[1]-sens.readings[ii].landmark.y
	        H[0,0]=(hat_x[0]-sens.readings[ii].landmark.x)/range_est
	        H[0,1]=(hat_x[1]-sens.readings[ii].landmark.y)/range_est
	        H[0,2]=0
	        H[1,0]=(sens.readings[ii].landmark.y-hat_x[1])/numpy.square(range_est)
		#H[1,0]=-H[0,1]/sens.readings[ii].range
		#print(H[1,0])
		#print(-H[0,1]/sens.readings[ii].range)
	        H[1,1]=-(sens.readings[ii].landmark.x-hat_x[0])/numpy.square(range_est)
	        H[1,2]=-1
		range_v=math.sqrt((hat_x[0]-sens.readings[ii].landmark.x)*(hat_x[0]-sens.readings[ii].landmark.x)+(hat_x[1]-sens.readings[ii].landmark.y)*(hat_x[1]-sens.readings[ii].landmark.y))
	        bearing_v=math.atan2(-bb,-aa)-hat_x[2]
	        
	        midv=numpy.vstack((range_v,bearing_v))
	        if midv[1]<-numpy.pi:
	 	    midv[1]=midv[1]+2*numpy.pi
		if midv[1]>numpy.pi:
	 	    midv[1]=midv[1]-2*numpy.pi
		if kk==1:
	            Final_H=H
		    addy=numpy.vstack((sens.readings[ii].range,sens.readings[ii].bearing))
		    if addy[1]<-numpy.pi:
	 	    	addy[1]=addy[1]+2*numpy.pi
		    if addy[1]>numpy.pi:
	 	    	addy[1]=addy[1]-2*numpy.pi
		    yy=addy
		    bigv=midv
		    #print(hat_x)
		    #print(Final_H)
	 	    
		else:
		    Final_H=numpy.vstack((Final_H,H))
		    addy=numpy.vstack((sens.readings[ii].range,sens.readings[ii].bearing))
	 	    if addy[1]<-numpy.pi:
	 	    	addy[1]=addy[1]+2*numpy.pi
		    if addy[1]>numpy.pi:
	 	    	addy[1]=addy[1]-2*numpy.pi
		    yy=numpy.vstack((yy,addy))
		    bigv=numpy.vstack((bigv,midv))
	 	    #print(Final_H)
		    #print(yy)
	#print(Final_H)
	if kk==0:
	    w=numpy.zeros((2,2))
	    w[0,0]=0
	    
	    w[1,1]=0
	    
        else:
	    w=numpy.zeros((2*kk,2*kk))
	   
	    w[0,0]=0.1
	    w[1,1]=0.05
	    if kk==2:
	        w[2,2]=0.1
	        w[3,3]=0.05
            if kk==3:
	        w[2,2]=0.1
	        w[3,3]=0.05
                w[4,4]=0.1
	        w[5,5]=0.05
	    
	#w=numpy.zeros((2,2))
	
	
	hat_P=numpy.dot(numpy.dot(F,self.P),numpy.transpose(F))+self.V
	if kk==0:
	    Final_H=numpy.zeros((2,3))
	    yy=numpy.zeros((1,2))
	    
	    #S=numpy.dot(numpy.dot(Final_H,hat_P),numpy.transpose(Final_H))+w
	    #print(S)
	    #R=numpy.dot(numpy.dot(hat_P,numpy.transpose(Final_H)),numpy.linalg.inv(S))
	    #self.x=hat_x
	    self.x=hat_x
	    self.P=hat_P#-numpy.dot(numpy.dot(R,Final_H),hat_P)
	    #self.P=hat_P
	else:    
	    
	    #print(Final_H)
	    v=yy-bigv
	 
	    for ii in range(1,len(v)+1,2):
		#print(ii)
	    	if v[ii]<-numpy.pi:
	 	    v[ii]=v[ii]+2*numpy.pi
		if v[ii]>numpy.pi:
	 	    v[ii]=v[ii]-2*numpy.pi
            	
	    S=numpy.dot(numpy.dot(Final_H,hat_P),numpy.transpose(Final_H))+w
	    #print(S)
	    R=numpy.dot(numpy.dot(hat_P,numpy.transpose(Final_H)),numpy.linalg.inv(S))
	   
	 
	    #print(R)
	    #print(v)
            self.x=hat_x+numpy.dot(R,v)
	    #self.x=hat_x+aa
	    #print(self.x)
	    self.P=hat_P-numpy.dot(numpy.dot(R,Final_H),hat_P)

	kk=0

	#self.x=hat_x
	#self.P=hat_P

	#print(numpy.square(4))
	#v=yy-numpy.dot(Final_H,hat_x)  
	#S=numpy.dot(numpy.dot(Final_H,hat_P),numpy.transpose(Final_H))+w   
	#R=numpy.dot(numpy.dot(hat_P,numpy.transpose(Final_H)),numpy.linalg.inv(S))
	#self.x=hat_x+numpy.dot(R,v)
        #self.P=hat_P-numpy.dot(numpy.dot(R,Final_H),hat_P)
	
	#print(self.x)
	
        #### ----- YOUR CODE GOES HERE ----- ####
    
    def sensor_callback(self,sens):

        # Publish state estimate 
        self.estimate(sens)
        est_msg = RobotPose()
        est_msg.header.stamp = sens.header.stamp
        est_msg.pose.x = self.x[0]
        est_msg.pose.y = self.x[1]
        est_msg.pose.theta = self.x[2]
	#print(est_msg)
        self.pub_est.publish(est_msg)

if __name__ == '__main__':
    rospy.init_node('state_estimator', anonymous=True)
    est = Estimator()
    rospy.spin()
