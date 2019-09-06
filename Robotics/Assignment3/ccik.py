#!/usr/bin/env python

import math
import numpy
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from cartesian_control.msg import CartesianCommand
from urdf_parser_py.urdf import URDF
import random
import tf
from threading import Thread, Lock
import time

'''This is a class which will perform both cartesian control and inverse
   kinematics'''
class CCIK(object):
    def __init__(self):
	#Load robot from parameter server
        self.robot = URDF.from_parameter_server()

	#Subscribe to current joint state of the robot
        rospy.Subscriber('/joint_states', JointState, self.get_joint_state)

	#This will load information about the joints of the robot
        self.num_joints = 0
        self.joint_names = []
        self.q_current = []
        self.joint_axes = []
        self.get_joint_info()

	#This is a mutex
        self.mutex = Lock()

	#Subscribers and publishers for for cartesian control
        rospy.Subscriber('/cartesian_command', CartesianCommand, self.get_cartesian_command)
        self.velocity_pub = rospy.Publisher('/joint_velocities', JointState, queue_size=10)
        self.joint_velocity_msg = JointState()

        #Subscribers and publishers for numerical IK
        rospy.Subscriber('/ik_command', Transform, self.get_ik_command)
        self.joint_command_pub = rospy.Publisher('/joint_command', JointState, queue_size=10)
        self.joint_command_msg = JointState()

    '''This is a function which will collect information about the robot which
       has been loaded from the parameter server. It will populate the variables
       self.num_joints (the number of joints), self.joint_names and
       self.joint_axes (the axes around which the joints rotate)'''
    def get_joint_info(self):
        link = self.robot.get_root()
        while True:
            if link not in self.robot.child_map: break
            (joint_name, next_link) = self.robot.child_map[link][0]
            current_joint = self.robot.joint_map[joint_name]
            if current_joint.type != 'fixed':
                self.num_joints = self.num_joints + 1
                self.joint_names.append(current_joint.name)
                self.joint_axes.append(current_joint.axis)
            link = next_link

    '''This is the callback which will be executed when the cartesian control
       recieves a new command. The command will contain information about the
       secondary objective and the target q0. At the end of this callback, 
       you should publish to the /joint_velocities topic.'''
    def get_cartesian_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR CARTESIAN CONTROL HERE
	#cartesian_command /joint_velocities
 	msg=JointState() 
	msg.name=self.joint_names
	
	mid_q=[command.x_target.rotation.x,command.x_target.rotation.y,command.x_target.rotation.z,command.x_target.rotation.w]
	
	R_desired=tf.transformations.quaternion_matrix(mid_q)
	#print(R_desired)
	#R_desired[0,3]=command.x_target.translation.x
	#R_desired[1,3]=command.x_target.translation.y
	#R_desired[2,3]=command.x_target.translation.z
	b_T_eed=R_desired
	
	
	R_desired=R_desired[0:3,0:3]
        
	joint_transforms, b_T_ee=self.forward_kinematics(self.q_current)
	
	
	eec_T_b=numpy.linalg.inv(b_T_ee)
	c_T_d=numpy.dot(b_T_eed,eec_T_b)
	angle,axis=self.rotation_from_matrix(c_T_d)
	aa=angle*axis
	#print(aa)

	
        #x_matrix=numpy.dot(numpy.linalg.inv(b_T_ee),b_T_ee_desired)
        #x_matrix=numpy.dot(b_T_ee_desired,numpy.linalg.inv(b_T_ee))
	#print(x_matrix)
	delta_x_t=[command.x_target.translation.x,command.x_target.translation.y,command.x_target.translation.z]-b_T_ee[0:3,3]
        #delta_x=[x_matrix[0,3],x_matrix[1,3],x_matrix[2,3]]
	#print(delta_x)
        #delta_x1=numpy.array([command.x_target.translation.x-b_T_ee[0,3],command.x_target.translation.y-b_T_ee[1,3],command.x_target.translation.z-b_T_ee[2,3]])
	#print(delta_x1)
	R_current=b_T_ee[0:3,0:3]
	
	#R_mid=numpy.dot(numpy.linalg.inv(R_current),R_desired)
	#print(R_mid)
        R_mid=numpy.dot(R_desired,numpy.linalg.inv(R_current))
	#print(R_mid)
	c_angle, c_axis=self.rotation_from_matrix(R_mid[0:3,0:3])
	#delta_angle=c_angle*c_axis
	delta_angle=aa
	#print(delta_angle)
	delta_x=numpy.hstack((delta_x_t,delta_angle))
 	#print(delta_x)

	e_R_b=numpy.linalg.inv(R_current)
	
	data0=numpy.zeros((3,3))
        mid_RR1=numpy.hstack((e_R_b,data0))
	
        mid_RR2=numpy.hstack((data0,e_R_b))
	
        mid_RR=numpy.vstack((mid_RR1,mid_RR2))
        #print(mid_RR)
        
	for ii in range(3):
	    testabs=numpy.abs(delta_x[ii])
	    if testabs>0.1*numpy.pi:
                delta_x[ii]=0.1*delta_x[ii]/testabs
        
       
	for ii in range(3,6):    
            testabs=numpy.abs(delta_x[ii])
	    if testabs>1:
                delta_x[ii]=1.0*delta_x[ii]/testabs
	
        delta_x_new=numpy.dot(mid_RR,delta_x)

	v_ee=numpy.float64(1.0)*delta_x_new
	J = self.get_jacobian(b_T_ee, joint_transforms)
	J_s=numpy.linalg.pinv(J,1.0e-2)
	
	

	v_sol=numpy.dot(J_s,v_ee)
	#print(v_sol)
	J_pinv=numpy.linalg.pinv(J)
	#print(J_pinv)
	v_sol_f=v_sol
        
	if command.secondary_objective==True :
	    value_q_sec=3*(command.q0_target-self.q_current[0])
            q_sec=numpy.zeros(len(J[0,:]))
	    q_sec[0]=value_q_sec
	    II=numpy.eye(len(J[0,:]))
	    #print(len(J[0,:]))
            q_null=(II-numpy.dot(J_pinv,J))
            q_null=numpy.dot(q_null,q_sec)
            v_sol=v_sol+q_null
	    

	v_sol_f=v_sol
	'''
	vmax=-100
	rmax=-100
	for kk in range(3):
            if vmax<abs(v_sol_f[kk]):
 	        vmax=abs(v_sol_f[kk])  
 	for kk in range(3,6):
            if rmax<abs(v_sol_f[kk]):
 	        rmax=abs(v_sol_f[kk])
	
	for ii in range(3):
	    if v_sol_f[ii]>0.1:
                for jj in range(3):
                    v_sol_f[jj]=0.1*v_sol_f[jj]/vmax
            if v_sol_f[ii]<-0.1:
	        for jj in range(3):
                    v_sol_f[jj]=-0.1*v_sol_f[jj]/vmax
       
	for ii in range(3,6):    
            if v_sol_f[ii]>1:
                for jj in range(3,6):
                   v_sol_f[jj]=1*v_sol_f[jj]/rmax
	    if v_sol_f[ii]<-1:
	        for jj in range(3,6):
                   v_sol_f[jj]=-1*v_sol_f[jj]/rmax
        #print(vmax)
	#print(rmax)     
	#print(v_sol_f)
        '''

	for ii in range(3,6):    
            testabs=numpy.abs(v_sol_f[ii])
	    if testabs>1:
                v_sol_f[ii]=1*v_sol_f[ii]/testabs

	msg.velocity=v_sol_f
	#print(v_sol_f)
	#print(msg.velocity)
	#q_current=q_current+v_sol_f
	#msg.position=q_current
	self.velocity_pub.publish(msg)
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This is a function which will assemble the jacobian of the robot using the
       current joint transforms and the transform from the base to the end
       effector (b_T_ee). Both the cartesian control callback and the
       inverse kinematics callback will make use of this function.
       Usage: J = self.get_jacobian(b_T_ee, joint_transforms)'''
    def get_jacobian(self, b_T_ee, joint_transforms):
        J = numpy.zeros((6,self.num_joints))
	#print(J)
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR ASSEMBLING THE CURRENT JACOBIAN HERE
	#jacobian
        #for j in range(len(joint_transforms)):
	#b_T_j=joint_transforms[j]
	ee_T_b=numpy.linalg.inv(b_T_ee)
        for j in range(self.num_joints):
            #j_T_ee=inverse(b_T_j)*b_T_ee
	    #print(numpy.dot(numpy.linalg.inv(joint_transforms[j]),b_T_ee))	
            #j_T_ee=numpy.dot(numpy.linalg.inv(joint_transforms[j]),b_T_ee)
            #ee_T_j=inverse(j_T_ee)
            #ee_T_j=numpy.linalg.inv(j_T_ee)
	    ee_T_j=numpy.dot(ee_T_b,joint_transforms[j])
	    j_T_ee=numpy.linalg.inv(ee_T_j)
	    #print(ee_T_j)
	    #print(ee_T_j1)
	    #ee_R_j=ee_T_j[0:2,0:2]
            ee_R_j=ee_T_j[0:3,0:3]
            j_t_ee=j_T_ee[0:3,3]
	    x=j_t_ee[0]
	    y=j_t_ee[1]
	    z=j_t_ee[2]
            S_t=[[0,-z,y],[z,0,-x],[-y,x,0]]
	    mid_V=[]
	    mid_V=-numpy.dot(ee_R_j,S_t)
            
            V_new=[]
	    V_new=numpy.vstack((mid_V,ee_R_j))
	    #print(V_new)
	  
	    kk=1
	    if self.joint_axes[j][0]==1:
		k=3
            if self.joint_axes[j][0]==-1:
		k=3
		kk=-1
  	    if self.joint_axes[j][1]==1:
		k=4
	    if self.joint_axes[j][1]==-1:
		k=4
		kk=-1
            if self.joint_axes[j][2]==1:
		k=5
	    if self.joint_axes[j][2]==-1:
		k=5
		kk=-1
	    
	    J[:,j]=kk*V_new[:,k-3]  
	    	
	
        #--------------------------------------------------------------------------
        return J

    '''This is the callback which will be executed when the inverse kinematics
       recieve a new command. The command will contain information about desired
       end effector pose relative to the root of your robot. At the end of this
       callback, you should publish to the /joint_command topic. This should not
       search for a solution indefinitely - there should be a time limit. When
       searching for two matrices which are the same, we expect numerical
       precision of 10e-3.'''
    def get_ik_command(self, command):
        self.mutex.acquire()
        #--------------------------------------------------------------------------
        #FILL IN YOUR PART OF THE CODE FOR INVERSE KINEMATICS HERE
	#ik_command /joint_command
	msg=self.joint_command_msg
	msg.name=self.joint_names
	
	q_current=self.q_current
	msg.position=q_current
	mid_q=[command.rotation.x,command.rotation.y,command.rotation.z,command.rotation.w]
	R_desired=tf.transformations.quaternion_matrix(mid_q)
        success=False
 	for i in range(3):
 	    for ii in range(len(self.q_current)):	
 		q_current[ii]=random.uniform(0,6.28)
	    #print(q_current)
	    t_c=time.time()
 	    while (time.time()-t_c)<10:
		
		R_desired=R_desired[0:3,0:3]
		joint_transforms, b_T_ee=self.forward_kinematics(q_current)
		delta_x=[command.translation.x,command.translation.y,command.translation.z]-b_T_ee[0:3,3]
        	R_current=b_T_ee[0:3,0:3]
		q_rotation = tf.transformations.quaternion_from_matrix(b_T_ee)
              
 	            
		R_mid=numpy.dot(R_desired,numpy.linalg.inv(R_current))
		c_angle, c_axis=self.rotation_from_matrix(R_mid)
	        delta_angle=c_angle*c_axis
	        delta_x=numpy.hstack((delta_x,delta_angle))
		e_R_b=numpy.linalg.inv(R_current)
		data0=numpy.zeros((3,3))
        	mid_RR1=numpy.hstack((e_R_b,data0))
		mid_RR2=numpy.hstack((data0,e_R_b))
        	mid_RR=numpy.vstack((mid_RR1,mid_RR2))
        	delta_x_new=numpy.dot(mid_RR,delta_x)
        	J = self.get_jacobian(b_T_ee, joint_transforms)
		J_pinv=numpy.linalg.pinv(J)
		delta_q=numpy.dot(J_pinv,delta_x_new)
		
		
		q_current=q_current+delta_q
 		msg=JointState()
	        msg.name=self.joint_names
		msg.position=q_current
	        #self.joint_command_pub.publish(msg)
		#print(q_current)
		delta_xx=[command.translation.x,command.translation.y,command.translation.z]-b_T_ee[0:3,3]	
                
		IK=2
                for jj in range(3):
		    if abs(delta_xx[i])<1e-3:
			IK=IK*2
                if IK==16:
		    for kk in range(4):
		        if abs(q_rotation[kk]-mid_q[kk])<1e-3:
		            IK=IK/2		
		if IK==1:
		
		    self.joint_command_pub.publish(msg)
		    success=True
		    break
	        	    
	    if success:
	        break
	    print('end')
                
                
        #--------------------------------------------------------------------------
        self.mutex.release()

    '''This function will return the angle-axis representation of the rotation
       contained in the input matrix. Use like this: 
       angle, axis = rotation_from_matrix(R)'''
    def rotation_from_matrix(self, matrix):
        R = numpy.array(matrix, dtype=numpy.float64, copy=False)
        R33 = R[:3, :3]
        # axis: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, W = numpy.linalg.eig(R33.T)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        axis = numpy.real(W[:, i[-1]]).squeeze()
        # point: unit eigenvector of R33 corresponding to eigenvalue of 1
        l, Q = numpy.linalg.eig(R)
        i = numpy.where(abs(numpy.real(l) - 1.0) < 1e-8)[0]
        if not len(i):
            raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
        # rotation angle depending on axis
        cosa = (numpy.trace(R33) - 1.0) / 2.0
        if abs(axis[2]) > 1e-8:
            sina = (R[1, 0] + (cosa-1.0)*axis[0]*axis[1]) / axis[2]
        elif abs(axis[1]) > 1e-8:
            sina = (R[0, 2] + (cosa-1.0)*axis[0]*axis[2]) / axis[1]
        else:
            sina = (R[2, 1] + (cosa-1.0)*axis[1]*axis[2]) / axis[0]
        angle = math.atan2(sina, cosa)
        return angle, axis

    '''This is the function which will perform forward kinematics for your 
       cartesian control and inverse kinematics functions. It takes as input
       joint values for the robot and will return an array of 4x4 transforms
       from the base to each joint of the robot, as well as the transform from
       the base to the end effector.
       Usage: joint_transforms, b_T_ee = self.forward_kinematics(joint_values)'''
    def forward_kinematics(self, joint_values):
        joint_transforms = []

        link = self.robot.get_root()
        T = tf.transformations.identity_matrix()

        while True:
            if link not in self.robot.child_map:
                break

            (joint_name, next_link) = self.robot.child_map[link][0]
            joint = self.robot.joint_map[joint_name]

            T_l = numpy.dot(tf.transformations.translation_matrix(joint.origin.xyz), tf.transformations.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2]))
            T = numpy.dot(T, T_l)

            if joint.type != "fixed":
                joint_transforms.append(T)
                q_index = self.joint_names.index(joint_name)
                T_j = tf.transformations.rotation_matrix(joint_values[q_index], numpy.asarray(joint.axis))
                T = numpy.dot(T, T_j)

            link = next_link
        return joint_transforms, T #where T = b_T_ee

    '''This is the callback which will recieve and store the current robot
       joint states.'''
    def get_joint_state(self, msg):
        self.mutex.acquire()
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])
        self.mutex.release()


if __name__ == '__main__':
    rospy.init_node('cartesian_control_and_IK', anonymous=True)
    CCIK()
    rospy.spin()
