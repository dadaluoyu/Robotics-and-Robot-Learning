#!/usr/bin/env python

import numpy
import random
import sys

import geometry_msgs.msg
import moveit_msgs.msg
import moveit_msgs.srv
import rospy
import tf
import moveit_commander
from urdf_parser_py.urdf import URDF
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Transform
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time
import copy

def convert_to_message(T):
    t = geometry_msgs.msg.Pose()
    position = tf.transformations.translation_from_matrix(T)
    orientation = tf.transformations.quaternion_from_matrix(T)
    t.position.x = position[0]
    t.position.y = position[1]
    t.position.z = position[2]
    t.orientation.x = orientation[0]
    t.orientation.y = orientation[1]
    t.orientation.z = orientation[2]
    t.orientation.w = orientation[3]        
    return t

class MoveArm(object):

    def __init__(self):

        #Loads the robot model, which contains the robot's kinematics information
	self.num_joints = 0
        self.joint_names = []
        self.joint_axes = []
        self.robot = URDF.from_parameter_server()
        self.base = self.robot.get_root()
        self.get_joint_info()

        # Wait for moveit IK service
        rospy.wait_for_service("compute_ik")
        self.ik_service = rospy.ServiceProxy('compute_ik',  moveit_msgs.srv.GetPositionIK)
        print "IK service ready"

        # Wait for validity check service
        rospy.wait_for_service("check_state_validity")
        self.state_valid_service = rospy.ServiceProxy('check_state_validity',  
                                                      moveit_msgs.srv.GetStateValidity)
        print "State validity service ready"

        # MoveIt parameter
        robot_moveit = moveit_commander.RobotCommander()
        self.group_name = robot_moveit.get_group_names()[0]

	#Subscribe to topics
	rospy.Subscriber('/joint_states', JointState, self.get_joint_state)
	rospy.Subscriber('/motion_planning_goal', Transform, self.motion_planning)
        self.current_obstacle = "None"
        rospy.Subscriber('/obstacle', String, self.get_obstacle)

	#Set up publisher
	self.pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

    '''This callback provides you with the current joint positions of the robot 
     in member variable q_current.'''
    def get_joint_state(self, msg):
        self.q_current = []
        for name in self.joint_names:
            self.q_current.append(msg.position[msg.name.index(name)])

    '''This callback provides you with the name of the current obstacle which
    exists in the RVIZ environment. Options are "None", "Simple", "Hard",
    or "Super". '''
    def get_obstacle(self, msg):
        self.current_obstacle = msg.data

    '''This is the callback which will implement your RRT motion planning.
    You are welcome to add other functions to this class (i.e. an
    "is_segment_valid" function will likely come in handy multiple times 
    in the motion planning process and it will be easiest to make this a 
    seperate function and then call it from motion planning). You may also
    create trajectory shortcut and trajectory sample functions if you wish, 
    which will also be called from the motion planning function.'''    
    def motion_planning(self, ee_goal):
        print "Starting motion planning"
	########INSERT YOUR RRT MOTION PLANNING HERE##########
        msg=JointTrajectory()
	msg.joint_names=self.joint_names
	mid_ee_goal=[ee_goal.rotation.x,ee_goal.rotation.y,ee_goal.rotation.z,ee_goal.rotation.w]
	T_desired=tf.transformations.quaternion_matrix(mid_ee_goal)
	T_desired[0:3,3]=[ee_goal.translation.x,ee_goal.translation.y,ee_goal.translation.z]
	joint_goal=self.IK(T_desired)
	#print(joint_goal)
	t=convert_to_message(T_desired)
	q_current=self.q_current

	Node=[]
	Node_parent=[]
	
	Node.append(q_current)
	Node_parent.append(0)
	q_diff0=numpy.subtract(joint_goal,q_current)
	print(numpy.linalg.norm(q_diff0))
	valid,qnew=self.is_segment_valid(Node[-1],joint_goal)
	
	time0=time.time()
	#dmin=1000
	
	while time.time()-time0<200 and valid==False:
	    dmin=1000
	    #print(time.time()-time0)
	    qr=numpy.random.rand(len(q_current))
	    qr=-numpy.pi+2*numpy.pi*qr
	    
	    #print(qr)
	    for (ii,qi) in enumerate(Node):
		d=numpy.linalg.norm(qi-qr)
		#print(d)
		if d<dmin:
	            i_min=ii
		    dmin=d
	    qclose=Node[i_min]
	   
	    valid,qnew=self.is_segment_valid(qclose,qr)
	    print(qnew)
	    Node.append(qnew)
	    Node_parent.append(i_min) 
            valid,qnew=self.is_segment_valid(Node[-1],joint_goal)
	    #dmin=numpy.linalg.norm(q_current-qr)

	    if valid:
		break
	  
	

	#print(Node)

	if valid:
	    path=[]
	    path.append(Node[-1])
	    pid=Node_parent[-1]
	    while pid!=0:
		path.append(Node[pid])
		pid=Node_parent[pid]
	    path.append(Node[0])
	    #path.append(Node[pid])
	    ##print(path)
	    path.reverse()
	    ##print(path)
	    path.append(joint_goal)


	
	#resample
	start=q_current
	#resample(path)
	#path_resample=[q_current]
	path_resample=[]
	path_resample.append(path[0])
	for ii in range(len(path)-1):
	    valid,qnew=self.is_segment_valid(start,path[ii+1])	
	    if valid==False:
	        path_resample.append(path[ii])
	        start=path[ii]
	path_resample.append(path[-1])
	    #else:
	       # path_resample.append(path[ii+1])
	print(path_resample)
	
	path_new=[]
	p_final=path_new[:]	
	kk=0
	for ii in range(len(path_resample)-1):
	    tt=numpy.subtract(path_resample[ii+1],path_resample[ii])
	    q_seg=tt/20
	    for jj in range(21):
	        path_new.append(numpy.add(path_resample[ii],jj*q_seg))
		p_final.append(JointTrajectoryPoint())
	    	p_final[kk].positions=path_new[kk]
		kk=kk+1
		print(kk)
	print(kk)
	#print(p_final)
	##print(path_new)
	#print(joint_goal)

        
	
	'''
	#q_current=(q_current[0],q_current[1],q_current[2],q_current[3],q_current[4],q_current[5],q_current[6])
	path_test=[None,None]
	path_new_test=[None,None,None,None,None,None,None,None,None,None,None]	
	path_test[0]=q_current
	path_test[1]=joint_goal

	for ii in range(len(path_test[0])):
	    tt=numpy.subtract(path_test[1],path_test[0])
	    q_diff=tt
	    q_seg=q_diff/10
	
 	for ii in range(11):
	    path_new_test[ii]=numpy.add(path_test[0],ii*q_seg)
	#print(path_new)
	p=path_new_test[:]	
	for ii in range(11):
	    p[ii]=JointTrajectoryPoint()
	    p[ii].positions=path_new_test[ii]
	#print(p)ui
	'''

 	'''
	path_new=[]
	p_final=path_new[:]	
	for ii in range(len(path)):
            p_final.append(JointTrajectoryPoint())
	    p_final[ii].positions=path[ii]
	'''
	msg.points=p_final
	#msg.points=p

	#print(type(msg.points[0].positions))
	#print(self.is_state_valid(joint_goal))
	
	#print(self.is_segment_valid(q_current,joint_goal))
	
	#msg.points.positions[0]=q_current
	self.pub.publish(msg)

    def is_segment_valid(self,q1,q2):
	#print(q1)
	q_diff=numpy.subtract(q2,q1)
	if numpy.linalg.norm(q_diff)==0:
	    return True,q1
	dq_max=max(q_diff)
	delta_q0=0.5
	#print(dq_max)
	n=dq_max/delta_q0
	n=math.ceil(n)
	#print(dq_max,delta_q,n)
	###delta_q=dq_max/n
	delta_q0=dq_max/n
	delta_q=q_diff/n
	#print(delta_q)
	n=int(n)
	#print(n)
	if len(q1)==7:
	    q_return=[0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	else:
	    q_return=[0.0,0.0,0.0,0.0,0.0,0.0]
	#q_return=numpy.zeros(len(q1))
	#print(q_return)
	#print(q_return)
	#[0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	dgrow=False
	dq_grow=0.7
	qold=copy.deepcopy(q1)

	#print(qold)
	for ii in range(n-1):
	    #print(q1)
	    q1=q1+delta_q
	    qnew=copy.deepcopy(q1)
	    #print(numpy.linalg.norm(qold-qnew))
	    if numpy.linalg.norm(qold-qnew)>dq_grow:
		q1=q1-delta_q
		delta_q=0.7*delta_q
		delta_q0=0.7*delta_q0
	
	
	n_new=math.ceil(dq_max/delta_q0)
	delta_q=q_diff/n_new
	#print(111)	
	#print(delta_q)
	#print(222)
	#delta_q=dq_max/n_new
	n_new=int(n_new)
	q1=copy.deepcopy(qold)
	#print(q1)
	#print(4444444)
	#print(delta_q)
	valid=self.is_state_valid(q1)
	if valid:
	    for ii in range(n_new-1):
	        q1=q1+delta_q
		if dgrow==False and numpy.linalg.norm(qold-q1)>dq_grow:
		    q_return=q1-delta_q
	 	    dgrow=True
		#valid=self.is_state_valid(q1)
	        #print(q1)
	    	if (self.is_state_valid(q1)==False):
		    if dgrow==False:
			#print(1)
			#print(q1-delta_q)
			return False,q1-delta_q
		    else:
	  		#print(2)
			#print(q_return)
			return False,q_return
   	#print(3)
	#print(q_return)
	return True,q_return

    def trajectory_shortcut(self,trajectory):
    
	return trajectory

    def trajectory_resample(self,trajectory):
	return trajectory

        ######################################################

    """ This function will perform IK for a given transform T of the end-effector.
    It returns a list q[] of values, which are the result positions for the 
    joints of the robot arm, ordered from proximal to distal. If no IK solution 
    is found, it returns an empy list.
    """
    def IK(self, T_goal):
        req = moveit_msgs.srv.GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = moveit_msgs.msg.RobotState()
        req.ik_request.robot_state.joint_state.name = self.joint_names
        req.ik_request.robot_state.joint_state.position = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.ik_request.robot_state.joint_state.header.stamp = rospy.get_rostime()
        req.ik_request.avoid_collisions = True
        req.ik_request.pose_stamped = geometry_msgs.msg.PoseStamped()
        req.ik_request.pose_stamped.header.frame_id = self.base
        req.ik_request.pose_stamped.header.stamp = rospy.get_rostime()
        req.ik_request.pose_stamped.pose = convert_to_message(T_goal)
        req.ik_request.timeout = rospy.Duration(3.0)
        res = self.ik_service(req)
        q = []
        if res.error_code.val == res.error_code.SUCCESS:
            q = res.solution.joint_state.position
        return q

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


    """ This function checks if a set of joint angles q[] creates a valid state,
    or one that is free of collisions. The values in q[] are assumed to be values
    for the joints of the KUKA arm, ordered from proximal to distal. 
    """
    def is_state_valid(self, q):
        req = moveit_msgs.srv.GetStateValidityRequest()
        req.group_name = self.group_name
        req.robot_state = moveit_msgs.msg.RobotState()
        req.robot_state.joint_state.name = self.joint_names
        req.robot_state.joint_state.position = q
        req.robot_state.joint_state.velocity = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.effort = numpy.zeros(self.num_joints)
        req.robot_state.joint_state.header.stamp = rospy.get_rostime()
        res = self.state_valid_service(req)
        return res.valid


'''This is a class which you can use to keep track of your tree branches.
It is easiest to do this by appending instances of this class to a list 
(your 'tree'). The class has a parent field and a joint position field (q). 
You can initialize a new branch like this:
RRTBranch(parent, q)
Feel free to keep track of your branches in whatever way you want - this
is just one of many options available to you.'''
class RRTBranch(object):
    def __init__(self, parent, q):
	self.parent = parent
	self.q = q


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_arm', anonymous=True)
    ma = MoveArm()
    rospy.spin()

