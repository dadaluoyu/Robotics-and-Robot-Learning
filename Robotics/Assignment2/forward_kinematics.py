#!/usr/bin/env python

import numpy
import geometry_msgs.msg
import rospy
from sensor_msgs.msg import JointState
import tf
import tf2_ros
import tf.msg
from urdf_parser_py.urdf import URDF

"""This function will transform a 4x4 transformation matrix T into a ros message 
which can be published. In addition to the transform itself, the message
also specifies who is related by this transform, the parent and the child.
It is an optional function which you may use as you see fit."""
def convert_to_message(T, child, parent):
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child
    translation = tf.transformations.translation_from_matrix(T)
    rotation = tf.transformations.quaternion_from_matrix(T)
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]        
    return t
    
#Our main class for computing Forward Kinematics
class ForwardKinematics(object):

    #Initialization
    def __init__(self):
        """Announces that it will publish forward kinematics results, in the form of tfMessages.
        "tf" stands for "transform library", it's ROS's way of communicating around information
        about where things are in the world"""
        self.pub_tf = rospy.Publisher("/tf", tf.msg.tfMessage, queue_size=1)

        #Loads the robot model, which contains the robot's kinematics information
        self.robot = URDF.from_parameter_server()

        #Subscribes to information about what the current joint values are.
        rospy.Subscriber("joint_states", JointState, self.callback)


    """This function is called every time the robot publishes its joint values. You must use
    the information you get to compute forward kinematics.

    The callback you write should iterate through the entire robot chain, and publish 
    the transform for each link you find.
    """
    def callback(self, joint_values):
	# YOUR CODE GOES HERE

        #actually it's the same

        #add a judgement of the last link
        #The last link of the robot will have no child, and therefore will not appear in the robot's child_map.

        #shape
        link = self.robot.get_root()
        #print(link)
        #print(joint_values)

        #link-joint-link
        (base_joint_name, next_link) = self.robot.child_map[link][0]
        #print(next_joint_name)

        #next_joint value
        base_joint = self.robot.joint_map[base_joint_name]
        #print(next_joint)
        xyz=base_joint.origin.xyz
        rpy=base_joint.origin.rpy
        T_base=numpy.dot(tf.transformations.translation_matrix(xyz),
                   tf.transformations.euler_matrix(rpy[0],rpy[1],rpy[2]))
        t1=convert_to_message(T_base, next_link, link)
        self.pub_tf.publish([t1])
        
        now_link=next_link
       

        for i in range(len(joint_values.name)):
            
            #print(len(joint_values.name))
           
            
            (next_joint_name, next_link) = self.robot.child_map[now_link][0]
            
            next_joint = self.robot.joint_map[next_joint_name]
            xyz=next_joint.origin.xyz
            rpy=next_joint.origin.rpy
            #print(tf.transformations.quaternion_matrix(rpy))
            axis=next_joint.axis
            #print(axis)
            now_position=joint_values.position[joint_values.name.index(next_joint_name)]

            q = tf.transformations.quaternion_about_axis(now_position,axis)
            #print(q)
            T_position=tf.transformations.quaternion_matrix(q)

            
            T1 = numpy.dot(tf.transformations.translation_matrix(xyz),
                           tf.transformations.euler_matrix(rpy[0],rpy[1],rpy[2]))

            #print(T1)
            
           
            if i==0 : 
                T_ac=numpy.eye(4)

            T_now=numpy.dot(T1,T_position)
             
            T_ac=numpy.dot(T_ac,T_now)
            
            T_next=numpy.dot(T_base,T_ac)
              
            t1=convert_to_message(T_next, next_link, link)    
            #print(t1)
            now_link=next_link

            #aList.index( 'joint_value' )
            #use index to find the right joint_name then apply the index to find the right joint value
            #position[name.index(next_joint_name)]

  
            #print(1)
            self.pub_tf.publish([t1]) 
            #print(2)

        try:
            self.robot.child_map[next_link][0]
        except: #NameError
            var_exists = False
        else:
            var_exists = True

        if  var_exists:
            (end_joint_name, end_link) = self.robot.child_map[next_link][0]
            end_joint = self.robot.joint_map[end_joint_name]
            xyz=end_joint.origin.xyz
            rpy=end_joint.origin.rpy
            T_end=numpy.dot(tf.transformations.translation_matrix(xyz),
                   tf.transformations.euler_matrix(rpy[0],rpy[1],rpy[2]))
            T_end=numpy.dot(T_next,T_end)
            t1=convert_to_message(T_end, end_link, link)
            self.pub_tf.publish([t1])
            
        
       
if __name__ == '__main__':
    rospy.init_node('fwk', anonymous=True)
    fwk = ForwardKinematics()
    rospy.spin()

