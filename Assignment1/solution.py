#!/usr/bin/env python  
import rospy

import numpy
from numpy import *
import math

import tf
import tf2_ros
import geometry_msgs.msg

def angleof(v1, v2):
    v1_u = v1 / numpy.linalg.norm(v1)
    v2_u = v2 / numpy.linalg.norm(v2)
    return numpy.arccos(numpy.clip(numpy.dot(v1_u, v2_u), -1.0, 1.0))

def publish_transforms():
    
    global angle
    global x_axis
    global m3_camera
    global m1_robot

    t1 = geometry_msgs.msg.TransformStamped()
    t1.header.stamp = rospy.Time.now()
    t1.header.frame_id = "base_frame"
    t1.child_frame_id = "object_frame"
    transform1=tf.transformations.euler_matrix(0.64,0.64,0.0)
    m1 = array([1.5,0.8,0.0,1])
    m1 = m1.reshape(-1,1)
    m1 = numpy.matmul(transform1,m1)
    t1.transform.translation.x = m1[0]
    t1.transform.translation.y = m1[1]
    t1.transform.translation.z = m1[2]
    q1 = tf.transformations.quaternion_from_euler(0.64, 0.64, 0.0)
    t1.transform.rotation.x = q1[0]
    t1.transform.rotation.y = q1[1]
    t1.transform.rotation.z = q1[2]
    t1.transform.rotation.w = q1[3]
   
    #print(t1.transform)
   # t1.transform.translation.x = 1.5
   # t1.transform.translation.y = 0.8
   # t1.transform.translation.z = 0.0
   
    br.sendTransform(t1)

    t2 = geometry_msgs.msg.TransformStamped()
    t2.header.stamp = rospy.Time.now()
    t2.header.frame_id = "base_frame"
    t2.child_frame_id = "robot_frame"
    transmid=tf.transformations.euler_matrix(0.0,1.5,0.0)
    m2 = array([0.0,0.0,-2.0,1])
    m2 = m2.reshape(-1,1)
    m2 = numpy.matmul(transmid,m2)
    t2.transform.translation.x = m2[0]
    t2.transform.translation.y = m2[1]
    t2.transform.translation.z = m2[2]
    q2 = tf.transformations.quaternion_about_axis(1.5, (0,1,0))
    t2.transform.rotation.x = q2[0]
    t2.transform.rotation.y = q2[1]
    t2.transform.rotation.z = q2[2]
    t2.transform.rotation.w = q2[3]    

    #transeular=tf.transformations.euler_from_quaternion(q2)
    #transmid=tf.transformations.euler_matrix(0.0,1.5,0.0)

    #print(t2)

    #t2.transform.translation.x = 0.0
    #t2.transform.translation.y = 0.0
    #t2.transform.translation.z = -2.0*transmid
   
    br.sendTransform(t2)

    T1 = numpy.dot(tf.transformations.quaternion_matrix(q1),
                   tf.transformations.translation_matrix((1.5, 0.8, 0.0))
                   )

    T1_inverse = tf.transformations.inverse_matrix(T1)

    T2 = numpy.dot(tf.transformations.quaternion_matrix(q2),
                   tf.transformations.translation_matrix((0.0, 0.0, -2.0)))

    T2_inverse = tf.transformations.inverse_matrix(T2)

    qq=tf.transformations.quaternion_from_euler(0,0,0)
    T3 = numpy.dot(tf.transformations.quaternion_matrix(qq),
                   tf.transformations.translation_matrix((0.3, 0.0, 0.3)))

    T3_inverse = tf.transformations.inverse_matrix(T3)

    t3 = geometry_msgs.msg.TransformStamped()
    t3.header.stamp = rospy.Time.now()
    t3.header.frame_id = "robot_frame"
    t3.child_frame_id = "camera_frame"
    t3.transform.translation.x = 0.3
    t3.transform.translation.y = 0.0
    t3.transform.translation.z = 0.3
    m3 = array([0.3,0.0,0.3,1])
    m3 = m3.reshape(-1,1)

#p2 is m1

    m1_robot=numpy.matmul(T2_inverse,m1)
    
    m3_camera=numpy.matmul(T3_inverse,m1_robot)
    
    m3_camera = m3_camera[:(len(m3_camera)-1)]
    #print(m3_camera)
    #print(m3_camera)
 
    #mc=numpy.matmul(T2_inverse,m3)
    #xc=mc[0]+m2[0]
    #yc=mc[1]+m2[1]
    #zc=mc[2]+m2[2]
    #print(xc,yc,zc)
    #k=numpy.sqrt(numpy.sum(numpy.square(mc+m2-m1)))
    #print(k)
   # mfinal=mc+m2-m1
    #mlate = array([xc[0],yc[0],zc[0],1])
   # mlate = mlate.reshape(-1,1)
    #mlast=numpy.matmul(T2,mlate)
   # print(mfinal)
    #print(mlast)



    x_axis = [1,0,0]
    mid=map(list, zip(*m3_camera))[0]


    #print(mid)

   #[0.3165921,0.64167661,3.14846999]
   
   # for i in range(3): 
   #   mid[i]=m3_camera[i]
   # print(mid)
    angle = angleof(x_axis, mid)
    v_normal = numpy.cross(x_axis, mid)
   
   # theta1=math.atan2(m3_camera[2],m3_camera[0])
    #theta2=math.atan2(numpy.sqrt(numpy.square(m3_camera[2])+numpy.square(m3_camera[0])),m3_camera[1])

    q3 = tf.transformations.quaternion_about_axis(angle,v_normal)


    t3.transform.rotation.x = q3[0]
    t3.transform.rotation.y = q3[1]
    t3.transform.rotation.z = q3[2]
    t3.transform.rotation.w = q3[3]
    br.sendTransform(t3)

    #T3 = numpy.dot(tf.transformations.translation_matrix((0.3, 0.0, 0.3)),
       #            tf.transformations.quaternion_matrix(q3))

    #print(T3)

    

    # t4 = geometry_msgs.msg.TransformStamped()
    # t4.header.stamp = rospy.Time.now()
    # t4.header.frame_id = "camera_frame"
    # t4.child_frame_id = "object_frame"
    # tr4 = tf.transformations.translation_from_matrix(T2_inverse)
    # t4.transform.translation.x = tr4[0]
    # t4.transform.translation.y = tr4[1]
    # t4.transform.translation.z = tr4[2]
    # q4 = tf.transformations.quaternion_from_matrix(T2_inverse)
    # t4.transform.rotation.x = q4[0]
    # t4.transform.rotation.y = q4[1]
    # t4.transform.rotation.z = q4[2]
    # t4.transform.rotation.w = q4[3]
    # br.sendTransform(t4)

if __name__ == '__main__':
    rospy.init_node('tf2_examples')

    br = tf2_ros.TransformBroadcaster()
    #rospy.sleep(0.5)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        publish_transforms()
       #rospy.sleep(0.5)
        r.sleep()  
