#!/usr/bin/env python

##########################################
##### WRITE YOUR CODE IN THIS FILE #######
##########################################

import numpy 
import rospy
import csv
from sklearn import svm
from sklearn.linear_model import LinearRegression
from sklearn.decomposition import PCA
from hand_analysis.msg import GraspInfo

class hand_analysis(object):
    def __init__(self):
    	self.sub=rospy.Subscriber(name = "/grasp_info", data_class = GraspInfo, callback = self.callback, queue_size = 100)
    	self.pub=rospy.Publisher("/labeled_grasp_info", GraspInfo, queue_size=100)
	
        file = rospy.get_param('~train_filename')
        data = numpy.genfromtxt(fname=file, delimiter = ',', skip_header=1)
	
	clf = svm.SVC(gamma='scale', decision_function_shape='ovo')
	self.clf_g_l=clf.fit(data[:,9:24],data[:,0])
	
	clf2= svm.SVC(C=213,gamma='scale', decision_function_shape='ovo')
	self.clf_EMG_l=clf2.fit(data[:,1:9],data[:,0])

	clf3=LinearRegression().fit(data[:,1:9],data[:,9:24])
	self.clf_EMG_g=clf3
	#clf3.score(data[:,1:9],data[:,9:24])
	#clf3.predict(data[:,1:9])

	clf4=PCA(n_components=2)
	self.clf_h_l=clf4.fit(data[:,9:24])
	print('train finished')


    def callback(self, msg):
        if (msg.label<=0):
	    if (len(msg.glove)>0):
		msg.label=numpy.array(self.clf_g_l.predict(numpy.array(msg.glove).reshape(1,-1)))[0] #glove-label
	    else:
		if (len(msg.emg)<>0):
		    msg.label=numpy.array(self.clf_EMG_l.predict(numpy.array(msg.emg).reshape(1,-1)))[0] #EMG-label

	if (len(msg.glove)==0):
	    if (len(msg.emg)<>0):
		msg.glove=numpy.array(self.clf_EMG_g.predict(numpy.array(msg.emg).reshape(1,-1)))[0] #EMG-glove
	else:
	    if (len(msg.glove_low_dim)==0):
	        msg.glove_low_dim=numpy.array(self.clf_h_l.transform(numpy.array(msg.glove).reshape(1,-1)))[0]
	
	if (len(msg.glove_low_dim)<>0):
	        msg.glove=numpy.array(self.clf_h_l.inverse_transform(numpy.array(msg.glove_low_dim).reshape(1,-1)))[0]

        self.pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('hand_analysis', anonymous=True)
    hand_analysis()
    rospy.spin()
    
    	
    
#reader=csv.reader(open("data/object_grasping_30sec.csv","rb"),delimiter=",")
#x=list(reader)

#joint_angles0-14
#emg0-7
#clas_gt



"""
glove-label SVM
EMG-label SVM
EMG-glove regression
high-low PCA
low-high PCA

from sklearn import svm
clf=svm.SVC(gamma='scale')
slf.fit(glove,label)
clf.predict(glove)

from sklearn.linear_model import LinearRegression
reg=LinearRegression().fit(EMG,glove)
reg.score(EMG,glove)
reg.predict(EMG)

from sklearn.decomposition import PCA
pca=PCA(n_components=2)
pca.fit(high)


if (the object label is missing)
	if (glove data present)
		glove-label
	else 
		EMG-label

if (the glove data is missing)
	EMG-glove
else 
	if (low_dimensional glove data is missing)
	    projecting
	if (only low)
	    inverse projection
	    msg.glove


print(x)

rospy.Subscriber(name = "/grasp_info", data_class = GraspInfo, callback = self.callback, queue_size = 100)

rospy.Publisher("/labeled_grasp_info", GraspInfo, queue_size=100)
"""
