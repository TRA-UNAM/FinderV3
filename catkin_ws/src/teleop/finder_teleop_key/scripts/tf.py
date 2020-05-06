#!/usr/bin/env python

import rospy
import tf2_ros
import tf
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Int16
import numpy as np


def callback(data):
	vr=data.data
	VR=vr*(0.0625)#Se multiplica por el radio promedio de las ruedas del FINDER V3
	

def callback2(data):
	vl=data.data
	VL=vl*(0.0625)
	
	


def funcion():
	global x
	global thetak
	global VR, VL
	VR = 0
	VL = 0
	thetak=0
	x=0
	rospy.init_node('tf_node', anonymous=True)
	rospy.Subscriber("/left_wheel_speed", Int16, callback)
	rospy.Subscriber("/right_wheel_speed", Int16,callback2)
	rospy.spin()
	rate=rospy.Rate(10)
	br=tf2_ros.TransformBroadcaster()
	
	while not rospy.is_shutdown():
		linear_velocity=((VR+VL)/(2))
		angular_velocity=((VR-VL)/(0.40))#Divido entre la distancia entre ruedas u orugas
		s=linear_velocity*0.1#Multiplico por el tiempo
		theta=angular_velocity*0.1#Multiplico por el tiempo (Se supone el robot se mueve muy poco)
		x=x+(s*(np.cos(thetak+(theta/2))))
		thetak=thetak+theta
		br.sendTransform((x, 0, 0),tf.transformations.quaternion_from_euler(0,0,thetak),rospy.Time.now(),"base_link","odom")
		rate.sleep()

	

if __name__=='__main__':
	try:
	   	funcion()
	except rospy.ROSInterruptException:
		pass
	