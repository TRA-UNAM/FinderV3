#!/usr/bin/env python

import rospy
import tf
import math
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

def lwheel_callback(msg):
	#global robot_x, robot_y, robot_t
	#robot_x += 0.05*data.linear.x*math.cos(robot_t)
	#robot_y += 0.05*data.linear.x*math.sin(robot_t)
	#robot_t += 0.05*data.angular.z
	global lspeed, last_lwheel
	lspeed = msg.data - last_lwheel
	if lspeed > 512:
		lspeed -= 1024
	if lspeed < -512:
		lspeed += 1024
	last_lwheel = msg.data

def rwheel_callback(msg):
	global rspeed, last_rwheel
	rspeed = msg.data - last_rwheel
	if rspeed > 512:
		rspeed -= 1024
	if rspeed < -512:
		rspeed += 1024
	last_rwheel = msg.data

def main():
	rospy.init_node('odom_node', anonymous=True)
	rospy.Subscriber("/lwheel", Int16, lwheel_callback)
	rospy.Subscriber("rwheel", Int16, rwheel_callback)
	rate = rospy.Rate(10)
	#br = tf.TransformBroadcaster()
	#global robot_x, robot_y, robot_t
	#robot_x = 0
	#robot_y = 0
	#robot_t = 0
	global lspeed, last_lwheel
	lspeed = 0
	last_lwheel = 0
	global rspeed, last_rwheel
	rspeed = 0
	last_rwheel = 0

	while not rospy.is_shutdown():
		#br.sendTransform((robot_x, robot_y, 0), tf.transformations.quaternion_from_euler(0, 0, robot_t),rospy.Time.now(),"base_footprint","odom")
		print lspeed
		print rspeed
		rate.sleep()

if __name__=='__main__':
	main()
