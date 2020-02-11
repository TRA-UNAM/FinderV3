#! /usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Twist

def cmd_vel_callback(data):
	global robot_x, robot_y, robot_t	
	robot_x += 0.1*data.linear.x*math.cos(robot_t)
	robot_y += 0.1*data.linear.x*math.sin(robot_t)
	robot_t += 0.1*data.angular.z

def main():
	rospy.init_node('odom_node', anonymous=True)
	rospy.Subscriber("/base_controller/command", Twist, cmd_vel_callback)	#the value in /cmd_vel gows from -0.5 to 0.5 (m/S)
	rate = rospy.Rate(10)
	br = tf.TransformBroadcaster()
	global robot_x, robot_y, robot_t
	robot_x = 0
	robot_y = 0
	robot_t = 0

	while not rospy.is_shutdown():
		br.sendTransform((robot_x, robot_y, 0), tf.transformations.quaternion_from_euler(0, 0, robot_t),rospy.Time.now(),"base_link","odom")
		rate.sleep()

if __name__=='__main__':
	main()
