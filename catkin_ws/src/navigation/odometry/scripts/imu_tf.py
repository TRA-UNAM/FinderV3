#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
import tf

def imu_callback(data):
	global imu_x, imu_y, imu_z, imu_w
	imu_x = data.orientation.x
	imu_y = data.orientation.y
	imu_z = data.orientation.z
	imu_w = data.orientation.w

def main():
	rospy.init_node('imu_tf_node', anonymous=True)
	rospy.Subscriber("/imu/data", Imu, imu_callback)	#the value in /cmd_vel gows from -0.5 to 0.5 (m/S)
	rate = rospy.Rate(10)
	br = tf.TransformBroadcaster()
	tfs_msg = TransformStamped
	global imu_x, imu_y, imu_z, imu_w
	imu_x = 0
	imu_y = 0
	imu_z = 0
	imu_w = 0

	while not rospy.is_shutdown():
		br.sendTransform((0, 0, 0), (imu_x, imu_y, imu_z, imu_w),rospy.Time.now(),"imu_link","base_link")
		rate.sleep()

if __name__=='__main__':
	main()

