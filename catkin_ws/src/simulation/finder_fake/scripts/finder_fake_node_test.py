#! /usr/bin/env python

import rospy
import tf
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import JointState

def cmd_vel_callback(data):
    global robot_x, robot_y, robot_t    
    robot_x += 0.05*data.linear.x*math.cos(robot_t)
    robot_y += 0.05*data.linear.x*math.sin(robot_t)
    robot_t += 0.05*data.angular.z

def flipper1_out_callback(data):
    global flipper1_out

    flipper1_out=data.data

def flipper2_out_callback(data):
    global flipper2_out
    flipper2_out=data.data

def flipper3_out_callback(data):
    global flipper3_out
    flipper3_out=data.data

def flipper4_out_callback(data):
    global flipper4_out
    flipper4_out=data.data

def base_rotation_out_callback(data):
    global base_rotation_out
    base_rotation_out=data.data

def shoulder_rotation_out_callback(data):
    global shoulder_rotation_out
    shoulder_rotation_out=data.data

def elbow_rotation_out_callback(data):
    global elbow_rotation_out
    elbow_rotation_out=data.data

def roll_rotation_out_callback(data):
    global roll_rotation_out
    roll_rotation_out=data.data

def pitch_rotation_out_callback(data):
    global pitch_rotation_out
    pitch_rotation_out=data.data

def roll_rotation_2_out_callback(data):
    global roll_rotation_2_out
    roll_rotation_2_out=data.data

def gripper_rotation_out_callback(data):
    global gripper_rotation_out
    gripper_rotation_out=data.data


def main():
    rospy.init_node('fake_node_test', anonymous=True)
    pub=rospy.Publisher('/joint_states', JointState, queue_size=10)
    global robot_x, robot_y, robot_t, flipper1_out, flipper2_out, flipper3_out, flipper4_out, base_rotation_out, shoulder_rotation_out, elbow_rotation_out, roll_rotation_out, pitch_rotation_out, roll_rotation_2_out, gripper_rotation_out
    robot_x = 0
    robot_y = 0
    robot_t = 0
    flipper1_out=0.0
    flipper2_out=0.0
    flipper3_out=0.0
    flipper4_out=0.0
    base_rotation_out=0.0
    shoulder_rotation_out=0.0
    elbow_rotation_out=0.0
    roll_rotation_out=0.0
    pitch_rotation_out=0.0
    roll_rotation_2_out=1.6
    gripper_rotation_out=1.6
    seq=0
    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_callback)   #the value in /cmd_vel gows from -0.5 to 0.5 (m/S)
    rospy.Subscriber('/flipper1_out',Float64, flipper1_out_callback)
    rospy.Subscriber('/flipper2_out',Float64, flipper2_out_callback)
    rospy.Subscriber('/flipper3_out',Float64, flipper3_out_callback)
    rospy.Subscriber('/flipper4_out',Float64, flipper4_out_callback)
    rospy.Subscriber('/base_rotation_out',Float32, base_rotation_out_callback)
    rospy.Subscriber('/shoulder_rotation_out',Float32, shoulder_rotation_out_callback)
    rospy.Subscriber('/elbow_rotation_out',Float32, elbow_rotation_out_callback)
    rospy.Subscriber('/roll_rotation_out',Float32, roll_rotation_out_callback)
    rospy.Subscriber('/pitch_rotation_out',Float32, pitch_rotation_out_callback)
    rospy.Subscriber('/roll_rotation_2_out',Float32, roll_rotation_2_out_callback)
    rospy.Subscriber('/gripper_rotation_out',Float32, gripper_rotation_out_callback)
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    joint_state=JointState()
    
    
    
    while not rospy.is_shutdown():
        br.sendTransform((robot_x, robot_y, 0), tf.transformations.quaternion_from_euler(0, 0, robot_t),rospy.Time.now(),"base_link","odom")
        joint_state.header.stamp=rospy.get_rostime()
        joint_state.header.frame_id=''
        joint_state.name=["right_front_flipper","left_front_flipper","right_back_flipper","left_back_flipper","base_rotation","shoulder_rotation","elbow_rotation","roll_rotation","pitch_rotation","roll_rotation_2","gripper_rotation"]
        joint_state.position=[flipper1_out,flipper2_out,flipper3_out,flipper4_out,base_rotation_out,shoulder_rotation_out,elbow_rotation_out,roll_rotation_out,pitch_rotation_out,roll_rotation_2_out,gripper_rotation_out]
        joint_state.velocity=[]
        joint_state.effort=[]
        joint_state.header.seq=seq+1
        pub.publish(joint_state)
        rate.sleep()

if __name__=='__main__':
    main()
