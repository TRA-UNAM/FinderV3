#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

FINDER_MAX_LIN_VEL = 0.2
FINDER_MAX_ANG_VEL = 0.5
FLIPPER_MAX_VALUE= 3.14


msg = """
Control Your FINDER and Your Flippers!
---------------------------

Moving around:   
    w
a   s   d
    x   

Right_front_flipper:   
    o 

    l 

Left_front_flipper:   
    r 

    t

Right_back_flipper:   
    i

    k

Left_back_flipper:   
    t 

    g

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity
o/i, o/l, i/k, t/g : increse/decrease angular position
space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""
"""class Juntas:
    def __init__(JointState,header,name,position,velocity,effort):
        JointState.header=header
        JointState.name=name
        JointState.position=position   
        JointState.velocity=velocity
        JointState.effort=effort
class Header:
    def __init__(header,seq,stamp,frame_id):
        header.seq=seq
        header.stamp=stamp
        header.frame_id=frame_id
"""
def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def angs(target_linear_vel, ):
    return "currently:\tvalue Flippers %s\t " % (target_linear_vel)

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def constrain1(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    
    vel = constrain(vel, -FINDER_MAX_LIN_VEL, FINDER_MAX_LIN_VEL)
    

    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -FINDER_MAX_ANG_VEL, FINDER_MAX_ANG_VEL)

    return vel

def checkAngularLimitFlipper(vel):
    vel = constrain1(vel, -FLIPPER_MAX_VALUE, FLIPPER_MAX_VALUE)

    return vel
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('finder_teleop_key')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub2=rospy.Publisher('/joint_states', JointState, queue_size=10)
    

    status=0
    status0=0
    status1=0
    status2=0
    status3=0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    control_angular_flipper0 = 0.0
    control_angular_flipper1 = 0.0
    control_angular_flipper2 = 0.0
    control_angular_flipper3 = 0.0
    LIN_VEL_STEP_SIZE=0.1
    ANG_VEL_STEP_SIZE=0.2
    target_angular_flipper0=0.0
    target_angular_flipper1 =0.0
    target_angular_flipper2=0.0
    target_angular_flipper3=0.0
    ANG_FLIPPER_STEP_SIZE=0.1
    global pos
    name=["right_front_flipper","left_front_flipper","right_back_flipper","left_back_flipper","base_rotation","shoulder_rotation","elbow_rotation","roll_rotation","pitch_rotation","roll_rotation_2","gripper_rotation"]
    pos=[0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
    vel=[]
    effort=[]
    seq=0    

    try:
        print(msg)
        while(1):

            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            #Flipper derecho frontal
            elif key == 'o' :
                target_angular_flipper0= checkAngularLimitFlipper(target_angular_flipper0 - ANG_FLIPPER_STEP_SIZE)
                status0 = status0 + 0.1
                print("Right_front_flipper "+angs(target_angular_flipper0))
                control_angular_flipper0 = makeSimpleProfile(control_angular_flipper0, target_angular_flipper0, (ANG_FLIPPER_STEP_SIZE))
                pos[0]=control_angular_flipper0
                
            elif key == 'l' :
                target_angular_flipper0= checkAngularLimitFlipper(target_angular_flipper0 + ANG_FLIPPER_STEP_SIZE)
                status0 = status0 + 0.1
                print("Right_front_flipper "+angs(target_angular_flipper0))
                control_angular_flipper0 = makeSimpleProfile(control_angular_flipper0, target_angular_flipper0, (ANG_FLIPPER_STEP_SIZE))
                pos[0]=control_angular_flipper0

            #Flipper izquierdo frontal
            elif key == 'r' :
                target_angular_flipper1= checkAngularLimitFlipper(target_angular_flipper1 - ANG_FLIPPER_STEP_SIZE)
                status1 = status1 + 0.1
                print("Left_front_flipper "+angs(target_angular_flipper1))
                control_angular_flipper1 = makeSimpleProfile(control_angular_flipper1, target_angular_flipper1, (ANG_FLIPPER_STEP_SIZE))
                pos[1]=control_angular_flipper1

            elif key == 'f' :
                target_angular_flipper1= checkAngularLimitFlipper(target_angular_flipper1 + ANG_FLIPPER_STEP_SIZE)
                status1 = status1 + 0.1
                print("Left_front_flipper "+angs(target_angular_flipper1))
                control_angular_flipper1 = makeSimpleProfile(control_angular_flipper1, target_angular_flipper1, (ANG_FLIPPER_STEP_SIZE))
                pos[1]=control_angular_flipper1

            #Flipper derecho trasero
            elif key == 'i' :
                target_angular_flipper2= checkAngularLimitFlipper(target_angular_flipper2 - ANG_FLIPPER_STEP_SIZE)
                status2 = status2 + 0.1
                print("Right_back_flipper "+angs(target_angular_flipper2))
                control_angular_flipper2 = makeSimpleProfile(control_angular_flipper2, target_angular_flipper2, (ANG_FLIPPER_STEP_SIZE))
                pos[2]=control_angular_flipper2
            elif key == 'k' :
                target_angular_flipper2= checkAngularLimitFlipper(target_angular_flipper2 + ANG_FLIPPER_STEP_SIZE)
                status2 = status2 + 0.1
                print("Right_back_flipper "+angs(target_angular_flipper2))
                control_angular_flipper2 = makeSimpleProfile(control_angular_flipper2, target_angular_flipper2, (ANG_FLIPPER_STEP_SIZE))
                pos[2]=control_angular_flipper2
            #Flipper izquierdo trasero
            elif key == 't' :
                target_angular_flipper3= checkAngularLimitFlipper(target_angular_flipper3 - ANG_FLIPPER_STEP_SIZE)
                status3 = status3 + 0.1
                print("Left_back_flipper "+angs(target_angular_flipper3))
                control_angular_flipper3 = makeSimpleProfile(control_angular_flipper3, target_angular_flipper3, (ANG_FLIPPER_STEP_SIZE))
                pos[3]=control_angular_flipper3

            elif key == 'g' :
                target_angular_flipper3= checkAngularLimitFlipper(target_angular_flipper3 + ANG_FLIPPER_STEP_SIZE)
                status3 = status3 + 0.1
                print("Left_back_flipper "+angs(target_angular_flipper3))
                control_angular_flipper3 = makeSimpleProfile(control_angular_flipper3, target_angular_flipper3, (ANG_FLIPPER_STEP_SIZE))
                pos[3]=control_angular_flipper3

            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            else:
                if (key == '\x03'):
                    break
            
            if status == 5 :
                print(msg)
                status = 0

            if status1==31.4:
                print(msg2)
                status1 = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)
            joint_state=JointState()
            joint_state.header.seq=seq+1
            joint_state.header.frame_id=''
            joint_state.header.stamp=rospy.get_rostime()
            joint_state.name=name
            joint_state.position=pos
            joint_state.velocity=vel
            joint_state.effort=effort
            pub2.publish(joint_state)
            
    #except:
        #print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)
    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
