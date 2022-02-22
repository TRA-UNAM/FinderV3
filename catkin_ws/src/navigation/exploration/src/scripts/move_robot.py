#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
import math
import numpy as np
from time import time
import tf
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool as Flag
from visualization_msgs.msg import Marker

class Node:

    def __init__(self):

        rospy.init_node('move_robot')
        self.loop=rospy.Rate(5)
        self.points=[]
        self.goal_x=0
        self.goal_y=0
        self.laser_readings=[]
        self.listener=tf.TransformListener()
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.flag=Flag()
        self.response=rospy.Publisher('/move_base_simple/goal_response',Flag,queue_size=10)
        self.pub_vis =rospy.Publisher('/visualization_marker',Marker, queue_size=10)
        self.markers=Marker(ns="points",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
        self.markers.header.stamp=rospy.Time()
        self.markers.header.frame_id="/map"
        self.markers.pose.orientation.w=1.0
        #Esta es la posicion de la marca
        self.markers.pose.position.x=0 
        self.markers.pose.position.y=0
        self.markers.pose.position.z=0
        #self.markers
        self.markers.scale.x=0.2#Tamaño de los self.markers
        self.markers.scale.y=0.2
        self.markers.scale.z=0.2
        #Los self.markers seran Azules
        #self.markers.color.r=1.0#Color  
        self.markers.color.r=1.0#Color 
        self.markers.color.a=1.0#Nitidez
    
    def callback_scan(self,msg):
        
        self.laser_readings = [[0,0] for i in range(len(msg.ranges))]
        for i in range(len(msg.ranges)):
            self.laser_readings[i] = [msg.ranges[i], msg.angle_min + i*msg.angle_increment]

    def callback_potential_fields(self,msg):
        self.goal_x=msg.x
        self.goal_y=msg.y
        self.potential_fields()
        
        
    def visualization_points(self,points):

        self.markers.points=points

        self.pub_vis.publish(self.markers)
    
    def getPosRobot(self):

        try:
            ([pos_x_robot, pos_y_robot, z], rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            pos_a_robot = 2*math.atan2(rot[2], rot[3])
            if pos_a_robot > math.pi:
                pos_a_robot = pos_a_robot- 2*math.pi
            elif pos_a_robot<=-math.pi:
                pos_a_robot = pos_a_robot+ 2*math.pi

            
            
        except:
            pass

        return pos_x_robot,pos_y_robot,pos_a_robot


    def calculate_control(self,pos_x_robot,pos_y_robot,robot_a,goal_x,goal_y):
        cmd_vel = Twist()
        
        #
        # Implement the control law given by:
        #
        a_error=(math.atan2(goal_y-pos_y_robot,goal_x-pos_x_robot))-robot_a#Obtengo el error de angulo
        alpha=0.5
        beta=0.5
        
        if a_error>math.pi:
            a_error=a_error-2*math.pi
        
        elif a_error<=-math.pi:
            a_error=a_error+2*math.pi

        
        v = 0.5*math.exp(-a_error*a_error/alpha)
        w = 0.5*(2/(1 + math.exp(-a_error/beta)) - 1)
        
        

        cmd_vel.linear.x=v
        cmd_vel.angular.z=w

        return cmd_vel



    def attraction_force(self,pos_x_robot, pos_y_robot, goal_x, goal_y):
        #
        # Calculate the attraction force, given the robot and goal positions.
        # Return a tuple of the form [force_x, force_y]
        # where force_x and force_y are the X and Y components
        # of the resulting attraction force w.r.t. map.
        #
        atraction_intensity=1
        force_x=(atraction_intensity/math.sqrt((pos_x_robot-goal_x)**2+(pos_y_robot-goal_y)**2))*(pos_x_robot-goal_x)
        force_y=(atraction_intensity/math.sqrt((pos_x_robot-goal_x)**2+(pos_y_robot-goal_y)**2))*(pos_y_robot-goal_y)

        return [force_x, force_y]

    def rejection_force(self,pos_x_robot, pos_y_robot, robot_a, laser_readings):
        #
        # Calculate the total rejection force given by the average
        # of the rejection forces caused by each laser reading.
        # laser_readings is an array where each element is a tuple [distance, angle] el angulo es con respecto al robot
        # both measured w.r.t. robot's frame.
        # See lecture notes for equations to calculate rejection forces.
        # Return a tuple of the form [force_x, force_y]
        # where force_x and force_y are the X and Y components
        # of the resulting rejection force w.r.t. map.
        #
        force_x=0
        force_y=0
        d0=0.8#A partir de 1 metros del robot empezara la repulsion
        rejection_intensity=2
        i=0
        
        for lectura_ls in laser_readings:
            d=lectura_ls[0]#Corresponde a la distancia d, desde el robot hasta el obstaculo
            
            
            if d>0 and d<d0 and np.isfinite(d) and not np.isnan(d):
                i+=1  
                theta_obs=robot_a+lectura_ls[1]#Obtengo el angulo del obstaculo con respecto al sistema de referencia general, ya que al angulo del robot le sumo el angulo del obstaculo con respecto al robot
                obs_point_x=pos_x_robot+d*math.cos(theta_obs)
                obs_point_y=pos_y_robot+d*math.sin(theta_obs)
                force_x+=((rejection_intensity*math.sqrt((1/d)-(1/d0)))/d)*(obs_point_x-pos_x_robot)
                force_y+=((rejection_intensity*math.sqrt((1/d)-(1/d0)))/d)*(obs_point_y-pos_y_robot)
            else:
                pass

        if i!=0:
            force_x=force_x/i
            force_y=force_y/i
            
        
        return [force_x, force_y]



    def potential_fields(self):

        
        
        print ("Moving to goal point " + str([self.goal_x, self.goal_y]) + " by potential fields\n")
        pos_x_robot, pos_y_robot, pos_a_robot=self.getPosRobot()
        epsilon=0.5
        dist_to_goal=math.sqrt((self.goal_x - pos_x_robot)**2 + (self.goal_y - pos_y_robot)**2)
        start=time()
        p=Point()
        while dist_to_goal>1:

            [fax, fay] = self.attraction_force(pos_x_robot, pos_y_robot, self.goal_x, self.goal_y)#Calculamos la fuerza de atraccion
            [frx, fry] = self.rejection_force(pos_x_robot, pos_y_robot, pos_a_robot,self.laser_readings)#Calculamos la fuerza de repulsion
            [fx,fy]=[fax+frx,fay+fry]#Obtenemos la fuerza resultante
            [px,py]=[pos_x_robot-epsilon*fx,pos_y_robot-epsilon*fy]#Obtenemos los puntos objetivo locales con la fuerza neta restada multiplicada por epsilon
            msg_cmd_vel=self.calculate_control(pos_x_robot,pos_y_robot,pos_a_robot,px,py)
            self.pub_cmd_vel.publish(msg_cmd_vel)
            p.x=self.goal_x
            p.y=self.goal_y
            self.visualization_points([p])
            self.loop.sleep()
            pos_x_robot, pos_y_robot, pos_a_robot=self.getPosRobot()
            dist_to_goal=math.sqrt((self.goal_x - pos_x_robot)**2 + (self.goal_y - pos_y_robot)**2)
            
            if (time()-start)>15:
                self.pub_cmd_vel.publish(Twist())
                break
            
                
        
        self.flag.data=False
        self.response.publish(self.flag)
        self.pub_cmd_vel.publish(Twist())
        print("Goal point reached\n")

    
    def main(self):    
        rospy.Subscriber('/navigation/move_base_simple/goal',Point,self.callback_potential_fields)
        rospy.Subscriber('/scan',LaserScan,self.callback_scan)
        rospy.spin()

if __name__ == "__main__":
    
    node=Node() 
    node.main()
    
    
    
    
    


