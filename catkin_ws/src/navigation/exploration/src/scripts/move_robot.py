#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
import math
import numpy as np
from time import time
import tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from exploration.msg import PotentialFields


class Servicio:

    def __init__(self):

        
        self.loop=rospy.Rate(5)
        self.robot_a=0
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.goal_x=0
        self.goal_y=0
        self.laser_readings=[]
        self.width=0
        self.height=0
        self.resolution=0
        self.map_origin_pos_x=0
        self.map_origin_pos_y=0
        self.listener=tf.TransformListener()
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.get_goal = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/navigation/move_base_simple/goal',PotentialFields,self.callback_potential_fields)
    
    
    def callback_scan(self,msg):
        
        self.laser_readings = [[0,0] for i in range(len(msg.ranges))]
        for i in range(len(msg.ranges)):
            self.laser_readings[i] = [msg.ranges[i], msg.angle_min + i*msg.angle_increment]
            
    
    
    def getPosRobot(self):

        
        ([x, y, z], rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        self.robot_a = 2*math.atan2(rot[2], rot[3])
        if self.robot_a > math.pi:
            self.robot_a = self.robot_a- 2*math.pi
        elif self.robot_a<=-math.pi:
            self.robot_a = self.robot_a+ 2*math.pi

        
        self.pos_x_robot = (abs(self.map_origin_pos_x))+(x)+(0.7*math.cos(self.robot_a))
        self.pos_y_robot = (abs(self.map_origin_pos_y))+(y)+(0.7*math.sin(self.robot_a))
        

    

    def callback_potential_fields(self,msg):
        self.width=msg.width
        self.height=msg.height
        self.resolution=msg.resolution
        self.map_origin_pos_x=msg.map_origin_pos_x
        self.map_origin_pos_y=msg.map_origin_pos_y
        self.goal_x=msg.goal.x
        self.goal_y=msg.goal.y
        self.move_to_goal()
        



    def attraction_force(self,pos_x_robot, pos_y_robot, goal_x, goal_y):
        #
        # Calculate the attraction force, given the robot and goal positions.
        # Return a tuple of the form [force_x, force_y]
        # where force_x and force_y are the X and Y components
        # of the resulting attraction force w.r.t. map.
        #
        intensidad_atraccion=1
        force_x=(intensidad_atraccion/math.sqrt((pos_x_robot-goal_x)**2+(pos_y_robot-goal_y)**2))*(pos_x_robot-goal_x)
        force_y=(intensidad_atraccion/math.sqrt((pos_x_robot-goal_x)**2+(pos_y_robot-goal_y)**2))*(pos_y_robot-goal_y)

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
        intensidad_repulsion=3
        i=0
        
        for lectura_ls in laser_readings:
            d=lectura_ls[0]#Corresponde a la distancia d, desde el robot hasta el obstaculo
            
            
            if d>0 and d<d0 and np.isfinite(d) and not np.isnan(d):
                i+=1  
                theta_obs=robot_a+lectura_ls[1]#Obtengo el angulo del obstaculo con respecto al sistema de referencia general, ya que al angulo del robot le sumo el angulo del obstaculo con respecto al robot
                obs_point_x=pos_x_robot+d*math.cos(theta_obs)
                obs_point_y=pos_y_robot+d*math.sin(theta_obs)
                force_x+=((intensidad_repulsion*math.sqrt((1/d)-(1/d0)))/d)*(obs_point_x-pos_x_robot)
                force_y+=((intensidad_repulsion*math.sqrt((1/d)-(1/d0)))/d)*(obs_point_y-pos_y_robot)
            else:
                pass

        if i!=0:
            force_x=force_x/i
            force_y=force_y/i
            
        
        return [force_x, force_y]




    def move_to_goal(self):
        
        
        self.getPosRobot()
        epsilon=0.5
        dist_to_goal=math.sqrt((self.goal_x - self.pos_x_robot)**2 + (self.goal_y - self.pos_y_robot)**2)
        start=time()
        
        while dist_to_goal>0.5:

            self.loop.sleep()
            rospy.Subscriber("/scan", LaserScan, self.callback_scan)
            [fax, fay] = self.attraction_force(self.pos_x_robot, self.pos_y_robot, self.goal_x, self.goal_y)#Calculamos la fuerza de atraccion
            [frx, fry] = self.rejection_force (self.pos_x_robot, self.pos_y_robot, self.robot_a,self.laser_readings)#Calculamos la fuerza de repulsion
            [fx,fy]=[fax+frx,fay+fry]#Obtenemos la fuerza resultante
            [px,py]=[self.pos_x_robot-epsilon*fx,self.pos_y_robot-epsilon*fy]#Obtenemos los puntos objetivo locales con la fuerza neta restada multiplicada por epsilon
            msg_cmd_vel=self.calculate_control(self.pos_x_robot,self.pos_y_robot,self.robot_a,px,py)
            self.pub_cmd_vel.publish(msg_cmd_vel)
            self.getPosRobot()
            dist_to_goal=math.sqrt((self.goal_x - self.pos_x_robot)**2 + (self.goal_y - self.pos_y_robot)**2)
            
            if (time()-start)>15:
                self.pub_cmd_vel.publish(Twist())
                break
                
            
            
        self.pub_cmd_vel.publish(Twist())
        

    
        
        
        
        

    
    

if __name__ == "__main__":
    rospy.init_node('move_robot')
    servicio=Servicio()
    rospy.spin()


