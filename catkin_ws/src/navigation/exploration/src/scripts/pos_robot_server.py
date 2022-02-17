#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import math
import rospy
import tf
from exploration.srv import GetPosRobot,GetPosRobotResponse
import numpy as np



class Server:

    def __init__(self):
        
       
       self.pos_x_robot=0
       self.pos_y_robot=0
       self.robot_a=0
       self.width=0
       self.height=0
       self.resolution=0
       self.map_origin_pos_x=0
       self.map_origin_pos_y=0
       self.listener=tf.TransformListener()
       


    def callback_GetPosRobot(self):
        
        try:
            (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            self.robot_a = 2*math.atan2(rot[2], rot[3])
            if self.robot_a > math.pi:
                self.robot_a = self.robot_a- 2*math.pi
            elif self.robot_a<=-math.pi:
                self.robot_a = self.robot_a+ 2*math.pi

            
            self.pos_x_robot = (abs(self.map_origin_pos_x))+(trans[0])+(0.7*math.cos(self.robot_a))
            self.pos_y_robot = (abs(self.map_origin_pos_x))+(trans[1])+(0.7*math.sin(self.robot_a))
            
        except:
            pass
        
        
        


    def handle_GetPosRobot(self,req):
        self.resolution=req.resolution
        self.map_origin_pos_x=req.map_origin_pos_x
        self.map_origin_pos_y=req.map_origin_pos_y
        self.width=req.width
        self.height=req.height
        self.callback_GetPosRobot()
        print("We already get the position of the robot")
        return GetPosRobotResponse(pos_x_robot=self.pos_x_robot,pos_y_robot=self.pos_y_robot,robot_a=self.robot_a)




    def GetPosRobot(self):
        
        rospy.Service('/navigation/localization/get_pos_robot', GetPosRobot, self.handle_GetPosRobot)
        print("The Pos Robot Server is ready for the request")
        
    

            
            
    
if __name__ == "__main__":
    rospy.init_node('pos_robot_server')
    server=Server()
    server.GetPosRobot()
    rospy.spin()
    

    
      
        
    

