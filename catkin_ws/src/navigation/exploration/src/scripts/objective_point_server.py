#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import GetObjectivePoint,GetObjectivePointResponse
import numpy as np
import math
from geometry_msgs.msg import Point
import heapq

class Server:




    
    def handle_GetObjectivePoint(self,req):
        alpha=0.6
        betha=0.4
        h=[]
        for i in range(len(req.points)):
            
            a_error=(math.atan2(req.points[i].y-req.pos_y_robot,req.points[i].x-req.pos_x_robot))-req.robot_a#Obtengo el error de angulo
            if a_error>math.pi:
                a_error=a_error-2*math.pi
        
            elif a_error<=-math.pi:
                a_error=a_error+2*math.pi

            d_error=math.sqrt((req.points[i].x - req.pos_x_robot)**2 + (req.points[i].y - req.pos_y_robot)**2)
            heapq.heappush(h,((alpha*(a_error**2)+betha*d_error),(req.points[i].x ,req.points[i].y)))
            
            

        
        (objective_x,objective_y)=heapq.heappop(h)[1]
        
        if objective_x==req.last_obj_x and objective_y==req.last_objs_y:
            (objective_x,objective_y)=heapq.heappop(h)[1]
        
        
        objective=Point()
        objective.x=objective_x
        objective.y=objective_y
        
        return GetObjectivePointResponse(objective=objective)
        

    def GetObjectivePoint(self):
            
        rospy.Service('/navigation/mapping/get_objective_point', GetObjectivePoint, self.handle_GetObjectivePoint)
        print("The Objective Point Server is ready for the request")
            


            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('objective_point_server')
    server=Server()
    server.GetObjectivePoint()
    rospy.spin()
    