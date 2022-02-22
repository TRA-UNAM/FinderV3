#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import GetGoalPoint,GetGoalPointResponse
import numpy as np
import math
from geometry_msgs.msg import Point
import heapq

class Server:




    
    def handle_GetGoalPoint(self,req):
        alpha=0.6
        betha=0.4
        h=[]
        for i in range(len(req.points)):
            
            a_error=(math.atan2(req.points[i].y-req.pos_y_robot,req.points[i].x-req.pos_x_robot))-req.pos_a_robot#Obtengo el error de angulo
            if a_error>math.pi:
                a_error=a_error-2*math.pi
        
            elif a_error<=-math.pi:
                a_error=a_error+2*math.pi

            d_error=math.sqrt((req.points[i].x - req.pos_x_robot)**2 + (req.points[i].y - req.pos_y_robot)**2)
            heapq.heappush(h,((alpha*(a_error**2)+betha*d_error),(req.points[i].x ,req.points[i].y)))
            
            

        
        (goal_x,goal_y)=heapq.heappop(h)[1]
        
        
        if abs(goal_x-req.last_objective.x)<0.05 and abs(goal_y-req.last_objective.y)<0.05:
            (goal_x,goal_y)=heapq.heappop(h)[-1]
        
        
        goal=Point()
        goal.x=goal_x
        goal.y=goal_y
        
        return GetGoalPointResponse(goal=goal)
        

    def GetGoalPoint(self):
            
        rospy.Service('/navigation/mapping/get_goal_point', GetGoalPoint, self.handle_GetGoalPoint)
        print("The Goal Point Server is ready for the request")
            


            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('goal_point_server')
    server=Server()
    server.GetGoalPoint()
    rospy.spin()
    