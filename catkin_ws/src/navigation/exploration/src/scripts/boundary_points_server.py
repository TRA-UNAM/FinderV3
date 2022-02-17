#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import GetBoundaryPoints, GetBoundaryPointsResponse
import numpy as np
import cv2
from geometry_msgs.msg import Point

class Server():

    def __init__(self):

        self.boundary_points=[]
        
        
        

    def handle_GetBoundaryPoints(self,req):
        
        inflated_map=np.array(req.map.data).reshape((req.map.info.height, req.map.info.width))
        inflated_map = np.uint8(inflated_map)
        x=cv2.Sobel(inflated_map,cv2.CV_16S,1,0)
        y=cv2.Sobel(inflated_map,cv2.CV_16S,0,1)
        absX = cv2.convertScaleAbs(x)   # Transferencia de regreso a uint8  
        absY = cv2.convertScaleAbs(y) 
        borders= cv2.addWeighted(absX,0.2,absY,0.2,0)  
        ret,borders=cv2.threshold(borders,90,255,cv2.THRESH_BINARY)
        kernel = np.ones((2,2),np.uint8)
        borders=cv2.dilate(borders,kernel)
        borders=cv2.erode(borders,kernel)
        borders=cv2.erode(borders,kernel)
        borders_y=(np.where(borders==255)[0])*req.map.info.resolution
        borders_x=(np.where(borders==255)[1])*req.map.info.resolution
        for i in range(len(borders_x)):
            p=Point()
            p.x=borders_x[i]
            p.y=borders_y[i]
            p.z=0
            self.boundary_points.append(p)
            
            
        
        
        print("We already get the boundary points")
        return GetBoundaryPointsResponse(points=self.boundary_points,k=0)#Devolver todo en [m] 

        

    def GetBoundaryPoints(self):

        rospy.Service('/navigation/mapping/get_boundary_points', GetBoundaryPoints, self.handle_GetBoundaryPoints)
        print("The Boundary Points Server is ready for the request")
        
        
        
if __name__ == "__main__":
    rospy.init_node('boundary_points_server')
    server=Server()
    server.GetBoundaryPoints()
    rospy.spin()
    
    
