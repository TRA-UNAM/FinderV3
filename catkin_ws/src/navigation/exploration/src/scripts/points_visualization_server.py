#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from exploration.srv import GetPointsVisualization,GetPointsVisualizationResponse


class Server:
   
    def handle_GetPointsVisualization(self,req):
        
        
        pub =rospy.Publisher('/visualization_marker',Marker, queue_size=100)
        markers=Marker(ns="points",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
        markers.header.stamp=rospy.Time()
        markers.header.frame_id="/map"
        markers.pose.orientation.w=1.0
        #Esta es la posicion de la marca
        markers.pose.position.x=req.map_origin_pos_x+0.025 #La referencia se encuentra en la esquina inferior derecha se se ve el robot avanzando hacia enfrente, en el punto más alejado verde en una esquina, y se encuentra a -10.275 positivos en ambos ejes
        markers.pose.position.y=req.map_origin_pos_y+0.025
        markers.pose.position.z=0
        #markers
        markers.scale.x=0.2#Tamaño de los markers
        markers.scale.y=0.2
        markers.scale.z=0.2
        #Los markers seran Azules
        #markers.color.r=1.0#Color  
        markers.color.r=1.0#Color 
        markers.color.a=1.0#Nitidez
        """
        p=Point()
        p.x=req.pos_x_robot#Para visualizar la posicion del robot
        p.y=req.pos_y_robot
        p.z=0
        markers.points.append(p)
        """
      
        
        
        
        markers.points=req.points
        
        
        
        pub.publish(markers)
        return GetPointsVisualizationResponse()
        

    def GetPointsVisualization(self):
            
        rospy.Service('/navigation/mapping/get_points_visalization', GetPointsVisualization, self.handle_GetPointsVisualization)
        print("The Points Visualization Server is ready for the request")
        
    

            
            
    
if __name__ == "__main__":
    rospy.init_node('points_visualization_server')
    server=Server()
    server.GetPointsVisualization()
    rospy.spin()
    