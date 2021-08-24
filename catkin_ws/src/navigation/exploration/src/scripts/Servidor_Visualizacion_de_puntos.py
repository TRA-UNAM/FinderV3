#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from exploration.srv import Visualizar_Puntos,Visualizar_PuntosResponse


class Servicio:


   
    def handle(self,req):
        
        
        pub =rospy.Publisher('/visualization_marker',Marker, queue_size=100)
        puntos=Marker(ns="puntos_objetivo",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
        puntos.header.stamp=rospy.Time()
        puntos.header.frame_id="/map"
        puntos.pose.orientation.w=1.0
        #Esta es la posicion de la marca
        puntos.pose.position.x=req.posicion_x+0.025 #La referencia se encuentra en la esquina inferior derecha se se ve el robot avanzando hacia enfrente, en el punto más alejado verde en una esquina, y se encuentra a -10.275 positivos en ambos ejes
        puntos.pose.position.y=req.posicion_y+0.025
        puntos.pose.position.z=0
        #Puntos
        puntos.scale.x=0.1#Tamaño de los puntos
        puntos.scale.y=0.1
        puntos.scale.z=0.1
        #Los puntos seran verdes
        #puntos.color.r=1.0#Color  
        puntos.color.b=1.0#Color 
        puntos.color.a=1.0#Nitidez

        p=Point()
        p.x=req.posicion_x_robot#Para visualizar la posicion del robot
        p.y=req.posicion_y_robot
        p.z=0
        puntos.points.append(p)
        
        
        
        
        for i in range(len(req.coord_x)):
            p=Point()
            p.x=req.coord_x[i]
            p.y=req.coord_y[i]
            p.z=0
            puntos.points.append(p)
        
        
        
        pub.publish(puntos)
        return Visualizar_PuntosResponse()
        

    def Visualizar_objetivos(self):
            
        rospy.Service('/servicio_visualizacion', Visualizar_Puntos, self.handle)
        print("Listo para dibujar los puntos")
            
    

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Visualizar')
    servicio=Servicio()
    servicio.Visualizar_objetivos()
    rospy.spin()
    