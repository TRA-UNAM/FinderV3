#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def visualizacion_objetivos(self,objetivo,pos_x,pos_y):
    
    
    pub =rospy.Publisher('/visualization_marker',Marker, queue_size=100)
    puntos=Marker(ns="puntos_objetivo",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
    puntos.header.stamp=rospy.Time()
    puntos.header.frame_id="/map"
    puntos.pose.orientation.w=1.0
    #Esta es la posicion de la marca
    puntos.pose.position.x=self.dato.posicion_x+0.025 #La referencia se encuentra en la esquina inferior derecha se se ve el robot avanzando hacia enfrente, en el punto más alejado verde en una esquina, y se encuentra a -10.275 positivos en ambos ejes
    puntos.pose.position.y=self.dato.posicion_y+0.025
    puntos.pose.position.z=0
    #Puntos
    puntos.scale.x=0.08#Tamaño de los puntos
    puntos.scale.y=0.08
    puntos.scale.z=0.08
    #Los puntos seran verdes
    #puntos.color.r=1.0#Color  
    puntos.color.b=1.0#Color 
    puntos.color.a=1.0#Nitidez

    p=Point()
    p.x=pos_x#Se hizo una regla de tres o se multiplico por la resolución del mapa para ajustar los valores de la matriz a los valores de 
    p.y=pos_y
    p.z=0
    puntos.points.append(p)
    
    
    
    
    for i in range(len(objetivo)):
        p=Point()
        p.x=objetivo[1]*self.dato.resolution#[i]*self.dato.resolution#Se hizo una regla de tres o se multiplico por la resolución del mapa para ajustar los valores de la matriz a los valores de 
        p.y=objetivo[0]*self.dato.resolution#[i]*self.dato.resolution
        p.z=0
        puntos.points.append(p)
    
    
    
    pub.publish(puntos)
    
    

