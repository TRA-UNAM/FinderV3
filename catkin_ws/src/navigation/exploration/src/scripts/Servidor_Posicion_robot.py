#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
import math
import rospy
import tf
from exploration.srv import Posicion_robot,Posicion_robotResponse
import numpy as np
import Metodos_de_navegacion_autonoma as md


class Servicio:

    def __init__(self):
        
       
       self.f=0
       self.pos_x_robot=0
       self.pos_y_robot=0
       self.angulo_robot=0
       self.width=0
       self.height=0
       self.resolucion=0
       self.posicion_x=0
       self.posicion_y=0
       self.listener=tf.TransformListener()
       self.Datos_rviz_Posicion_robot()


    def posicion_robot_callback(self):
        
        try:
            (trans, rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            self.robot_a = 2*math.atan2(rot[2], rot[3])
            if self.robot_a > math.pi:
                self.robot_a = self.robot_a- 2*math.pi
            elif self.robot_a<=-math.pi:
                self.robot_a = self.robot_a+ 2*math.pi

            
            self.robot_x = (abs(self.posicion_x))+(trans[0])+(0.7*math.cos(self.robot_a))
            self.robot_y = (abs(self.posicion_y))+(trans[1])+(0.7*math.sin(self.robot_a))
            
        except:
            pass
        
        
        


    def handle_Posicion_robot(self,req):
        self.resolution=req.resolution
        self.posicion_x=req.posicion_x
        self.posicion_y=req.posicion_y
        self.width=req.width
        self.height=req.height
        self.posicion_robot_callback()
        print("Ya se obtuvo la posicion del robot")
        self.f=1
        return Posicion_robotResponse(posicion_x_robot=self.robot_x,posicion_y_robot=self.robot_y,robot_a=self.robot_a)




    def Datos_rviz_Posicion_robot(self):
        
        S=rospy.Service('/servicio_posicion_robot', Posicion_robot, self.handle_Posicion_robot)
        print("Listo para devolver la posicion del robot")
        if self.f==1:
            S.shutdown()
    

            
            
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Posicion_robot')
    servicio=Servicio()
    rospy.spin()
    

    
      
        
    

