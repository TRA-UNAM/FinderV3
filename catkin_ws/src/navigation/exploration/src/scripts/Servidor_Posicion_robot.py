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
                self.robot_a -= 2*math.pi
            elif self.robot_a<=-math.pi:
                self.robot_a += 2*math.pi

            
            self.robot_x = (abs(self.posicion_x)/self.resolution)+(trans[0]/self.resolution)+(0.7*math.cos(self.robot_a)/self.resolution)
            self.robot_y = (abs(self.posicion_y)/self.resolution)+(trans[1]/self.resolution)+(0.7*math.sin(self.robot_a)/self.resolution)
            
        except:
            pass
        
        
        


    def handle_Posicion_robot(self,req):
        self.resolution=req.resolution
        self.posicion_x=req.posicion_x
        self.posicion_y=req.posicion_y
        self.width=req.width
        self.height=req.height
        #rospy.Subscriber('/tf',Odometry,self.posicion_robot_callback,queue_size=10)
        self.posicion_robot_callback()
        print("Ya se obtuvo la posición del robot")
        return Posicion_robotResponse(posicion_x_robot=int(self.robot_x),posicion_y_robot=int(self.robot_y),robot_a=self.robot_a)




    def Datos_rviz_Posicion_robot(self):
        
        rospy.Service('/servicio_posicion_robot', Posicion_robot, self.handle_Posicion_robot)
        print("Listo para devolver la posicion del robot")
        
        #servicio_datos_mapa.shutdown("Vuelva pronto")#Se termina el servicio

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Posicion_robot')
    servicio=Servicio()
    rospy.spin()
    

    
      
        
    

