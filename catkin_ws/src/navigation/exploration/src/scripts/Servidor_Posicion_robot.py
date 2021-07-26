#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
import math
import rospy
from nav_msgs.msg import Odometry
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
       


    def posicion_robot_callback(self,dato):
        
        self.angulo_robot=2*math.atan2(dato.pose.pose.orientation.z,dato.pose.pose.orientation.w)#Se trata de un cuaternion, por ello debo de hacer esto con dichas componentes
        correcion_x=0.7*math.cos(self.angulo_robot)
        correcion_y=0.7*math.sin(self.angulo_robot)
        self.pos_y_robot=(self.width/2)+(correcion_y/self.resolution)+(self.posicion_y/self.resolution)#Estan intercambiados en el mapa, por eso necesito invertirlos
        self.pos_x_robot=(self.height/2)+(correcion_x/self.resolution)+(self.posicion_x/self.resolution)#Se ajusta y se suman 4 metros para que este justo en el laser
        
        
        


    def handle_Posicion_robot(self,req):
        self.resolution=req.resolution
        self.posicion_x=req.posicion_x
        self.posicion_y=req.posicion_y
        self.width=req.width
        self.height=req.height
        rospy.Subscriber('/odom',Odometry,self.posicion_robot_callback,queue_size=20)
        print("Ya se obtuvo la posición del robot")
        return Posicion_robotResponse(posicion_x_robot=self.pos_x_robot,posicion_y_robot=self.pos_y_robot)




    def Datos_rviz_Posicion_robot(self):
        
        rospy.Service('/servicio_posicion_robot', Posicion_robot, self.handle_Posicion_robot)
        print("Listo para devolver la posicion del robot")
        
        #servicio_datos_mapa.shutdown("Vuelva pronto")#Se termina el servicio

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Posicion_robot')
    servicio=Servicio()
    servicio.Datos_rviz_Posicion_robot()
    rospy.spin()
    

    
      
        
    

