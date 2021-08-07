#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda


import rospy
from exploration.srv import Datos_rviz_mapeo, Inflar_mapa, Puntos_Frontera, Posicion_robot, Mapa_Costos
from nav_msgs.msg import OccupancyGrid
import numpy as np
import Servidor_Puntos_Frontera as Spf
import Servidor_K_means as km
import Servidor_Visualización_de_puntos as vp
import Servidor_Obtencion_ruta as ruta
import os
import sys
import Servidor_de_control_para_mover_el_robot as mr
from geometry_msgs.msg import Twist
import math


class Nodo:

    def __init__(self):
        self.init_node=rospy.init_node("Exploration_and_Mapping", anonymous=True)
        self.dato=0
        self.dato_mi=0
        self.dato_mc=0
        self.dato_pf=0
        self.mapa_inflado=0
        self.celdas_a_inflar=5#número de celdas a inflar, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.celdas_de_costo=5#número de celdas de costo desde el obstaculo, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.puntos_frontera=[]
        self.path=[]
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.costos=0
        self.mapa_de_costos=0
        self.costos_rutas=0
        self.centroides=0
        self.celdas_a_inflar=5
        self.cliente_mapa_costos=0
        self.cliente_mapa=0
        self.cliente_posicion_robot=0
        self.cliente_mapa_inflado=0

        

    

    def prueba_exploracion(self):
        os.system("clear")
        #---------------Obtencion de mapa------------------------------------
        print("Esperando al servicio_del_mapa")
        rospy.wait_for_service('/servicio_mapa')#Espero hasta que el servicio este habilitado
        try:
            if self.cliente_mapa==0:
                self.cliente_mapa=rospy.ServiceProxy('/servicio_mapa',Datos_rviz_mapeo)#Creo un handler para poder llamar al servicio

            self.dato=self.cliente_mapa()#Llamo el servicio
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor mapa: %s"%e)

        
        print("Ya obtuve el mapa\n")


       
        
        #----------------------------------------------------------------------
        
            
        #----------------Obtención de la posición del robot--------------------------------

        
        print("Esperando al servicio_posicion_robot")
        rospy.wait_for_service('/servicio_posicion_robot')#Espero hasta que el servicio este habilitado
        try:
            if self.cliente_posicion_robot==0:
                self.cliente_posicion_robot=rospy.ServiceProxy('/servicio_posicion_robot',Posicion_robot)#Creo un handler para poder llamar al servicio
            posicion_robot=self.cliente_posicion_robot(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution)
            self.pos_x_robot=posicion_robot.posicion_x_robot
            self.pos_y_robot=posicion_robot.posicion_y_robot
            self.robot_a=posicion_robot.robot_a
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor posicion del robot: %s"%e)

        
        print("Ya obtuve la posicion del robot")
        print("La posicion del robot es: "+str((self.pos_x_robot,self.pos_y_robot))+"\n")

        #---------------------------------------------------------------------
        #vp.visualizacion_objetivos(self,self.pos_x_robot, self.pos_y_robot)
        
        
        #----------------Obtencion_mapa_inflado--------------------------------
        print("Esperando al servicio_del_mapa_inflado")
        rospy.wait_for_service('/servicio_mapa_inflado')#Espero hasta que el servicio este habilitado
        try:
            if self.cliente_mapa_inflado==0:
                self.cliente_mapa_inflado=rospy.ServiceProxy('/servicio_mapa_inflado',Inflar_mapa)#Creo un handler para poder llamar al servicio
            self.dato_mi=self.cliente_mapa_inflado(celdas_a_inflar=self.celdas_a_inflar,seq=self.dato.seq,stamp=self.dato.stamp,frame_id=self.dato.frame_id,posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,orientacion_x=self.dato.orientacion_x,orientacion_y=self.dato.orientacion_y,orientacion_z=self.dato.orientacion_z,orientacion_w=self.dato.orientacion_w,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution,mapa=self.dato.mapa)
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor inflar mapa: %s"%e)

        
        print("Ya obtuve el mapa inflado\n")

        self.mapa_inflado=self.dato_mi.mapa_inflado
        #-------------------------------------------------------------------------

        #----------------Obtencion_mapa de costos---------------------------------
        print("Esperando al servicio_del_mapa_costos")
        rospy.wait_for_service('/servicio_mapa_costos')#Espero hasta que el servicio este habilitado
        try:
            if self.cliente_mapa_costos==0:
                self.cliente_mapa_costos=rospy.ServiceProxy('/servicio_mapa_costos',Mapa_Costos)#Creo un handler para poder llamar al servicio
            self.dato_mc=self.cliente_mapa_costos(num_celdas_costo=self.celdas_a_inflar,width=self.dato.width,height=self.dato.height,mapa=self.mapa_inflado)
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor mapa de costos: %s"%e)

        
        print("Ya obtuve el mapa inflado\n")

        self.mapa_de_costos=self.dato_mc.mapa_costos
        #-------------------------------------------------------------------------

        #----------------Obtencion puntos frontera--------------------------------
        print("Esperando la obtencion de puntos frontera")
        self.puntos_frontera=Spf.Busqueda_Puntos_frontera(self,self.mapa_inflado)
        print("Ya obtuve el los puntos frontera\n")       
        #---------------------------------------------------------------------
        
        error=[]
        
        #----------------Obtencion_centroides--------------------------------
        if len(self.puntos_frontera)>0:
            self.centroides=km.k_means(self.puntos_frontera)
        
        if len(self.centroides)!=0:
            print("Esperando la obtencion los centroides de los puntos frontera")
            print("Ya obtuve los centroides: "+str(self.centroides)+"\n")
            print("Obtendre el punto objetivo")
            
        for i in range(len(self.centroides)):
            error_a=(math.atan2(self.centroides[i][0]-self.pos_y_robot,self.centroides[i][1]-self.pos_x_robot))-self.robot_a#Obtengo el error de angulo
            if abs(error_a)>(math.pi+(math.pi)/3):
                pass
            else:
                punto_objetivo=self.centroides[i]
                
        
        for (x,y) in self.puntos_frontera:
            error.append(math.sqrt(((punto_objetivo[0] - x)**2 + ((punto_objetivo[1] - y)**2))))

        
        indice=min(error)
        for i in range(len(error)):
            if error[i]==indice:
                break
        punto_objetivo=self.puntos_frontera[i]
        #---------------------------------------------------------------------
        
        #----------------Obtención de la ruta--------------------------------------
            #print(self.pos_x_robot,self.pos_y_robot,punto_objetivo[0],punto_objetivo[1])
        if self.pos_x_robot!=0 and self.pos_y_robot!=0:
            
            
            #self.path=ruta.a_star(int(self.pos_y_robot),int(self.pos_x_robot),self.puntos_frontera[punto_objetivo[0]][0],self.puntos_frontera[punto_objetivo[0]][1],self.mapa_inflado,self.mapa_de_costos,self)
            #if len(self.path)!=0:
                #self.path=ruta.get_smooth_path(self.path,0.7,0.1)
            #self.path.append([self.pos_y_robot,self.pos_x_robot])
    #----------------Visualización de objetivos--------------------------------
        
        
            #while True:
                #vp.visualizacion_objetivos(self,self.puntos_frontera[punto_objetivo[0]],self.pos_y_robot,self.pos_x_robot)
            
            mr.mover_robot(self,self.puntos_frontera[i],self.dato.posicion_x,self.dato.posicion_y,self.dato.width,self.dato.height,self.dato.resolution,self.cliente_posicion_robot)  
    #---------------Mover el robot al punto deseado----------------------------

            



if __name__== "__main__":
    nodo=Nodo()
    loop=rospy.Rate(20)
    while not rospy.is_shutdown():
        loop.sleep()
        nodo.prueba_exploracion()
    

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_cmd_vel.publish(Twist())