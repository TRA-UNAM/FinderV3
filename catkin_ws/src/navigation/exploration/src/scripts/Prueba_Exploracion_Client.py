#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda


import rospy
from exploration.srv import Datos_rviz_mapeo, Inflar_mapa, Puntos_Frontera, Posicion_robot, Mapa_Costos, Puntos_Frontera, Visualizar_Puntos, Centroides, Objetivo, Mover_robot
from nav_msgs.msg import OccupancyGrid
import numpy as np
#import Servidor_Obtencion_ruta as ruta
import os
import sys
#import Servidor_de_control_para_mover_el_robot as mr
from geometry_msgs.msg import Twist
import math
import heapq
#import matplotlib.pyplot as plt


class Nodo:

    def __init__(self):
        self.init_node=rospy.init_node("Exploration_and_Mapping", anonymous=True)
        self.dato=0
        self.dato_mi=0
        self.dato_mc=0
        self.dato_pf=0
        self.dato_pr=0
        self.dato_o=0
        self.dato_v=0
        self.dato_km=0
        self.celdas_a_inflar=0.2#metros a inflar, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.celdas_de_costo=0.2#metros desde el obstaculo, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.pos_x_robot_ant=-10
        self.pos_y_robot_ant=-10
        self.robot_a_ant=2
        #self.costos_rutas=0
        self.cliente_mapa_costos=0
        self.cliente_mapa=0
        self.cliente_posicion_robot=0
        self.cliente_mapa_inflado=0
        self.cliente_puntos_frontera=0
        self.cliente_visualizacion=0
        self.cliente_centroides=0
        self.cliente_objetivo=0
        self.cliente_mr=0
        self.centroides_x=[]
        self.centroides_y=[]
        self.obj_ant_x=0
        self.obj_ant_y=0
        


        

    

    def prueba_exploracion(self):

        
        os.system("clear")#Limpiar terminal

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
        
            
        #----------------Obtencion de la posicion del robot en [m]--------------------------------

        
        print("Esperando al servicio_posicion_robot")
        rospy.wait_for_service('/servicio_posicion_robot')#Espero hasta que el servicio este habilitado
        try:
            
            self.cliente_posicion_robot=rospy.ServiceProxy('/servicio_posicion_robot',Posicion_robot)#Creo un handler para poder llamar al servicio
            self.dato_pr=self.cliente_posicion_robot(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution)
            
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor posicion del robot: %s"%e)

        
        print("Ya obtuve la posicion del robot")
        print("La posicion del robot en [m] es: "+str((self.dato_pr.posicion_x_robot,self.dato_pr.posicion_y_robot))+"\n")

        #---------------------------------------------------------------------
        
            
        if self.dato_pr.posicion_x_robot!=self.pos_x_robot_ant or self.dato_pr.robot_a!=self.robot_a_ant or self.dato_pr.posicion_y_robot!=self.pos_y_robot_ant:
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

            
            #-------------------------------------------------------------------------
            """
            #----------------Obtencion_mapa de costos---------------------------------
            print("Esperando al servicio_del_mapa_costos")
            rospy.wait_for_service('/servicio_mapa_costos')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_mapa_costos==0:
                    self.cliente_mapa_costos=rospy.ServiceProxy('/servicio_mapa_costos',Mapa_Costos)#Creo un handler para poder llamar al servicio
                self.dato_mc=self.cliente_mapa_costos(num_celdas_costo=self.celdas_de_costo,width=self.dato.width,height=self.dato.height,mapa=self.mapa_inflado,resolution=self.dato.resolution)
                self.dato_mc.mapa_costos#Asigno el mapa de costos una vez lo obtuve del servicio
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud del servidor mapa de costos: %s"%e)

            
            print("Ya obtuve el mapa de costos\n")

            
            #-------------------------------------------------------------------------
            """
            #----------------Obtencion puntos frontera--------------------------------
            print("Esperando al servicio_puntos_frontera")
            rospy.wait_for_service('/servicio_puntos_frontera')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_puntos_frontera==0:
                    self.cliente_puntos_frontera=rospy.ServiceProxy('/servicio_puntos_frontera',Puntos_Frontera)#Creo un handler para poder llamar al servicio
                self.dato_pf=self.cliente_puntos_frontera(width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution,mapa_inflado=self.dato_mi.mapa_inflado)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud del servidor puntos frontera: %s"%e)

            
            print("Ya obtuve los puntos frontera\n") 
            print("Encontre un total de "+str(len(self.dato_pf.coord_x))+" candidatos\n")


            
            
            #---------------------------------------------------------------------

            

            """
            
            
            #----------------Visualizar Puntos Frontera--------------------------------
            print("Esperando al servicio_visualizacion")
            rospy.wait_for_service('/servicio_visualizacion')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_visualizacion==0:
                    self.cliente_visualizacion=rospy.ServiceProxy('/servicio_visualizacion',Visualizar_Puntos)#Creo un handler para poder llamar al servicio
                    
                    
                self.dato_v=self.cliente_visualizacion(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,coord_x=self.dato_pf.coord_x,coord_y=self.dato_pf.coord_y,posicion_x_robot=self.dato_pr.posicion_x_robot,posicion_y_robot=self.dato_pr.posicion_y_robot)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud del servidor puntos frontera: %s"%e)

            
            print("Ya se pueden visualizar los puntos\n") 
        
        
            #---------------------------------------------------------------------
        
            """
            
            #----------------Obtener centroides--------------------------------
            print("Esperando al servicio centroides")
            rospy.wait_for_service('/servicio_centroides')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_centroides==0:
                    self.cliente_centroides=rospy.ServiceProxy('/servicio_centroides',Centroides)#Creo un handler para poder llamar al servicio
                
                self.dato_km=self.cliente_centroides(coord_x=self.dato_pf.coord_x,coord_y=self.dato_pf.coord_y)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud de obtener los centroides: %s"%e)

            
            print("Ya se tienen los puntos objetivo a partir de {} clusters\n".format(self.dato_km.k)) 
            
            #self.centroides_x=set(np.concatenate((self.centroides_x, self.dato_km.centroides_x)))
            #self.centroides_y=set(np.concatenate((self.centroides_y, self.dato_km.centroides_y)))
            
            
            
            #---------------------------------------------------------------------
        
            #----------------Visualizar Centroides --------------------------------
            print("Esperando al servicio_visualizacion")
            rospy.wait_for_service('/servicio_visualizacion')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_visualizacion==0:
                    self.cliente_visualizacion=rospy.ServiceProxy('/servicio_visualizacion',Visualizar_Puntos)#Creo un handler para poder llamar al servicio
                    
                    
                self.dato_v=self.cliente_visualizacion(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,coord_x=self.dato_km.centroides_x,coord_y=self.dato_km.centroides_y,posicion_x_robot=self.dato_pr.posicion_x_robot,posicion_y_robot=self.dato_pr.posicion_y_robot)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud del servidor puntos frontera: %s"%e)

            
            print("Ya se pueden visualizar los puntos\n") 
            
            
            #---------------------------------------------------------------------
            
            
            
            #----------------Obtener el punto objetivo--------------------------------
            print("Esperando al servicio punto objetivo")
            rospy.wait_for_service('/servicio_objetivo')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_objetivo==0:
                    self.cliente_objetivo=rospy.ServiceProxy('/servicio_objetivo',Objetivo)#Creo un handler para poder llamar al servicio

                self.dato_o=self.cliente_objetivo(centroides_x=self.dato_km.centroides_x,centroides_y=self.dato_km.centroides_y,posicion_x_robot=self.dato_pr.posicion_x_robot,posicion_y_robot=self.dato_pr.posicion_y_robot,robot_a=self.dato_pr.robot_a,obj_ant_x=self.obj_ant_x,obj_ant_y=self.obj_ant_y) 
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud de obtener un punto objetivo: %s"%e)

            
            print("Ya se tienen el punto objetivo: {},{}".format(self.dato_o.obj_x,self.dato_o.obj_y)) 
            #self.centroides_x=self.dato_o.centroides_x
            #self.centroides_y=self.dato_o.centroides_y
            obj_x=[self.dato_o.obj_x]
            obj_y=[self.dato_o.obj_y]
            
            
            
            

            
            #---------------------------------------------------------------------
            """
            
            #----------------Visualizar Punto objetivo--------------------------------
            print("Esperando al servicio_visualizacion")
            rospy.wait_for_service('/servicio_visualizacion')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_visualizacion==0:
                    self.cliente_visualizacion=rospy.ServiceProxy('/servicio_visualizacion',Visualizar_Puntos)#Creo un handler para poder llamar al servicio
                    
                    
                self.dato_v=self.cliente_visualizacion(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,coord_x=obj_x,coord_y=obj_y,posicion_x_robot=self.dato_pr.posicion_x_robot,posicion_y_robot=self.dato_pr.posicion_y_robot)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud del servidor puntos frontera: %s"%e)

            
            print("Ya se pueden visualizar los puntos\n") 
            
            
            #---------------------------------------------------------------------
            
            """

            
            
            #----------------Mover al Robot--------------------------------
                
            
            print("Esperando al servicio_mover_robot")
            rospy.wait_for_service('/servicio_mover_robot')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_mr==0:
                    self.cliente_mr=rospy.ServiceProxy('/servicio_mover_robot',Mover_robot)#Creo un handler para poder llamar al servicio
                    
                    
                dato_mr=self.cliente_mr(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,obj_x=obj_x,obj_y=obj_y,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud de mover el robot: %s"%e)

            
            print("Ya movi el robot hasta el punto seleccionado\n") 
            
        

        
            
        
        
            
            #---------------------------------------------------------------------
            
            
            #-----------------------Refresco la posicion del robot anterior----------------------------------------
            self.pos_x_robot_ant=self.dato_pr.posicion_x_robot
            self.pos_y_robot_ant=self.dato_pr.posicion_y_robot
            self.robot_a_ant=self.dato_pr.robot_a
            self.obj_ant_x=self.dato_o.obj_x
            self.obj_ant_y=self.dato_o.obj_y

            #---------------------------------------------------------------------
            #----------------Obtencion de la ruta--------------------------------------
                #print(self.pos_x_robot,self.pos_y_robot,punto_objetivo[0],punto_objetivo[1])
            #if self.pos_x_robot!=0 and self.pos_y_robot!=0:
                
                
                #self.path=ruta.a_star(int(self.pos_y_robot),int(self.pos_x_robot),self.puntos_frontera[punto_objetivo[0]][0],self.puntos_frontera[punto_objetivo[0]][1],self.mapa_inflado,self.mapa_de_costos,self)
                #if len(self.path)!=0:
                    #self.path=ruta.get_smooth_path(self.path,0.7,0.1)
                #self.path.append([self.pos_y_robot,self.pos_x_robot])
    

if __name__== "__main__":
    nodo=Nodo()
    loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        nodo.prueba_exploracion()
        loop.sleep()
    

    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_cmd_vel.publish(Twist())