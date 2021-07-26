#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda


import rospy
from exploration.srv import Datos_rviz_mapeo, Inflar_mapa, Puntos_Frontera, Posicion_robot
from nav_msgs.msg import OccupancyGrid
import numpy as np
import Servidor_Puntos_Frontera as Spf
import Servidor_K_means as km
import Visualización_de_puntos as vp




class Nodo:

    def __init__(self):
        self.init_node=rospy.init_node("Exploration_and_Mapping", anonymous=True)
        self.dato=0
        self.dato_mi=0
        self.dato_pf=0
        self.mapa_inflado=0
        self.celdas_a_inflar=5#número de celdas a inflar, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.celdas_de_costo=5#número de celdas de costo desde el obstaculo, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.puntos_frontera=[]
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.costos=0
        self.mapa_de_costos=0
        self.costos_rutas=0
        self.centroides=0
        

    

    def prueba_exploracion(self):
        #---------------Obtencion de mapa------------------------------------
        print("Esperando al servicio_del_mapa")
        rospy.wait_for_service('/servicio_mapa')#Espero hasta que el servicio este habilitado
        try:
            cliente_mapa=rospy.ServiceProxy('/servicio_mapa',Datos_rviz_mapeo)#Creo un handler para poder llamar al servicio
            self.dato=cliente_mapa()#Llamo el servicio
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor mapa: %s"%e)

        
        print("Ya obtuve el mapa\n")


        posicion_ant_robot_x=1
        posicion_ant_robot_y=1

        
        #----------------------------------------------------------------------
        if round(self.pos_x_robot,1)!=round(posicion_ant_robot_x,1) and round(self.pos_x_robot,1)!=round(posicion_ant_robot_y,1):
            
            #----------------Obtención de la posición del robot--------------------------------

            posicion_ant_robot_x=self.pos_x_robot
            posicion_ant_robot_y=self.pos_y_robot 

            print("Esperando al servicio_posicion_robot")
            rospy.wait_for_service('/servicio_posicion_robot')#Espero hasta que el servicio este habilitado
            try:
                cliente_posicion_robot=rospy.ServiceProxy('/servicio_posicion_robot',Posicion_robot)#Creo un handler para poder llamar al servicio
                posicion_robot=cliente_posicion_robot(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution)
                self.pos_x_robot=posicion_robot.posicion_x_robot
                self.pos_y_robot=posicion_robot.posicion_y_robot
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud del servidor posicion del robot: %s"%e)

            
            print("Ya obtuve la posicion del robot")
            print("La posicion del robot es: "+str((self.pos_x_robot,self.pos_y_robot))+"\n")

            #---------------------------------------------------------------------




            #----------------Obtencion_mapa_inflado--------------------------------
            print("Esperando al servicio_del_mapa_inflado")
            rospy.wait_for_service('/servicio_mapa_inflado')#Espero hasta que el servicio este habilitado
            try:
                cliente_mapa_inflado=rospy.ServiceProxy('/servicio_mapa_inflado',Inflar_mapa)#Creo un handler para poder llamar al servicio
                self.dato_mi=cliente_mapa_inflado(celdas_a_inflar=self.celdas_a_inflar,seq=self.dato.seq,stamp=self.dato.stamp,frame_id=self.dato.frame_id,posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,orientacion_x=self.dato.orientacion_x,orientacion_y=self.dato.orientacion_y,orientacion_z=self.dato.orientacion_z,orientacion_w=self.dato.orientacion_w,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution,mapa=self.dato.mapa)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud del servidor inflar mapa: %s"%e)

            
            print("Ya obtuve el mapa inflado\n")

            self.mapa_inflado=self.dato_mi.mapa_inflado
            #---------------------------------------------------------------------

            #----------------Obtencion_puntos frontera--------------------------------
            print("Esperando la obtencion de puntos frontera")
            self.puntos_frontera=Spf.Busqueda_Puntos_frontera(self,self.mapa_inflado)
            print("Ya obtuve el los puntos frontera\n")       

            
            #---------------------------------------------------------------------

            #----------------Obtencion_centroides--------------------------------
            if len(self.puntos_frontera)>0:
                self.centroides=km.k_means(self.puntos_frontera)
            if len(self.centroides)!=0:
                print("Esperando la obtencion los centroides de los puntos frontera")
                print("Ya obtuve los centroides: "+str(self.centroides)+"\n")
                print("Obtendre el punto objetivo")
                distancia_1=np.square(((self.centroides[0][0]-self.pos_x_robot)**2)+((self.centroides[0][1]-self.pos_y_robot)**2))
                distancia_2=np.square(((self.centroides[1][0]-self.pos_x_robot)**2)+((self.centroides[1][1]-self.pos_y_robot)**2))
                if distancia_1>distancia_2:
                    punto_objetivo=self.centroides[0]
                else:
                    punto_objetivo=self.centroides[1]
                
                print("El punto objetivo es: "+str(punto_objetivo)+"\n")
            #---------------------------------------------------------------------


            
            #----------------Visualización de objetivos--------------------------------
                
                while True:
                    vp.visualizacion_objetivos(self,punto_objetivo)
                    





if __name__== "__main__":
    nodo=Nodo()
    loop=rospy.Rate(20)
    while not rospy.is_shutdown():
        loop.sleep()
        nodo.prueba_exploracion()
        