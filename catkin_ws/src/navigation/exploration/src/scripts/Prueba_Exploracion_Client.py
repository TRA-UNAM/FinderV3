#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda


import rospy
from exploration.srv import Datos_rviz_mapeo, Inflar_mapa, Puntos_Frontera
from nav_msgs.msg import OccupancyGrid
import numpy as np
import Servidor_Puntos_Frontera as Spf




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
        

    

    def prueba_exploracion(self):
        #---------------Obtencion de mapa------------------------------------
        print("Esperando al servicio_del_mapa")
        rospy.wait_for_service('/servicio_mapa')#Espero hasta que el servicio este habilitado
        try:
            cliente_mapa=rospy.ServiceProxy('/servicio_mapa',Datos_rviz_mapeo)#Creo un handler para poder llamar al servicio
            self.dato=cliente_mapa()#Llamo el servicio
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor mapa: %s"%e)

        
        print("Ya obtuve el mapa")
        
        #----------------------------------------------------------------------
        
        #----------------Obtencion_mapa_inflado--------------------------------
        print("Esperando al servicio_del_mapa_inflado")
        rospy.wait_for_service('/servicio_mapa_inflado')#Espero hasta que el servicio este habilitado
        try:
            cliente_mapa_inflado=rospy.ServiceProxy('/servicio_mapa_inflado',Inflar_mapa)#Creo un handler para poder llamar al servicio
            self.dato_mi=cliente_mapa_inflado(celdas_a_inflar=self.celdas_a_inflar,seq=self.dato.seq,stamp=self.dato.stamp,frame_id=self.dato.frame_id,posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,orientacion_x=self.dato.orientacion_x,orientacion_y=self.dato.orientacion_y,orientacion_z=self.dato.orientacion_z,orientacion_w=self.dato.orientacion_w,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution,mapa=self.dato.mapa)
            
        
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor inflar mapa: %s"%e)

        
        print("Ya obtuve el mapa inflado")

        self.mapa_inflado=self.dato_mi.mapa_inflado
        #---------------------------------------------------------------------

        #----------------Obtencion_puntos frontera--------------------------------
        print("Esperando la obtencion de puntos frontera")
        self.puntos_frontera=Spf.Busqueda_Puntos_frontera(self,self.mapa_inflado)
        

        
        #---------------------------------------------------------------------




if __name__== "__main__":
    nodo=Nodo()
    loop=rospy.Rate(1)
    while not rospy.is_shutdown():
        loop.sleep()
        nodo.prueba_exploracion()