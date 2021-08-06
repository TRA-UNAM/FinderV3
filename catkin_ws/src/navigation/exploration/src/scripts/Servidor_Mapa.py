#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
#from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import Datos_rviz_mapeo,Datos_rviz_mapeoResponse
import numpy as np
import Metodos_de_navegacion_autonoma as md


class Servicio:

    def __init__(self):
        
       
       
       self.seq=0
       self.stamp=0
       self.frame_id=0
       self.resolution=0
       self.width=0
       self.height=0
       self.pos_x=0
       self.pos_y=0
       self.ori_x=0
       self.ori_y=0
       self.ori_z=0
       self.ori_w=0
       self.mapa=[]
       
       


    def map_callback(self,dato):
            #Estoy reacomodando el arreglo para poder manejarlo como una matriz, basandome en la altura y ancho
        self.seq=dato.header.seq
        self.stamp=dato.header.stamp
        self.frame_id=dato.header.frame_id
        self.resolution=dato.info.resolution
        self.width=dato.info.width
        self.height=dato.info.height
        self.pos_x=dato.info.origin.position.x
        self.pos_y=dato.info.origin.position.y
        self.ori_x=dato.info.origin.orientation.x
        self.ori_y=dato.info.origin.orientation.y
        self.ori_z=dato.info.origin.orientation.z
        self.ori_w=dato.info.origin.orientation.w
        self.mapa=dato.data
        self.Datos_rviz_mapeo_server()
        
        try:
            self.Datos_rviz_mapeo_server()
        finally:
            
            return Datos_rviz_mapeoResponse(seq=self.seq,stamp=self.stamp,frame_id=self.frame_id,posicion_x=dato.info.origin.position.x,posicion_y=dato.info.origin.position.y,orientacion_x=self.ori_x,orientacion_y=self.ori_y,orientacion_z=self.ori_z,orientacion_w=self.ori_w,width=self.width,height=self.height,resolution=self.resolution,mapa=self.mapa)
        
        
            

    def handle_Datos_rviz_mapa(self,req):
        
        print("Ya se obtuvo el mapa")
        
        return Datos_rviz_mapeoResponse(seq=self.seq,stamp=self.stamp,frame_id=self.frame_id,posicion_x=self.pos_x,posicion_y=self.pos_y,orientacion_x=self.ori_x,orientacion_y=self.ori_y,orientacion_z=self.ori_z,orientacion_w=self.ori_w,width=self.width,height=self.height,resolution=self.resolution,mapa=self.mapa)



    def Datos_rviz_mapeo_server(self):
        
        rospy.Service('/servicio_mapa', Datos_rviz_mapeo, self.handle_Datos_rviz_mapa)
        print("Listo para devolver los datos del mapa")
        
        #servicio_datos_mapa.shutdown("Vuelva pronto")#Se termina el servicio

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Mapa')
    servicio=Servicio()
    rospy.Subscriber('/map',OccupancyGrid,servicio.map_callback,queue_size=100)
    rospy.spin()
    
    
    

    
      
        
    
