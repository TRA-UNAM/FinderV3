#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
#from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import Inflar_mapa, Inflar_mapaResponse
import numpy as np
import Metodos_de_navegacion_autonoma as md


class Servicio:

    def __init__(self):
        
       self.mapa=[]
       self.mapa_inflado=[]


    


    def handle(self,req):
        self.mapa=np.array(req.mapa).reshape((req.height, req.width))
        self.mapa_inflado=md.mapa_inflado_2(self,3,self.mapa)
        return Datos_rviz_mapeoResponse()




    def Mapa_Inflado(self):
        
        rospy.Service('/servicio_mapa_inflado', Inflar_mapa, self.handle)
        print("Listo para devolver los datos del mapa inflado")
        rospy.spin()
        #servicio_datos_mapa.shutdown("Vuelva pronto")#Se termina el servicio

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Mapa_Inflado')
    servicio=Servicio()
    servicio.Mapa_Inflado()
    
    

    
      
        
    
