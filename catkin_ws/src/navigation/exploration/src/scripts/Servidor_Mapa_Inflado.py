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
        
       self.mapa_inflado=[]


    def handle(self,req):

        grafo=np.array(req.mapa).reshape((req.height, req.width))
        num_celdas_inflar=req.celdas_a_inflar
        print ("Calculando el mapa inflando con " +str(num_celdas_inflar) + " celdas")
        pub_inflated=rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
        c, l=np.shape(grafo)
        self.mapa_inflado=np.copy(grafo)
        #Voy a inflar el mapa para poder obtener las rutas y que el robot se aleje de las paredes del mapa, entonces sobre el original cada vez que me encuentre 
        
        for i in range(c):
            for j in range(l): 
                if grafo[i,j]==100:#checamos para cada punto del mapa si esta ocupado y si lo esta entonces inflamos
                    for k1 in range(i-num_celdas_inflar,i+num_celdas_inflar+1):#Aqui voy a iterar para inflar las filas
                        for k2 in range(j-num_celdas_inflar,j+num_celdas_inflar+1):#Aqui para inflar las columnas
                            self.mapa_inflado[k1,k2]=grafo[i,j]#Con k1 y k2 delimito el cuadrado que voy a llenar y marco como ocupada el cuadro al rededor del punto segun el numero de celdas que desean
        
        
        #Voy a publicar el mapa inflado para observarlo, por ello debo poner otro mapa
        mapa_ocupacion = OccupancyGrid()
        mapa_ocupacion.header.seq=req.seq
        mapa_ocupacion.header.stamp=req.stamp
        mapa_ocupacion.header.frame_id=req.frame_id
        mapa_ocupacion.info.resolution=req.resolution
        mapa_ocupacion.info.width = req.width
        mapa_ocupacion.info.height = req.height
        mapa_ocupacion.info.origin.position.x =req.posicion_x
        mapa_ocupacion.info.origin.position.y =req.posicion_y
        mapa_ocupacion.info.origin.orientation.x =req.orientacion_x
        mapa_ocupacion.info.origin.orientation.y =req.orientacion_y
        mapa_ocupacion.info.origin.orientation.z =req.orientacion_z
        mapa_ocupacion.info.origin.orientation.w =req.orientacion_w
        mapa_ocupacion.data= np.ravel(np.reshape(self.mapa_inflado, (len(req.mapa), 1)))
        pub_inflated.publish(mapa_ocupacion)
        
        
        return Inflar_mapaResponse(mapa_inflado=mapa_ocupacion.data)




    def Mapa_Inflado(self):

        rospy.Service('/servicio_mapa_inflado', Inflar_mapa, self.handle)
        print("Listo para devolver los datos del mapa inflado")
      
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Mapa_Inflado')
    servicio=Servicio()
    servicio.Mapa_Inflado()
    rospy.spin()
    
    

    
      
        
    
