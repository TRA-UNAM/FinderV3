#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
#from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import Mapa_Costos, Mapa_CostosResponse
import numpy as np
import Metodos_de_navegacion_autonoma as md


class Servicio:

    def __init__(self):
        
       self.mapa_costos=[]


    def handle(self,req):

        grafo=np.array(req.mapa).reshape((req.height, req.width))
        print ("Calculando el mapa de costos con " +str(req.num_celdas_costo) + " celdas\n")
        cost_map = np.copy(grafo)
        [height, width] = grafo.shape
        
        for i in range(height):
            for j in range(width):
                if grafo[i,j]==100:#checamos para cada punto del mapa si esta ocupado 
                    for k1 in range(-req.num_celdas_costo,req.num_celdas_costo+1):#Voy a ir de menos el radio hasta más el radio
                        for k2 in range(-req.num_celdas_costo,req.num_celdas_costo+1):#Aqui para inflar las columnas
                            cost=req.num_celdas_costo-max(abs(k1),abs(k2))#Se resta el radio de costo a la cordenada en valor absoluto mayor 
                            if cost>cost_map[i+k1,j+k2]:#Si el costo es mayor a el valor que tenia la celda entonces si la pongo, si no es así, se queda igual
                                cost_map[i+k1,j+k2]=cost+1#Con k1 y k2 delimito el cuadrado que voy a llenar y voy a tener que comparar para ver si sustituyo
        
        
        self.mapa_costos= np.ravel(np.reshape(cost_map, (len(req.mapa), 1)))  
            
        
        return Mapa_CostosResponse(mapa_costos=self.mapa_costos)




    def Mapa_Costos(self):

        rospy.Service('/servicio_mapa_costos', Mapa_Costos, self.handle)
        print("Listo para devolver los datos del mapa de costos")
      
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Mapa_Costos')
    servicio=Servicio()
    servicio.Mapa_Costos()
    rospy.spin()
    
    

    
      
        
    
