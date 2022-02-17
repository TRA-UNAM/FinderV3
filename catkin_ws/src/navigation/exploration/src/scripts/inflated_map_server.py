#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import GetInflatedMap, GetInflatedMapResponse
import numpy as np



class Server:

    def __init__(self):
        
       self.inflated_map=[]


    def handle_GetInflatedMap(self,req):
        
        map=np.array(req.map.data).reshape((req.map.info.height, req.map.info.width))
        inflated_cells=int(req.inflated_cells/req.map.info.resolution)
        print ("Getting the inflated map with " +str(inflated_cells) + " inflated cells")
        pub_inflated=rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
        c, l=np.shape(map)
        self.inflated_map=np.copy(map)
        #Voy a inflar el mapa para poder obtener las rutas y que el robot se aleje de las paredes del mapa, entonces sobre el original cada vez que me encuentre 
        
        for i in range(c):
            for j in range(l): 
                if map[i,j]==100:#checamos para cada punto del mapa si esta ocupado y si lo esta entonces inflamos
                    for k1 in range(i-inflated_cells,i+inflated_cells+1):#Aqui voy a iterar para inflar las filas
                        for k2 in range(j-inflated_cells,j+inflated_cells+1):#Aqui para inflar las columnas
                            self.inflated_map[k1,k2]=map[i,j]#Con k1 y k2 delimito el cuadrado que voy a llenar y marco como ocupada el cuadro al rededor del punto segun el numero de celdas que desean
        
        
        #Voy a publicar el mapa inflado para observarlo, por ello debo poner otro mapa
        
        req.map.data= np.ravel(np.reshape(self.inflated_map, (len(req.map.data), 1)))
        pub_inflated.publish(req.map)
        
        
        return GetInflatedMapResponse(inflated_map=req.map)




    def GetInflatedMap(self):

        rospy.Service('/navigation/mapping/get_inflated_map', GetInflatedMap, self.handle_GetInflatedMap)
        print("The Inflated Map Server is ready for the request")
      
        
    
if __name__ == "__main__":
    rospy.init_node('inflated_map_server')
    server=Server()
    server.GetInflatedMap()
    rospy.spin()
    
    

    
      
        
    
