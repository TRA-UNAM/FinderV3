#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import Objetivo,ObjetivoResponse
import numpy as np
import math
import random
import sys
import heapq

class Algoritmo:

    def __init__(self,height,width):
        self.in_open_list=np.full(np.shape((height, width)),False)#Estoy creando los verificadores de si esta en la lista cerrada o abierta y los llena de puros Falsos
        self.in_close_list=np.full(np.shape((height, width)),False)#Lista de puntos visitados
        self.open_list=[]
        self.close_list=[]
        






   
def handle(req):
    
    algoritmo=Algoritmo()
    for i in range(len(req.coord_x)):
        
        error_a=(math.atan2(req.coord_y[i]-req.posicion_y_robot,req.coord_x[i]-req.posicion_x_robot))-req.robot_a#Obtengo el error de angulo
        if error_a>math.pi:
            error_a=error_a-2*math.pi
    
        elif error_a<=-math.pi:
            error_a=error_a+2*math.pi

        error_d=math.sqrt((req.coord_x[i] - req.posicion_x_robot)**2 + (req.coord_y[i] - req.posicion_y_robot)**2)
        
        h=int(abs(error_a)+error_d)
        heapq.heappush(objetivo_x,(h,req.coord_x[i]))
        heapq.heappush(objetivo_y,(h,req.coord_y[i]))
    
    
    
    indice=heapq.heappop(open_list)[1]
    return ObjetivoResponse(obj_x=objetivo_x[indice],obj_y=objetivo_y[indice])
    

def Objetivo():
        
    rospy.Service('/servicio_objetivo', Objetivo, handle)
    print("Listo para obtener el objetivo")
        


            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Objetivo')
    Objetivo()
    rospy.spin()
    