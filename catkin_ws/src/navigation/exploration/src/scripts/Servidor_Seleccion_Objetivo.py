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

class Servicio:
    
    def handle(self,req):
        
        mapa_cts=np.array(req.mapa_costos).reshape((req.height, req.width))
        h=[]
        for i in range(len(req.centroides_x)):
            
            error_a=(math.atan2(req.centroides_y[i]-req.posicion_y_robot,req.centroides_x[i]-req.posicion_x_robot))-req.robot_a#Obtengo el error de angulo
            if error_a>math.pi:
                error_a=error_a-2*math.pi
        
            elif error_a<=-math.pi:
                error_a=error_a+2*math.pi

            error_d=math.sqrt((req.centroides_x[i] - req.posicion_x_robot)**2 + (req.centroides_y[i] - req.posicion_y_robot)**2)
            
            h.append(int(abs(error_a)+error_d+mapa_cts[req.centroides_y[i]//req.resolution][req.centroides_x[i]//req.resolution]))
            
        indice=np.where(h==min(h))[0]

        obj_x=req.centroides_x[indice]
        obj_y=req.centroides_y[indice]
        req.centroides_x.remove(obj_x)
        req.centroides_y.remove(obj_y)
        
        return ObjetivoResponse(obj_x=obj_x,obj_y=obj_y,centroides_x=req.centroides_x,centroides_y=req.centroides_y)
        

    def Objetivo(self):
            
        rospy.Service('/servicio_objetivo', Objetivo, self.handle)
        print("Listo para obtener el objetivo")
            


            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Objetivo')
    Objetivo()
    rospy.spin()
    