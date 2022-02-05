#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import Objetivo,ObjetivoResponse
import numpy as np
import math
import sys
import heapq

class Servicio:
    
    def handle(self,req):
        alpha=0.8
        betha=0.2
        #mapa_cts=np.array(req.mapa_costos).reshape((req.height, req.width))
        #print(mapa_cts)
        h=[]
        for i in range(len(req.centroides_x)):
            
            error_a=(math.atan2(req.centroides_y[i]-req.posicion_y_robot,req.centroides_x[i]-req.posicion_x_robot))-req.robot_a#Obtengo el error de angulo
            if error_a>math.pi:
                error_a=error_a-2*math.pi
        
            elif error_a<=-math.pi:
                error_a=error_a+2*math.pi

            error_d=math.sqrt((req.centroides_x[i] - req.posicion_x_robot)**2 + (req.centroides_y[i] - req.posicion_y_robot)**2)
            #costo=mapa_cts[int(req.centroides_y[i]/req.resolution),int(req.centroides_x[i]/req.resolution)]
            #print(costo)
            heapq.heappush(h,((alpha*(error_a**2)+betha*error_d),(req.centroides_x[i],req.centroides_y[i])))
            
            

        
        (objetivo_x,objetivo_y)=heapq.heappop(h)[1]
        list(req.centroides_x).remove(objetivo_x)
        list(req.centroides_y).remove(objetivo_y)

        if objetivo_x==req.obj_ant_x and objetivo_y==req.obj_ant_y:
            (objetivo_x,objetivo_y)=heapq.heappop(h)[1]
        
        
        
        #print(objetivo_x,objetivo_y)
        return ObjetivoResponse(obj_x=objetivo_x,obj_y=objetivo_y)#,centroides_x=req.centroides_x,centroides_y=req.centroides_y)
        

    def Servicio_Objetivo(self):
            
        rospy.Service('/servicio_objetivo', Objetivo, self.handle)
        print("Listo para obtener el objetivo")
            


            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Objetivo')
    servicio=Servicio()
    servicio.Servicio_Objetivo()
    rospy.spin()
    