#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import Objetivo,ObjetivoResponse
import numpy as np
import math
import random


class Servicio:


   
    def handle(self,req):
        objetivo_x=[]
        objetivo_y=[]
        for i in range(len(req.coord_x)):
            
            error_a=(math.atan2(req.coord_y[i]-req.posicion_y_robot,req.coord_x[i]-req.posicion_x_robot))-req.robot_a#Obtengo el error de angulo
            if error_a>math.pi:
                error_a=error_a-2*math.pi
        
            elif error_a<=-math.pi:
                error_a=error_a+2*math.pi

            
            if abs(error_a)>1.658062789:#Si es mayor a +-90 grados
                pass
            else:
                objetivo_x.append(req.coord_x[i])
                objetivo_y.append(req.coord_y[i])
        
        """
        dist_min=[]
        for j in range(len(objetivo_x)):
            minim_dst=math.sqrt((objetivo_x[j] - req.posicion_x_robot)**2 + ((objetivo_y[j] - req.posicion_y_robot)**2))
            dist_min.append(minim_dst)
        """
        candidatos=np.array(objetivo_x)
        #indice=np.where(dist_min==max(dist_min))[0][0]
        indice=np.where(candidatos==random.choice(objetivo_x))[0][0]
        return ObjetivoResponse(obj_x=objetivo_x[indice],obj_y=objetivo_y[indice])
        

    def Objetivo(self):
            
        rospy.Service('/servicio_objetivo', Objetivo, self.handle)
        print("Listo para obtener el objetivo")
            
    

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Objetivo')
    servicio=Servicio()
    servicio.Objetivo()
    rospy.spin()
    