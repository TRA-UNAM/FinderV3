#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import Objetivo,ObjetivoResponse
import numpy as np
import math


class Servicio:


   
    def handle(self,req):
        objetivo_x=[]
        objetivo_y=[]
        for i in range(len(req.coord_x)):
            error_a=(math.atan2(req.coord_y[i]-req.posicion_y_robot,req.coord_x[i]-req.posicion_y_robot))-req.robot_a#Obtengo el error de angulo

            print(error_a)
            if error_a>2.617993 or error_a<-2.617993:#Mayor y menor a +-150 grados
                pass
            else:
                objetivo_x.append(req.coord_x[i])
                objetivo_y.append(req.coord_y[i])
        
        
        dist_min=[]
        for j in range(len(objetivo_x)):
            minim_dst=math.sqrt((objetivo_x[j] - req.posicion_x_robot)**2 + ((objetivo_y[j] - req.posicion_y_robot)**2))
            dist_min.append(minim_dst)
        
        dist_min=np.array(dist_min)
        indice=np.where(dist_min==min(dist_min))[0][0]

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
    