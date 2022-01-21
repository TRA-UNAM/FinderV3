#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
#from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import Puntos_Frontera, Puntos_FronteraResponse
import numpy as np
import cv2 as cv

class Servicio():

    def __init__(self):

        self.x=[]
        self.y=[]
        
        

    def handle(self,req):
        
        grafo=np.array(req.mapa_inflado).reshape((req.height, req.width))
        c, l=np.shape(grafo)
        grafo.flags.writeable = True
        grafo[grafo==-1]=255#Desconocido
        grafo[grafo==0]=0#Conocido
        #grafo[grafo==100]=255
        grafo = np.uint8(grafo)
        x=cv.Sobel(grafo,cv.CV_16S,1,0)
        y=cv.Sobel(grafo,cv.CV_16S,0,1)
        absX = cv.convertScaleAbs(x)   # Transferencia de regreso a uint8  
        absY = cv.convertScaleAbs(y) 
        bordes= cv.addWeighted(absX,0.5,absY,0.5,0)  
        #bordes=cv.Canny(grafo.T,200,255)
        cv.imshow("Result", bordes.T)  
        cv.waitKey(0)  
        cv.destroyAllWindows()
        
        print("Ya termine de calcular los puntos frontera\n")
        return Puntos_FronteraResponse(coord_x=self.x,coord_y=self.y)#Devolver todo en [m] 

        

    def Puntos_Frontera(self):

        rospy.Service('/servicio_puntos_frontera', Puntos_Frontera, self.handle)
        print("Listo para devolver los puntos frontera encontrados")
        
            #------------------------------------------------------------------           
        
if __name__ == "__main__":
    rospy.init_node('Servidor_Puntos_Frontera_FS')
    servicio=Servicio()
    servicio.Puntos_Frontera()
    rospy.spin()
    
    
