#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
#from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import Puntos_Frontera, Puntos_FronteraResponse
import numpy as np
import cv2

class Servicio():

    def __init__(self):

        self.x=[]
        self.y=[]
        
        

    def handle(self,req):
        
        grafo=np.array(req.mapa_inflado).reshape((req.height, req.width))
        #c, l=np.shape(grafo)
        #grafo.flags.writeable = True
        grafo = np.uint8(grafo)
        #cv2.imshow("Result", grafo) 
        x=cv2.Sobel(grafo,cv2.CV_16S,1,0)
        y=cv2.Sobel(grafo,cv2.CV_16S,0,1)
        absX = cv2.convertScaleAbs(x)   # Transferencia de regreso a uint8  
        absY = cv2.convertScaleAbs(y) 
        #print(absX[100,100])
        bordes= cv2.addWeighted(absX,0.2,absY,0.2,0)  
        #bordes=cv2.Canny(grafo.T,200,255)
        ret,bordes=cv2.threshold(bordes,90,255,cv2.THRESH_BINARY)
        kernel = np.ones((2,2),np.uint8)
        bordes=cv2.dilate(bordes,kernel)
        bordes=cv2.erode(bordes,kernel)
        bordes=cv2.erode(bordes,kernel)
        #bordes = cv2.morphologyEx(bordes, cv2.MORPH_CLOSE, kernel) 
        self.y=(np.where(bordes==255)[0])*req.resolution
        self.x=(np.where(bordes==255)[1])*req.resolution
        #print(x[0],y[0])
        #cv2.imshow("Result", bordes.T)  
        #cv2.waitKey(0)  
        #cv2.destroyAllWindows()
        
          
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
    
    
