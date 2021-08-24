#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
#from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import Puntos_Frontera, Puntos_FronteraResponse
import numpy as np
import Metodos_de_navegacion_autonoma as md

class Servicio():

    def __init__(self):

        self.x=[]
        self.y=[]
        
        

    def handle(self,req):
        grafo=np.array(req.mapa_inflado).reshape((req.height, req.width))
        c, l=np.shape(grafo)
        #nodos_objetivo=[]
        
        for j in range(l):
            for i in range(c): 
                
                if i==0 and j==0:
                
                    if (grafo[i,j]+grafo[i,j+1])==-1:
                            
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            #nodos_objetivo.append((i,j+1))
                            self.y.append(i*req.resolution)
                            self.x.append((j+1)*req.resolution)
                        

                    elif (grafo[i,j]+grafo[i+1,j])==-1:
                            
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i+1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i+1,j))

                elif i==0 and j>0 and j!=l-1:
                

                    if (grafo[i,j]+grafo[i,j+1])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j+1)*req.resolution)
                            #nodos_objetivo.append((i,j+1))

                    elif (grafo[i,j]+grafo[i+1,j])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i+1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i+1,j))

                    elif (grafo[i,j]+grafo[i,j-1])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j-1)*req.resolution)
                            #nodos_objetivo.append((i,j-1))

                elif i==0 and j==l-1:
                    
                    if (grafo[i,j]+grafo[i,j-1])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j-1)*req.resolution)
                            #nodos_objetivo.append((i,j-1))

                    elif (grafo[i,j]+grafo[i+1,j])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i+1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i+1,j))

                elif i>0 and i !=c-1 and j==0:
                    
                    if (grafo[i,j]+grafo[i-1,j])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))

                        else:
                            self.y.append((i-1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i-1,j))

                    elif (grafo[i,j]+grafo[i+1,j])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i+1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i+1,j))

                    elif (grafo[i,j]+grafo[i,j+1])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)

                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j+1)*req.resolution)
                            #nodos_objetivo.append((i,j+1))
                    
                elif i>0 and i!=c-1 and j>0 and j!=l-1:
                    
                    if (grafo[i,j]+grafo[i,j+1])==-1:

                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j+1)*req.resolution)
                            #nodos_objetivo.append((i,j+1))

                    elif (grafo[i,j]+grafo[i+1,j])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i+1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i+1,j))

                    elif (grafo[i,j]+grafo[i,j-1])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j-1)*req.resolution)
                            #nodos_objetivo.append((i,j-1))

                    elif (grafo[i,j]+grafo[i-1,j])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i-1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i-1,j))

                elif i>0 and i!=c-1 and j==l-1:
                    
                    if (grafo[i,j]+grafo[i,j-1])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j-1)*req.resolution)
                            #nodos_objetivo.append((i,j-1))

                    elif (grafo[i,j]+grafo[i-1,j])==-1:
                    
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i-1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i-1,j))

                    elif (grafo[i,j]+grafo[i+1,j])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i+1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i+1,j))
                    
                elif i==c-1 and j==0:
                    
                    if (grafo[i,j]+grafo[i,j+1])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j+1)*req.resolution)
                            #nodos_objetivo.append((i,j+1))

                    elif (grafo[i,j]+grafo[i-1,j])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i-1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i-1,j))

                elif i==c-1 and j>0 and j!=l-1:
                    
                    if (grafo[i,j]+grafo[i,j-1])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j-1)*req.resolution)
                            #nodos_objetivo.append((i,j-1))

                    elif (grafo[i,j]+grafo[i-1,j])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i-1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i-1,j))

                    elif (grafo[i,j]+grafo[i,j+1])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j+1)*req.resolution)
                            #nodos_objetivo.append((i,j+1))

                elif i==c-1 and j==l-1:
                
                    if (grafo[i,j]+grafo[i,j-1])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append(i*req.resolution)
                            self.x.append((j-1)*req.resolution)
                            #nodos_objetivo.append((i,j-1))

                    elif (grafo[i,j]+grafo[i-1,j])==-1:
                        
                        if grafo[i,j]==0:
                            self.y.append(i*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i,j))
                        else:
                            self.y.append((i-1)*req.resolution)
                            self.x.append(j*req.resolution)
                            #nodos_objetivo.append((i-1,j))

        
        print("Ya termine de calcular los puntos frontera\n")
        return Puntos_FronteraResponse(coord_x=self.x,coord_y=self.y)#Devolver todo en [m] 

        

    def Puntos_Frontera(self):

        rospy.Service('/servicio_puntos_frontera', Puntos_Frontera, self.handle)
        print("Listo para devolver los puntos frontera encontrados")
        
            #------------------------------------------------------------------           
        
if __name__ == "__main__":
    rospy.init_node('Servidor_Puntos_Frontera')
    servicio=Servicio()
    servicio.Puntos_Frontera()
    rospy.spin()
    
    
