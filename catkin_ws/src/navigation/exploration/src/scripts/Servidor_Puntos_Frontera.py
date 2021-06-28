#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
#from sensor_msgs.msg import LaserScan
import rospy
from nav_msgs.msg import OccupancyGrid
from exploration.srv import Puntos_Frontera, Puntos_FronteraResponse
import numpy as np
import Metodos_de_navegacion_autonoma as md



def Busqueda_Puntos_frontera(self,mapa_inflado):
    grafo=np.array(mapa_inflado).reshape((self.dato.height, self.dato.width))
    c, l=np.shape(grafo)
    nodos_objetivo=[]
    
    for j in range(l):
        for i in range(c): 
            
            if i==0 and j==0:
            
                if (grafo[i,j]+grafo[i,j+1])==-1:
                        
                        if grafo[i,j]==0:
                            nodos_objetivo.append((i,j))
                        else:
                            nodos_objetivo.append((i,j+1))
                    

                elif (grafo[i,j]+grafo[i+1,j])==-1:
                        
                        if grafo[i,j]==0:
                            nodos_objetivo.append((i,j))
                        else:
                            nodos_objetivo.append((i+1,j))

            elif i==0 and j>0 and j!=l-1:
            

                if (grafo[i,j]+grafo[i,j+1])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                    if (grafo[i,j]+grafo[i+1,j])==-1:

                        if grafo[i,j]==0:
                            nodos_objetivo.append((i,j))
                        else:
                            nodos_objetivo.append((i+1,j))

                elif (grafo[i,j]+grafo[i,j-1])==-1:

                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

            elif i==0 and j==l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==-1:

                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                elif (grafo[i,j]+grafo[i+1,j])==-1:

                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

            elif i>0 and i !=c-1 and j==0:
                
                if (grafo[i,j]+grafo[i-1,j])==-1:

                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))

                    else:
                        nodos_objetivo.append((i-1,j))

                elif (grafo[i,j]+grafo[i+1,j])==-1:

                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                elif (grafo[i,j]+grafo[i,j+1])==-1:

                    if grafo[i,j]==0:

                            nodos_objetivo.append((i,j))
                    else:
                            nodos_objetivo.append((i,j+1))
                
            elif i>0 and i!=c-1 and j>0 and j!=l-1:
                
                if (grafo[i,j]+grafo[i,j+1])==-1:

                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                elif (grafo[i,j]+grafo[i+1,j])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                elif (grafo[i,j]+grafo[i,j-1])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                elif (grafo[i,j]+grafo[i-1,j])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

            elif i>0 and i!=c-1 and j==l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                elif (grafo[i,j]+grafo[i-1,j])==-1:
                
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

                elif (grafo[i,j]+grafo[i+1,j])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))
                
            elif i==c-1 and j==0:
                
                if (grafo[i,j]+grafo[i,j+1])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                elif (grafo[i,j]+grafo[i-1,j])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

            elif i==c-1 and j>0 and j!=l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                elif (grafo[i,j]+grafo[i-1,j])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

                elif (grafo[i,j]+grafo[i,j+1])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

            elif i==c-1 and j==l-1:
            
                if (grafo[i,j]+grafo[i,j-1])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                elif (grafo[i,j]+grafo[i-1,j])==-1:
                    
                    if grafo[i,j]==0:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))


    print("Ya termine de calcular los puntos objetivo\n")
    print("Encontre un total de "+str(len(set(nodos_objetivo)))+" candidatos\n")
    return list(set(nodos_objetivo))

      
        
    
