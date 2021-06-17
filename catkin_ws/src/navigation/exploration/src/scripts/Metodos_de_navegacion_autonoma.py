#!/usr/bin/env python3
# coding=utf-8
import rospy
import numpy as np
#import networkx as nx
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.srv import GetMapResponse
from nav_msgs.srv import GetMapRequest
from nav_msgs.msg import Odometry
import heapq

#El mapa de Inflado 2 infla todos los puntos alrededor de un obstaculo el número de celdas que le digas
def mapa_inflado_2(self,num_celdas_inflar,grafo):

    print ("Calculando el mapa inflando " +str(num_celdas_inflar) + " cells\n")
    pub_inflated=rospy.Publisher("/inflated_map", OccupancyGrid, queue_size=10)
    c, l=np.shape(grafo)
    mapa_inflado=np.copy(grafo)
    #Voy a inflar el mapa para poder obtener las rutas y que el robot se aleje de las paredes del mapa, entonces sobre el original cada vez que me encuentre 
    
    for i in range(c):
        for j in range(l): 
            if grafo[i,j]==100:#checamos para cada punto del mapa si esta ocupado y si lo esta entonces inflamos
                for k1 in range(i-num_celdas_inflar,i+num_celdas_inflar+1):#Aqui voy a iterar para inflar las filas
                    for k2 in range(j-num_celdas_inflar,j+num_celdas_inflar+1):#Aqui para inflar las columnas
                        mapa_inflado[k1,k2]=grafo[i,j]#Con k1 y k2 delimito el cuadrado que voy a llenar y marco como ocupada el cuadro al rededor del punto segun el numero de celdas que desean
    
    print("Acabe de inflar el mapa\n")  
    #Voy a publicar el mapa inflado para observarlo, por ello debo poner otro mapa
    mapa_ocupacion = OccupancyGrid()
    mapa_ocupacion.header.seq=self.dato.header.seq
    mapa_ocupacion.header.stamp=self.dato.header.stamp
    mapa_ocupacion.header.frame_id=self.dato.header.frame_id
    mapa_ocupacion.info.resolution=self.dato.info.resolution
    mapa_ocupacion.header.seq=self.dato.header.seq
    mapa_ocupacion.info.width = self.dato.info.width
    mapa_ocupacion.info.height = self.dato.info.height
    mapa_ocupacion.info.origin.position.x =self.dato.info.origin.position.x
    mapa_ocupacion.info.origin.position.y =self.dato.info.origin.position.y
    mapa_ocupacion.info.origin.orientation.x =self.dato.info.origin.orientation.x
    mapa_ocupacion.info.origin.orientation.y =self.dato.info.origin.orientation.y
    mapa_ocupacion.info.origin.orientation.z =self.dato.info.origin.orientation.z
    mapa_ocupacion.info.origin.orientation.w =self.dato.info.origin.orientation.w
    mapa_ocupacion.data= np.ravel(np.reshape(mapa_inflado, (len(self.dato.data), 1)))
    pub_inflated.publish(mapa_ocupacion)
    
    
    return mapa_inflado
#El mapa de Inflado 1 infla solo los puntos que se encuentran al lado de espacio conocido, pero requiere muchas más operaciones
def mapa_inflado_1(self,num_celdas_inflar,grafo):
    print ("Calculando el mapa inflando " +str(num_celdas_inflar) + " cells\n")
    c, l=np.shape(grafo)
    mapa_inflado=np.copy(grafo)
    #Voy a inflar el mapa para poder obtener las rutas y que el robot se aleje de las paredes del mapa, entonces sobre el original cada vez que me encuentre 
    #Voy a inflar con 5 celdas
    for i in range(c):
        for j in range(l): 
            
            if i>=0 and i<(num_celdas_inflar+1) and j==0:#CASO 1
               
               if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,j]=100#Solo me inflo del lado de lo conocido
                

               if (grafo[i,j]+grafo[i+1,j])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[0:i,j]=100

            elif i>0 and i<(num_celdas_inflar+1) and j>0 and j<(num_celdas_inflar+1):#CASO 2
               

                if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,0:j]=100

                if (grafo[i,j]+grafo[i+1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[0:i,j]=100
   
            elif i>0 and i<(num_celdas_inflar+1) and j>=(num_celdas_inflar+1) and j<(l-(num_celdas_inflar+1)):#CASO 3


                if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100

                if (grafo[i,j]+grafo[i+1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[0:i,j]=100
                    
            elif i>0 and i<(num_celdas_inflar+1) and j>=(l-(num_celdas_inflar+1)) and j<(l-1):#CASO 4


                if (grafo[i,j]+grafo[i,j-1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100
                    else:
                        mapa_inflado[i,j:(l-1)]=100

                if (grafo[i,j]+grafo[i+1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[0:i,j]=100
                        
            elif i>=0 and i<(num_celdas_inflar+1) and j==l-1:#CASO 5
               
               if (grafo[i,j]+grafo[i,j-1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100
                    else:
                        mapa_inflado[i,j]=100#Solo me inflo del lado de lo conocido
                

               if (grafo[i,j]+grafo[i+1,j])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[0:i,j]=100         
                        
            elif i>=(num_celdas_inflar+1) and i<(c-num_celdas_inflar+1) and j==0:#CASO 6
               
               if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,j]=100#Solo me inflo del lado de lo conocido
                

               if (grafo[i,j]+grafo[i+1,j])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100

            elif i>=(num_celdas_inflar+1) and i<(c-num_celdas_inflar+1) and j>=(num_celdas_inflar+1) and j<(l-num_celdas_inflar+1):#CASO 7
               

                if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100

                if (grafo[i,j]+grafo[i+1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
  
            elif i>=(num_celdas_inflar+1) and i<(c-num_celdas_inflar+1) and j==(l-1):#CASO 8
               
               if (grafo[i,j]+grafo[i,j-1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100
                    else:
                        mapa_inflado[i,j]=100#Solo me inflo del lado de lo conocido
                

               if (grafo[i,j]+grafo[i+1,j])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
            
            elif i>=(c-(num_celdas_inflar+1)) and i<=(c-1) and j==0:#CASO 9


                if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,j]=100

                if (grafo[i,j]+grafo[i-1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
                    else:
                        mapa_inflado[i:(c-1),j]=100
                        
            elif i>=(c-(num_celdas_inflar+1)) and i<=(c-1) and j>0 and j<(num_celdas_inflar+1):#CASO 10
               

                if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,0:j]=100

                if (grafo[i,j]+grafo[i-1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
                    else:
                        mapa_inflado[i:(c-1),j]=100
      
            elif i>=(c-(num_celdas_inflar+1)) and i<=(c-1) and j>=(num_celdas_inflar+1) and j<(l-(num_celdas_inflar+1)):#CASO 11


                if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100

                if (grafo[i,j]+grafo[i-1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
                    else:
                        mapa_inflado[i:(c-1),j]=100
            
            elif i>=(c-(num_celdas_inflar+1)) and i<=(c-1) and j>=(l-(num_celdas_inflar+1)) and j<(l-1):#CASO 12


                if (grafo[i,j]+grafo[i,j-1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100
                    else:
                        mapa_inflado[i,j:(l-1)]=100

                if (grafo[i,j]+grafo[i-1,j])==101:

                    if grafo[i,j]==100:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
                    else:
                        mapa_inflado[i:(c-1),j]=100         
                
            elif i>=(c-(num_celdas_inflar+1)) and i<=(c-1) and j==(l-1):#CASO 13
               
               if (grafo[i,j]+grafo[i,j-1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100
                    else:
                        mapa_inflado[i,j]=100#Solo me inflo del lado de lo conocido
                

               if (grafo[i,j]+grafo[i-1,j])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
                    else:
                        mapa_inflado[i:(c-1),j]=100    
                        
            elif i>=(num_celdas_inflar+1) and i<(c-num_celdas_inflar+1) and j>0 and j<(num_celdas_inflar+1):#CASO 14
                
                if (grafo[i,j]+grafo[i,j+1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j:j+num_celdas_inflar]=100
                    else:
                        mapa_inflado[i,0:j]=100
                

                if (grafo[i,j]+grafo[i+1,j])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100
                    else:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100    
        
            elif i>=(num_celdas_inflar+1) and i<(c-num_celdas_inflar+1) and j>=(l-(num_celdas_inflar+1)) and j<(c-1):#CASO 15
                
                if (grafo[i,j]+grafo[i,j-1])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i,j-num_celdas_inflar:j]=100
                    else:
                        mapa_inflado[i,j:(l-1)]=100
                

                if (grafo[i,j]+grafo[i-1,j])==101:
                    
                    if grafo[i,j]==100:
                        mapa_inflado[i-num_celdas_inflar:i,j]=100
                    else:
                        mapa_inflado[i:i+num_celdas_inflar,j]=100  
                        
    print("Acabe de inflar el mapa\n")  
    #Voy a publicar el mapa inflado para observarlo, por ello debo poner otro mapa
    mapa_ocupacion = OccupancyGrid()
    mapa_ocupacion.header.seq=self.dato.header.seq
    mapa_ocupacion.header.stamp=self.dato.header.stamp
    mapa_ocupacion.header.frame_id=self.dato.header.frame_id
    mapa_ocupacion.info.resolution=self.dato.info.resolution
    mapa_ocupacion.header.seq=self.dato.header.seq
    mapa_ocupacion.info.width = self.dato.info.width
    mapa_ocupacion.info.height = self.dato.info.height
    mapa_ocupacion.info.origin.position.x =self.dato.info.origin.position.x
    mapa_ocupacion.info.origin.position.y =self.dato.info.origin.position.y
    mapa_ocupacion.info.origin.orientation.x =self.dato.info.origin.orientation.x
    mapa_ocupacion.info.origin.orientation.y =self.dato.info.origin.orientation.y
    mapa_ocupacion.info.origin.orientation.z =self.dato.info.origin.orientation.z
    mapa_ocupacion.info.origin.orientation.w =self.dato.info.origin.orientation.w
    mapa_ocupacion.data= np.ravel(np.reshape(mapa_inflado, (len(self.dato.data), 1)))
    pub_inflated.publish(mapa_ocupacion)
    return mapa_inflado

def mapa_de_costos(self,num_celdas_costo,grafo):
    
    print ("Caculando el mapa de costos con " +str(num_celdas_costo) + " celdas\n")
    cost_map = np.copy(grafo)
    [height, width] = grafo.shape
    
    for i in range(height):
        for j in range(width):
            if grafo[i,j]==100:#checamos para cada punto del mapa si esta ocupado 
                for k1 in range(-num_celdas_costo,num_celdas_costo+1):#Voy a ir de menos el radio hasta más el radio
                    for k2 in range(-num_celdas_costo,num_celdas_costo+1):#Aqui para inflar las columnas
                        cost=num_celdas_costo-max(abs(k1),abs(k2))#Se resta el radio de costo a la cordenada en valor absoluto mayor 
                        if cost>cost_map[i+k1,j+k2]:#Si el costo es mayor a el valor que tenia la celda entonces si la pongo, si no es así, se queda igual
                            cost_map[i+k1,j+k2]=cost+1#Con k1 y k2 delimito el cuadrado que voy a llenar y voy a tener que comparar para ver si sustituyo
    
    
                    
    return cost_map
          
def convertir_matriz(grafo):
    """#Entra un grafo de 40x40 que es subgrafo que envuelve al robot en un area cuadrada, en la cual se hara la busqueda de objetivos.
    #Este grafo fue modificado colocando como 1 los valores conocidos, esto porque se consideran grafos conectados, y a los demas valores se les ha colocado un 0.
    #Segun la imagen enviada, la matriz debe de ser transformada a una matriz que relaciones puntos en el mapa, ya que en la matriz original se relacionaban coordenadas x= Filas, y=Columnas
    #Obtengo la forma del grafo ya que el numero de columnas y filas de la nueva matriz esta en funcion de las filas y columnas de la matriz original.
    nuevo_grafo=np.zeros((c*c,l*l))#Este sera el grafo que se utilizara para el reacomodo de la matriz
    #Una vez se tiene la matriz se procedera a recorrer todas las filas (i) para cada columna, esto con la intension de reacomodar la matriz segun el algoritmo pensado.
    #Se encuentran las variables d, a, iz, b que representan los posibles casos en los cuales puede entrar el elemento que buscamos sus vecinos.
    #Al iniciar se cae en la primera condicion de cuando i=j=0, esto quiere decir que solamente existen 2 posibles vecinos, a la derecha (d) y debajo (b), las formulas se obtuvieron de acuerdo a la forma de convertir un punto especifico a el valor de la nueva matriz
    #Posteriormente tenemos a k, que representa la expresion con la cual podemos obtener el valor que tiene un punto especifico en las nuevas filas de mi matriz, como ejemplo el elemento (0,0) seria en la nueva matriz el elemento 0 de las filas.
    #El ciclo termina una vez se ha recorrido toda la matriz y remapeado la misma, cayendo en todos los casos, y se puede verificar con una matriz de 3*3 como se muestra en la imagen anexada.
    #Para saber si el "nodo" esta conectado, se evalua el producto de 2 elemento del grafo original: supongamos que estamos en (0,0), este elemento tiene 2 posibles conexiones, por lo cual se multiplicara el valor de (0,0)*(0,1) y (0,0)*(1,0) si el producto da 1, esto quiere decir que estaban conectados (ya que ambos son espacio conocido), en caso contrario sera 0"""
    c, l=np.shape(grafo)
    #Se crea un grafo no dirigido, esto debido a que la forma de crear el grafo hace que mi grafo caiga dentro de esta clasificación
    #Cuando creo una arista, entre dos puntos, automaticamente creo los nodos (en este caso puntos) y estos no se sobreescriben.
    nuevo_grafo=nx.Graph()
    for j in range(l):
        for i in range(c): 
            """d=(i*c)+(j+1)
            a=((i-1)*c)+j
            iz=(i*c)+(j-1)
            b=((i+1)*c)+j
            k=i+(j*l)"""
            if i==0 and j==0:
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i+1,j])==1 :
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

            elif i==0 and j>0 and j!=l-1:
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j+1])==1 :
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

            elif i==0 and j==l-1:
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

            elif i>0 and i !=c-1 and j==0:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])
                
            elif i>0 and i!=c-1 and j>0 and j!=l-1:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                if (grafo[i,j]*grafo[i,j+1])==1 :
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])

                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

            elif i>0 and i!=c-1 and j==l-1:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,b]=grafo[i,j]*grafo[i+1,j]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

                if (grafo[i,j]*grafo[i+1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i+1,j),weight=grafo[i,j]*grafo[i+1,j])
                
            elif i==c-1 and j==0:
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

            elif i==c-1 and j>0 and j!=l-1:
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                #nuevo_grafo[k,d]=grafo[i,j]*grafo[i,j+1]
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1:
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])

                if (grafo[i,j]*grafo[i,j+1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j+1),weight=grafo[i,j]*grafo[i,j+1])

            elif i==c-1 and j==l-1:
                #nuevo_grafo[k,iz]=grafo[i,j]*grafo[i,j-1]
                #nuevo_grafo[k,a]=grafo[i,j]*grafo[i-1,j]
                if (grafo[i,j]*grafo[i,j-1])==1:
                    nuevo_grafo.add_edge((i,j),(i,j-1),weight=grafo[i,j]*grafo[i,j-1])

                if (grafo[i,j]*grafo[i-1,j])==1 :
                    nuevo_grafo.add_edge((i,j),(i-1,j),weight=grafo[i,j]*grafo[i-1,j])
                

    return nuevo_grafo
#Busqueda de Objetivos encuentra los puntos frontera en el mapa inflado                
def Busqueda_Objetivos(self,numero_de_celdas,grafo):

    c, l=np.shape(grafo)
    nodos_objetivo=[]
    
    for j in range(l):
        for i in range(c): 
            
            if i==0 and j==0:
               
               if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))
                

               if (grafo[i,j]+grafo[i+1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

            elif i==0 and j>0 and j!=l-1:
               

                if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                if (grafo[i,j]+grafo[i+1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                if (grafo[i,j]+grafo[i,j-1])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

            elif i==0 and j==l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i+1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

            elif i>0 and i !=c-1 and j==0:
                
                if (grafo[i,j]+grafo[i-1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))

                    else:
                        nodos_objetivo.append((i-1,j))

                if (grafo[i,j]+grafo[i+1,j])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                if (grafo[i,j]+grafo[i,j+1])==1:

                   if grafo[i,j]==1:

                        nodos_objetivo.append((i,j))
                   else:
                        nodos_objetivo.append((i,j+1))
                
            elif i>0 and i!=c-1 and j>0 and j!=l-1:
                
                if (grafo[i,j]+grafo[i,j+1])==1:

                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                if (grafo[i,j]+grafo[i+1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))

                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

            elif i>0 and i!=c-1 and j==l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                   
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

                if (grafo[i,j]+grafo[i+1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i+1,j))
                
            elif i==c-1 and j==0:
                
                if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

            elif i==c-1 and j>0 and j!=l-1:
                
                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))

                if (grafo[i,j]+grafo[i,j+1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j+1))

            elif i==c-1 and j==l-1:
               
                if (grafo[i,j]+grafo[i,j-1])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i,j-1))

                if (grafo[i,j]+grafo[i-1,j])==1:
                    
                    if grafo[i,j]==1:
                        nodos_objetivo.append((i,j))
                    else:
                        nodos_objetivo.append((i-1,j))


    print("Ya termine de calcular los puntos objetivo\n")
    print("Encontre un total de "+str(len(nodos_objetivo))+" candidatos\n")
    return nodos_objetivo      
#Me permite visualizar el número de puntos objetivos encontrados en el mapa a explorar
def Filtrado_de_objetivos(objetivos,mapa_de_costos):
    costos=[]
    for [x,y] in objetivos:
        heapq.heappush(costos,(mapa_de_costos[x,y],(x,y)))#Recorro todos los puntos y los voy colocando de menor a mayor costo
    
    
    return costos
     
def visualizacion_objetivos(self,objetivos,mapa):
    
    
    pub =rospy.Publisher('/visualization_marker',Marker, queue_size=100)
    puntos=Marker(ns="puntos_objetivo",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
    puntos.header.stamp=rospy.Time()
    puntos.header.frame_id="/map"
    puntos.pose.orientation.w=1.0
    #Esta es la posicion de la marca
    puntos.pose.position.x=mapa.info.origin.position.x+0.025 #La referencia se encuentra en la esquina inferior derecha se se ve el robot avanzando hacia enfrente, en el punto más alejado verde en una esquina, y se encuentra a -10.275 positivos en ambos ejes
    puntos.pose.position.y=mapa.info.origin.position.y+0.025
    puntos.pose.position.z=0
    #Puntos
    puntos.scale.x=0.1#Tamaño de los puntos
    puntos.scale.y=0.1
    puntos.scale.z=0.1
    #Los puntos seran verdes
    puntos.color.g=1.0#Color 
    puntos.color.a=1.0#Nitidez

    
    o=Point()
    o.x=self.pos_x_robot*mapa.info.resolution
    o.y=self.pos_y_robot*mapa.info.resolution
    o.z=0
    puntos.points.append(o)
    
    
    for (x,y) in objetivos:
        p=Point()
        
        p.x=y*(mapa.info.resolution)#Se hizo una regla de tres o se multiplico por la resolución del mapa para ajustar los valores de la matriz a los valores de 
        p.y=x*(mapa.info.resolution)
        p.z=0
        puntos.points.append(p)
    
    rate=rospy.Rate(20)
    
    pub.publish(puntos)
    rate.sleep()
    
def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map):
    #
    # Write a A* algorithm to find a path in an occupancy grid map given the start cell
    # [start_r, start_c], the goal cell [goal_r, goal_c] and the map 'grid_map'.
    # Return the set of points of the form [[start_r, start_c], [r1,c1], [r2,c2], ..., [goal_r, goal_c]]
    # If path cannot be found, return an empty tuple []
    # Use Manhattan distance as heuristic function
    # Hint: Use a priority queue to implement the open list
    # Documentation to implement priority queues in python can be found in
    # https://docs.python.org/2/library/heapq.html
    #
    g_values=np.full(np.shape(grid_map),sys.maxint)#Creo un arreglo para almacenar todas las g del mapa y lo lleno de valores muy grandes inicialmente para poder comparar e ir agregando las g, en este caso me sirve para poder ir llenando las g de los primeros puntos
    f_values=np.full(np.shape(grid_map),sys.maxint)#Creo un arreglo para almacenar todas las f del mapa y lo lleno de valores muy grandes inicialmente para poder comparar e ir agregando las f, en este caso me sirve para poder ir llenando las f de los primeros puntos
    p_nodes=np.full((np.shape(grid_map)[0],np.shape(grid_map)[1],2),-1)#Creo la lista de puntos anteriores para cada nodo con 2 dimensiones y la tercera con un valor de 2 ya que no lo necesitaremos, y los inicio con un valor de -1 para saber el limite
    in_open_list=np.full(np.shape(grid_map),False)#Estoy creando los verificadores de si esta en la lista cerrada o abierta y los llena de puros Falsos
    in_close_list=np.full(np.shape(grid_map),False)#Lista de puntos visitados
    
    open_list=[]#lista de puntos candidatos
    heapq.heappush(open_list,(0,[start_r,start_c]))#Agrego a la lista en la posicion 0 el punto inicial
    g_values[start_r,start_c]=0
    f_values[start_r,start_c]=0
    in_open_list[start_r,start_c]=True
    [r,c]=[start_r,start_c]#r,c sera el nodo actual, iniciando por la posicion del robot
    steps=0
    path=[]
    while len(open_list)>0 and [r,c]!=[goal_r,goal_c]:
        [r,c]=heapq.heappop(open_list)[1]#Estoy sacando el elemento con mayor prioridad y como me regresa la prioridad y el elemento entonces me interesa saolo sacar el elemento que es el 2 del arreglo
        in_close_list[r,c]=True#Decimos que ya visitamos el nodo que acabo de extraer 
        neighbors=[[r+1,c],[r-1,c],[r,c+1],[r,c-1]]#Sacamos los vecinos del punto actual
        for [nr,nc]in neighbors:#Para cada uno de los vecinos
            if grid_map[nr,nc]!=0 or in_close_list[nr,nc]:#Si el nodo vecino esta ocupado=100 o es desconocido=-1 o ya fue visitado se omite
                continue
            g=g_values[r,c]+1+cost_map[r,c]#Genero la g del punto anterios más 1, en este caso no es g=0 porque lo llene con un valor muy grande, pero no importa, ya que tiene el mismo efecto que empezar con g=0 y le agrego ademas una función de costos que es la distancia que hay de puntos a obstaculos ya que no quiero que el robot se acerque a obstaculos
            f=g+(np.abs(goal_r-nr)+np.abs(goal_c-nc))#Calculo la f con la g y la distancia de manhatan
            if f<f_values[nr,nc]:#Si el costo del nodo vecino de r,c es menor al costo del nodo vecino nr, nc entonces cambio la g del nodo vecino, ya que con ello digo que al nodo vecino llegue por otra ruta mejor
                g_values[nr,nc]=g #Aqui lo que hago es que yo llegue por una ruta al punto nr, nc y si la g fue menor que la g que ya se tenia, tal vez por otra ruta, eso quiere decir que la ruta nueva es mejor y debo cambiar esa g
                f_values[nr,nc]=f #Aqui lo que hago es que yo llegue por una ruta al punto nr, nc y si la g fue menor que la g que ya se tenia, tal vez por otra ruta, eso quiere decir que la ruta nueva es mejor y debo cambiar esa g
                p_nodes[nr,nc]=[r,c]#Aquí estoy agregando el predecesor del vecino si se encuentra que el costo es menor que el costo ya calculado por otra ruta tal vez
            if not in_open_list[nr,nc]: #Si el punto candidato no esta en la lista abierta entonces le cambio el valor, con esto podria agregar dicho punto a la lista abierta para buscar en ellos
                in_open_list[nr,nc]=True
                heapq.heappush(open_list,(f,[nr,nc]))#Agrego el punto candidato a la lista abierta si es que no estaba ya y le agrego la g para ordenar de menor g a mayor g e irlos sacando en ese orden
                
            steps+=1#Aumento el número de pasos que le tomo al algoritmo calcular la ruta y vuelvo a hacer lo mismo para cada punto de la lista abierta hasta terminar
       
    if[r,c]!=[goal_r,goal_c]:#Una vez checo para todos losn nodos, si el valor de r,c no es igual al objetivo es que no hay solucion
        print("Cannot calculate path by Dijsktra :c")
        return[]
    
    #Aqui r, c ya se quedo con los valores meta, por lo cual el punto anterior
    #Preguntar sobre p_nodes
    while [p_nodes[r,c][0],p_nodes[r,c][1]]!=[-1,-1]:#Sí si existe la solución entonces hasta encontrar el nodo [-1,-1] que fue con lo que se lleno y me dice que son todos los puntos
        path.insert(0,[r,c])#Voy agregando en la posicion cero cada una de las coordenadas que es el punto anterior
        [r,c]=p_nodes[r,c]
    print("The path has had "+str(steps)+" steps for been calculated")
    return path    
   
