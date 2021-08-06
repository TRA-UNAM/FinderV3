#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda
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
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import copy
import math
import time

    
def a_star(start_r, start_c, goal_r, goal_c, grid_map, cost_map,self):
    
    start_r=int(start_r)
    start_c=int(start_c)
    grid_map=np.array(grid_map).reshape((self.dato.height, self.dato.width))
    cost_map=np.array( cost_map).reshape((self.dato.height, self.dato.width))
    g_values=np.full(np.shape(grid_map),sys.maxsize)#Creo un arreglo para almacenar todas las g del mapa y lo lleno de valores muy grandes inicialmente para poder comparar e ir agregando las g, en este caso me sirve para poder ir llenando las g de los primeros puntos
    f_values=np.full(np.shape(grid_map),sys.maxsize)#Creo un arreglo para almacenar todas las f del mapa y lo lleno de valores muy grandes inicialmente para poder comparar e ir agregando las f, en este caso me sirve para poder ir llenando las f de los primeros puntos
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
        for [nr,nc] in neighbors:#Para cada uno de los vecinos
            
            if grid_map[nr,nc]!=0 or in_close_list[nr,nc]:#Si el nodo vecino esta ocupado=100 o es desconocido=-1 o ya fue visitado se omite
                continue
            g=g_values[r,c]+1+cost_map[r,c]#Genero la g del punto anterios más 1, en este caso no es g=0 porque lo llene con un valor muy grande, pero no importa, ya que tiene el mismo efecto que empezar con g=0 y le agrego ademas una función de costos que es la distancia que hay de puntos a obstaculos ya que no quiero que el robot se acerque a obstaculos
            f=g+(np.abs(goal_r-nr)+np.abs(goal_c-nc))#Calculo la f con la g y la distancia de manhatan
            if g<g_values[nr,nc]:#Si el costo del nodo vecino de r,c es menor al costo del nodo vecino nr, nc entonces cambio la g del nodo vecino, ya que con ello digo que al nodo vecino llegue por otra ruta mejor
                g_values[nr,nc]=g #Aqui lo que hago es que yo llegue por una ruta al punto nr, nc y si la g fue menor que la g que ya se tenia, tal vez por otra ruta, eso quiere decir que la ruta nueva es mejor y debo cambiar esa g
                f_values[nr,nc]=f #Aqui lo que hago es que yo llegue por una ruta al punto nr, nc y si la g fue menor que la g que ya se tenia, tal vez por otra ruta, eso quiere decir que la ruta nueva es mejor y debo cambiar esa g
                p_nodes[nr,nc]=[r,c]#Aquí estoy agregando el predecesor del vecino si se encuentra que el costo es menor que el costo ya calculado por otra ruta tal vez
            if not in_open_list[nr,nc]: #Si el punto candidato no esta en la lista abierta entonces le cambio el valor, con esto podria agregar dicho punto a la lista abierta para buscar en ellos
                in_open_list[nr,nc]=True
                heapq.heappush(open_list,(f,[nr,nc]))#Agrego el punto candidato a la lista abierta si es que no estaba ya y le agrego la g para ordenar de menor g a mayor g e irlos sacando en ese orden
                
            steps+=1#Aumento el número de pasos que le tomo al algoritmo calcular la ruta y vuelvo a hacer lo mismo para cada punto de la lista abierta hasta terminar

     
    if[r,c]!=[goal_r,goal_c] or steps<=500:#Una vez checo para todos losn nodos, si el valor de r,c no es igual al objetivo es que no hay solucion
        print("Cannot calculate path by A* :c")
        return[]
        
    if steps>500:
    #Aqui r, c ya se quedo con los valores meta, por lo cual el punto anterior
    #Preguntar sobre p_nodes
        while [p_nodes[r,c][0],p_nodes[r,c][1]]!=[-1,-1]:#Sí si existe la solución entonces hasta encontrar el nodo [-1,-1] que fue con lo que se lleno y me dice que son todos los puntos
            path.insert(0,[r,c])#Voy agregando en la posicion cero cada una de las coordenadas que es el punto anterior
            [r,c]=p_nodes[r,c]
            
        
    print("El camino necesito de "+str(steps)+" pasos para ser calculado")
    

    return path

def get_smooth_path(original_path, alpha, beta):
    

    smooth_path  = copy.deepcopy(original_path)            # At the beginnig, the smooth path is the same than the original path.
    tolerance    = 0.00001                                 # If gradient magnitude is less than a tolerance, we consider.
    gradient_mag = tolerance + 1                           # we have reached the local minimum.
    gradient     = [[0,0] for i in range(len(smooth_path))]# Gradient has N components of the form [x,y]. 
    epsilon      = 0.5   
    
    #print("Smoothing path with "+str(len(smooth_path))+ " points, using " +str(alpha)+ " as alpha and " +str(beta)+ " as beta")                                  # This variable will weight the calculated gradient.
    while gradient_mag>tolerance:#En este caso es decir que el gradiente o derivada sea mayor que casi 0, hasta que se tengo un valor menor o igual a la tolerancia se rompe el while
        gradient_mag=0
        [xi,yi]=smooth_path[0]#El punto 0
        [xn,yn]=smooth_path[1]#El punto sigueinte o 1
        [xq,yq]=original_path[0]#El punto del original en cero
        gx=alpha*(xi-xn)+beta*(xi-xq)#Este es el gradiente para el primer elemento
        gy=alpha*(yi-yn)+beta*(yi-yq)#Este es el gradiente para el primer elemento
        [xi,yi]=[xi-epsilon*gx,yi-epsilon*gy]#actualizo el punto
        smooth_path[0]=[xi,yi]#Para el primer punto
        gradient_mag+=gx**2+gy**2 #Aumento el gradiente
        for i in range(1,len(smooth_path)-1):#Voy a empezar en i=1 hasta el final porque las listas tienen len()-1
            [xp,yp]=smooth_path[i-1]#El punto anterior
            [xi,yi]=smooth_path[i]#Obtengo el punto de la ruta
            [xn,yn]=smooth_path[i+1]#El punto siguiente, no marca error porque el [1,len(smooth_path)-1) agarra el primero, pero el ultimo lo excluye o es el anterior a él el limite
            [xq,yq]=original_path[i]#Valor del mapa original
            gx=alpha*(2*xi-xp-xn)+beta*(xi-xq)#Este es el gradiente o la derivada
            gy=alpha*(2*yi-yp-yn)+beta*(yi-yq)
            [xi,yi]=[xi-epsilon*gx,yi-epsilon*gy]#actualizo el punto
            smooth_path[i]=[xi,yi]
            gradient_mag+=gx**2+gy**2 #Aumento el gradiente
        
        #Para el ultimo punto se calcula el gradiente
        [xi,yi]=smooth_path[-1]#El ultimo
        [xp,yp]=smooth_path[-2]#El penultimo
        [xq,yq]=original_path[-1]
        gx=alpha*(xi-xp)+beta*(xi-xq)#Este es el gradiepte para el primer elemento
        gy=alpha*(yi-yp)+beta*(yi-yq)#Este es el gradiente para el primer elemento
        [xi,yi]=[xi-epsilon*gx,yi-epsilon*gy]#actualizo el punto
        smooth_path[-1]=[xi,yi]#Para el primer punto
        gradient_mag+=gx**2+gy**2
        gradient_mag=math.sqrt(gradient_mag)


    
    return smooth_path

    
