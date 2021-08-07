#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import numpy as np
from copy import copy

def inicializar_centroides(objetivos,k=2):
    
    
    (centroid_min_x,centroid_min_y)=(min(objetivos[:][0]),min(objetivos[:][1]))
    (centroid_max_x,centroid_max_y)=(max(objetivos[:][0]),max(objetivos[:][1]))
    centroides=[]
    

    for i in range (k):#Para el número de clusters voy a adquirir un centroide para cada uno
        centroide_x=np.random.uniform(centroid_min_x,centroid_max_x)#Adquiero un número random entre el valor minimo y máximo de x y de y abajo
        centroide_y=np.random.uniform(centroid_min_y,centroid_max_y)
        centroid=(int(centroide_x),int(centroide_y))#Los convierto a enteros y los asigno para agregarlos a mi lista de centroides
        centroides.append(centroid)
    
    return centroides

def agrupar(objetivos, centroides):

    grupo_1=[]
    grupo_2=[]
    grupos=[]
    
    for i in range(len(centroides)-1):
        for j in objetivos:
        
            error_1=np.square(((j[0]-centroides[i][0])**2)+((j[1]-centroides[i][1])**2))
            
            error_2=np.square(((j[0]-centroides[i+1][0])**2)+((j[1]-centroides[i+1][1])**2))
        

            if error_1<error_2:
                grupo_1.append([j[0],j[1]])
            else:
                grupo_2.append([j[0],j[1]])

    #print(len(grupo_1))
    #print(len(grupo_2))
    
    grupos.append(grupo_1)
    grupos.append(grupo_2)

    return grupos

def actualizar_centroide(grupos,k):

    centroides_nuevos=[] 
    
    if k==2:
        for l in range(k):#Para el número de clusters voy a adquirir un centroide para cada uno
            centroides_totales=np.mean(grupos[l][:],axis=0)
            centroide_x=centroides_totales[0]
            centroide_y=centroides_totales[1]
            centroid=(int(centroide_x),int(centroide_y))#Los convierto a enteros y los aligno para agregarlos a mi lista de centroides
            centroides_nuevos.append(centroid)
    else:
        if len(grupos[0])!=0:
        
            centroides_totales=np.mean(grupos[0][:],axis=0)
            centroide_x=centroides_totales[0]
            centroide_y=centroides_totales[1]
            centroid=(int(centroide_x),int(centroide_y))#Los convierto a enteros y los aligno para agregarlos a mi lista de centroides
            centroides_nuevos.append(centroid)
        else:
            centroides_totales=np.mean(grupos[1][:],axis=0)
            centroide_x=centroides_totales[0]
            centroide_y=centroides_totales[1]
            centroid=(int(centroide_x),int(centroide_y))#Los convierto a enteros y los aligno para agregarlos a mi lista de centroides
            centroides_nuevos.append(centroid)
    
    
    return centroides_nuevos


def k_means(objetivos):
    
    centroides=inicializar_centroides(objetivos)
    
    centroides_ref_1=(0,0)
    
    
    while centroides_ref_1!=centroides[0]:
        centroides_ref_1=copy(centroides[0])
        grupos=agrupar(objetivos,centroides)
        if len(grupos[0])>0 and len(grupos[1])>0:
            centroides=actualizar_centroide(grupos,k=2)

        elif len(grupos[0])>0 and len(grupos[1])==0:
            centroides=actualizar_centroide(grupos,k=1)

        elif len(grupos[1])>0 and len(grupos[0])==0:
            centroides=actualizar_centroide(grupos,k=1)

    
    return centroides
