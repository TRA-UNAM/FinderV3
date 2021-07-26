#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import numpy as np

def inicializar_centroides(objetivos,k=2):
    
    
    (centroid_min_x,centroid_min_y)=(min(objetivos[:][0]),min(objetivos[:][1]))
    (centroid_max_x,centroid_max_y)=(max(objetivos[:][0]),max(objetivos[:][1]))
    
    centroides=[]

    for centroide in range (k):#Para el número de clusters voy a adquirir un centroide para cada uno
        centroide_x=np.random.uniform(centroid_min_x,centroid_max_x)#Adquiero un número random entre el valor minimo y máximo de x y de y abajo
        centroide_y=np.random.uniform(centroid_min_y,centroid_max_y)
        centroid=(int(centroide_x),int(centroide_y))#Los convierto a enteros y los aligno para agregarlos a mi lista de centroides
        centroides.append(centroid)
    
    return centroides

def suma_del_error_al_cuadrado(objetivos, centroides):

    grupo_1=[]
    grupo_2=[]
    grupos=[]
    if len(centroides)>0:#Por el momento se contemplan 2 clusters, si se desean más entonces agregar más errores
        for i in range(len(centroides)-1):
            for j in objetivos:

                error_1=np.square(((j[0]-centroides[i][0])**2)+((j[1]-centroides[i][1])**2))
                error_2=np.square(((j[0]-centroides[i+1][0])**2)+((j[1]-centroides[i+1][1])**2))
                
                if error_1<error_2:
                    grupo_1.append([j[0],j[1]])
                else:
                    grupo_2.append([j[0],j[1]])
    else:
        pass

    grupos.append(grupo_1)
    grupos.append(grupo_2)

    return grupos

def actualizar_centroide(grupos,k=2):

    centroides=[] 
    
    
    for l in range(k):#Para el número de clusters voy a adquirir un centroide para cada uno
        centroides_totales=np.mean(grupos[l][:],axis=0)
        if np.shape(centroides_totales)==(2,):
            centroide_x=centroides_totales[0]
            centroide_y=centroides_totales[1]
            centroid=(int(centroide_x),int(centroide_y))#Los convierto a enteros y los aligno para agregarlos a mi lista de centroides
            centroides.append(centroid)
    
    
    
    
    return centroides


def k_means(objetivos):
    
    centroides_ref=(0,0)
    centroides=inicializar_centroides(objetivos)
    while centroides_ref!=centroides:
        centroides_ref=centroides
        grupos=suma_del_error_al_cuadrado(objetivos,centroides)
        if len(grupos)!=0:
            centroides=actualizar_centroide(grupos)

        else:
            centroides=[]
    
    return centroides
