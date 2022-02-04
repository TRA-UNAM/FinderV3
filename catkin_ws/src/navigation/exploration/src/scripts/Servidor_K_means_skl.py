#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import Centroides,CentroidesResponse
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import math



class Servicio:


   
    def handle(self,req):
        puntos=[]
        desv_est=[]
        for i in range(len(req.coord_x)):
            puntos.append([req.coord_x[i],req.coord_y[i]])

        np.asarray(puntos)


        for i in range(1, 9):#Los 9 representa que como maximo tendria 8 grupos
            kmeans = KMeans(n_clusters = i, init = "k-means++", max_iter = 300, n_init = 10, random_state = 0)#n_init representa el numero de veces que el algoritmo de k-means correra con diferente semilla de centroide diferentes. Los resultados finales seran la mejor salida de ejecuciones ejecutivas en terminos de inercia.
            kmeans.fit(puntos)
            desv_est.append(math.sqrt(kmeans.inertia_/len(puntos)))#kmeans.inertia_ calcula la suma de los cuadrados de las distancias que existe entre los vectores de cada cluster a su centroide y al dividirlo entre n y sacar raiz obtengo la desv_estandar de los datos
        #wcss contiene dichas sumas, y entre mas clusters tengamos, menor seran esas distancias.
        #plt.plot([1,2,3,4,5,6,7,8],wcss)
        #plt.show()
        
        """
        wcss_resta=[]
        for i in range(7):
            wcss_resta.append(wcss[i]-wcss[i+1])#Obtengo las diferencias en el valor de wcss que se da cuando se aumenta en 1 el numero de clusters

        desv_est=np.std(wcss)
        #print(wcss_resta,desv_est)
        #dato=int(input())
        
        

        #A traves de los valores de wcss voy a obtener el numero optimo de clusters
        for i in range(len(wcss_resta)):
            if wcss_resta[i]>desv_est:#Si el cambio en el valor de wcss es mayor a la desviacion estandar de wcss entonces lo dejo pasar
                k=i+2
            else:
                break
        """
        #Voy a obtener la desviacion estandar y voy a controlar el tamano de los clusters, en el momento en que el numero de clusters sea menor a 1 metro entonces ese el valor anterior sera el valor de k idoneo
        #Que la distancia promedio entre cada punto y su centroide sea mayor o igual a 1 metro
        for i in range(8):
            
            if desv_est[i]>1:
                k=i+2
            
            


        kmeans = KMeans(n_clusters = k, init="k-means++", max_iter = 300, n_init = 10, random_state = 0)
        kmeans.fit(puntos)
        #y_kmeans = kmeans.fit_predict(puntos)
        centroides_x=kmeans.cluster_centers_[:,0]
        centroides_y=kmeans.cluster_centers_[:,1]
        print("Ya termine de obtener los centroides\n")
        return CentroidesResponse(centroides_x=centroides_x,centroides_y=centroides_y,k=k)
        

    def Centroides(self):
            
        rospy.Service('/servicio_centroides', Centroides, self.handle)
        print("Listo para obtener los centroides")
            
    

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Puntos_Objetivo')
    servicio=Servicio()
    servicio.Centroides()
    rospy.spin()
    