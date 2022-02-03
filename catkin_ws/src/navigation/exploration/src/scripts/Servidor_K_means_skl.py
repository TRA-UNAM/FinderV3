#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from exploration.srv import Puntos_Objetivo,Puntos_ObjetivoResponse
import numpy as np
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt



class Servicio:


   
    def handle(self,req):
        puntos=[]
        wcss=[]
        for i in range(len(req.coord_x)):
            puntos.append([req.coord_x[i],req.coord_y[i]])

        np.asarray(puntos)


        for i in range(1, 9):#Los 9 representa que como maximo tendria 8 grupos
            kmeans = KMeans(n_clusters = i, init = "k-means++", max_iter = 300, n_init = 10, random_state = 0)#n_init representa el numero de veces que el algoritmo de k-means correra con diferente semilla de centroide diferentes. Los resultados finales seran la mejor salida de ejecuciones ejecutivas en terminos de inercia.
            kmeans.fit(puntos)
            wcss.append(kmeans.inertia_)#kmeans.inertia_ calcula la suma de los cuadrados de las distancias que existe entre los vectores de cada cluster a su centroide
        #wcss contiene dichas sumas, y entre mas clusters tengamos, menor seran esas distancias.
        #plt.plot([1,2,3,4,5,6,7,8],wcss)
        #plt.show()
        
        
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
        
        kmeans = KMeans(n_clusters = k, init="k-means++", max_iter = 300, n_init = 10, random_state = 0)
        kmeans.fit(puntos)
        #y_kmeans = kmeans.fit_predict(puntos)
        centroides_x=kmeans.cluster_centers_[:,0]
        centroides_y=kmeans.cluster_centers_[:,1]
        print("Ya termine de obtener los centroides\n")
        return Puntos_ObjetivoResponse(centroides_x=centroides_x,centroides_y=centroides_y,k=k)
        

    def Puntos_Objetivo(self):
            
        rospy.Service('/servicio_centroides', Puntos_Objetivo, self.handle)
        print("Listo para obtener los puntos objetivo")
            
    

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('Servidor_Puntos_Objetivo')
    servicio=Servicio()
    servicio.Puntos_Objetivo()
    rospy.spin()
    