#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from exploration.srv import Puntos_Objetivo,Puntos_ObjetivoResponse
import numpy as np
from sklearn.cluster import KMeans


class Servicio:


   
    def handle(self,req):
        puntos=[]
        wcss=[]
        for i in range(len(req.coord_x)):
            puntos.append([req.coord_x[i],req.coord_y[i]])

        np.asarray(puntos)


        for i in range(1, 5):#Los 5 representa que como maximo tendria 4 grupos
            kmeans = KMeans(n_clusters = i, init = "k-means++", max_iter = 300, n_init = 10, random_state = 0)
            kmeans.fit(puntos)
            wcss.append(kmeans.inertia_)#El valor de wcss para cada uno de los k

        wcss_resta=[]
        for i in range(3):
            wcss_resta.append(wcss[i]-wcss[i+1])

        #A través de los valores de wcss voy a obtener el numero optimo de clusters
        for i in range(len(wcss_resta)):
            if np.mean(wcss_resta)< wcss_resta[i]:
                pass
            else:
                k=i
        
        kmeans = KMeans(n_clusters = k, init="k-means++", max_iter = 300, n_init = 10, random_state = 0)
        y_kmeans = kmeans.fit_predict(puntos)
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
    