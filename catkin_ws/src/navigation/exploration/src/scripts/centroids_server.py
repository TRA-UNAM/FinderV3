#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import rospy
from exploration.srv import GetBoundaryPoints,GetBoundaryPointsResponse
import numpy as np
from sklearn.cluster import KMeans
from geometry_msgs.msg import Point
import math



class Server:

    def __init__(self):
        
        self.boundary_points=[]
        
        


    def GetBoundaryPoints(self,req):
        
            #----------------Boundary Points Client--------------------------------
            print("Establishing the connection with Boundary Points Server")
            rospy.wait_for_service('/navigation/mapping/get_boundary_points')#Espero hasta que el servicio este habilitado
            try:
                
                client_boundary_points=rospy.ServiceProxy('/navigation/mapping/get_boundary_points',GetBoundaryPoints)#Creo un handler para poder llamar al servicio
                data_bp=client_boundary_points(map=req.map)
                self.boundary_points=data_bp.points
        
            except rospy.ServiceException as e:
                print("The request for the boundary points server failed: %s"%e)

            
            print("Already we get the data related with the GetBoundaryPoints service\n")
            print("The number of Boundary Points founded are "+str(len(self.boundary_points))+"\n")


   
    def handle_GetCentroids(self,req):

        
        self.GetBoundaryPoints(req)
        points=[]
        std_dev=[]
        for i in range(len(self.boundary_points)):
            points.append([self.boundary_points[i].x,self.boundary_points[i].y])

        np.asarray(points)


        for i in range(1, 9):#Los 9 representa que como maximo tendria 8 grupos
            kmeans = KMeans(n_clusters = i, init = "k-means++", max_iter = 300, n_init = 10, random_state = 0)#n_init representa el numero de veces que el algoritmo de k-means correra con diferente semilla de centroide diferentes. Los resultados finales seran la mejor salida de ejecuciones ejecutivas en terminos de inercia.
            kmeans.fit(points)
            std_dev.append(math.sqrt(kmeans.inertia_/len(points)))#kmeans.inertia_ calcula la suma de los cuadrados de las distancias que existe entre los vectores de cada cluster a su centroide y al dividirlo entre n y sacar raiz obtengo la std_devandar de los datos
        #wcss contiene dichas sumas, y entre mas clusters tengamos, menor seran esas distancias.
        
        for i in range(8):#Busco tener clusters cercanos a 1 m en su tamaño promedio
            
            if std_dev[i]>1:
                k=i+2
            

        kmeans = KMeans(n_clusters = k, init="k-means++", max_iter = 300, n_init = 10, random_state = 0)
        kmeans.fit(points)
        centroids_x=kmeans.cluster_centers_[:,0]
        centroids_y=kmeans.cluster_centers_[:,1]
        centroids=[]
        for i in range(len(centroids_x)):
            p=Point()
            p.x=centroids_x[i]
            p.y=centroids_y[i]
            p.z=0
            centroids.append(p)
        print("We already get the centroids\n")
        return GetBoundaryPointsResponse(points=centroids,k=k)
        

    def GetCentroids(self):
            
        rospy.Service('/navigation/mapping/get_boundary_points_clustered', GetBoundaryPoints, self.handle_GetCentroids)
        print("The Centroids Server is ready for the request")
            
    

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('centroids_server')
    server=Server()
    server.GetCentroids()
    rospy.spin()
    