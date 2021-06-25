#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Msoqueda
import rospy
#from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import Metodos_de_navegacion_autonoma as md
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import heapq
import math
import cv2



class Nodo:
    
    def __init__(self,twist):
        self.init_node=rospy.init_node("Exploration_and_Mapping", anonymous=True)
        self.twist= twist
        self.dato=0
        self.mapa=0
        self.mapa_inflado=0
        self.celdas_a_inflar=5#número de celdas a inflar, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.celdas_de_costo=5#número de celdas de costo desde el obstaculo, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.objetivos=[]
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.costos=0
        self.mapa_de_costos=0
        self.costos_rutas=0

    def posicion_robot_callback(self,data):
            #Obtendre la posicion del laser a partir de la posicion de base_footprint
            angulo_robot=2*math.atan2(data.pose.pose.orientation.z,data.pose.pose.orientation.w)#Se trata de un cuaternion, por ello debo de hacer esto con dichas componentes
            #pub =rospy.Publisher('/visualization_marker',Marker, queue_size=50)
            correcion_x=0.7*math.cos(angulo_robot)
            correcion_y=0.7*math.sin(angulo_robot)
            #print(correcion_x,correcion_y,data.pose.pose.orientation.z)
            self.pos_y_robot=(self.dato.info.width/2)+(correcion_y/self.dato.info.resolution)+((data.pose.pose.position.y)/self.dato.info.resolution)#Estan intercambiados en el mapa, por eso necesito invertirlos
            self.pos_x_robot=(self.dato.info.height/2)+(correcion_x/self.dato.info.resolution)+((data.pose.pose.position.x)/self.dato.info.resolution)#Se ajusta y se suman 4 metros para que este justo en el laser

    #Se obtiene un arreglo bidimensional que se transforma en un arreglo bidimensional, en donde las columnas corresponden a las coordenadas en x en el mapa de rviz
    #Mientras que las filas corresponden a las coordenadas en y. Una vez obtenido el mapa, se procede a alterar los valores de la matriz, con el fin de prepararlo tanto para la busqueda de objetivos, como para la evaluacion de distancias
    #Se cambian los valores conocidos (originalmente 0) a valores de 1
    #Mientras que los valores de espacio desconocido (valor de -1) se colocan en 0
    #Una vez se tiene esto, se procede a la busqueda de objetivos guiandonos con la posicion del elemento de la matriz, con ello determinamos el numero de nodos alrededor del actual que tenemos que buscar (se debe pulir ya que algunos puntos se repiten, debido a que no se gueardan los valores que ya se compararon, y ademas son demasiados puntos los que se analizan)
    #Una vez obtenido los puntos, se envian al metodo que los graficara, los puntos estan en coordenadas (x,y) y los valores de los puntos se encuentran invertidos (por lo mismo que se comento con los sistemas de referencia en rviz)    
    #Los puntos necesitan multiplicarse por el factor de escala que es 0.05 aprocimadamente (ya que es la conversion necesaria para pasar de puntos de la matriz a puntos en el mapa)
    #Posteriormente se les resta a estos puntos 10.27 aproximadamente, esto debido a que el origen se encuentra justo en el medio del mapa, y para situarnos en el 0 de la matriz, necesitamos recorrernos -10.27 en ambos ejes (ese valor se obtiene de datos del mapa, y se d¿hace un pequeño ajuste ya que los puntos se ven un poco desfasados)
    #Se publican a una frecuencia de 15 por segundo y en principio despues de esto vendria el filtrado de puntos objetivos
    
    
    
    def map_callback(self,dato):
        #Estoy reacomodando el arreglo para poder manejarlo como una matriz, basandome en la altura y ancho
        i=0
        
        self.dato=dato
        
        #-------------------------------- Estoy preparando una matriz, a partir de todos los puntos que me entrega rviz como arreglo, el elemento 0,0 se encuentra u origen se encuentra en la esquina superior izq, desde ahi, rviz empieza el arreglo que obtenemos en /map
        #Sin embargo, el sistema de referencia del mapa se encuentra justo en el medio, por lo cual debemos de hacer un ajuste para movernos en esta matriz, seria a cada punto restarle la posicion original obtenida del topico /map en origin/posicion y multiplicar los indices por el factor de escala de 0.05
        self.mapa=np.array(self.dato.data).reshape((self.dato.info.height, self.dato.info.width))
        #Las filas de la matriz corresponden a las coordenadas y
        #Las columnas de la matriz corresponden a las coordenas en x
        #self.mapa.flags.writeable = True
        #self.mapa[self.mapa==0]=1#Conocido y libre
        #self.mapa[self.mapa==-1]=0#Desconocido
        #Obtengo el mapa inflado
        self.mapa_inflado=md.mapa_inflado_2(self,self.celdas_a_inflar, self.mapa)
        #Obtengo el mapa de costos
        self.mapa_de_costos=md.mapa_de_costos(self,self.celdas_de_costo,self.mapa_inflado)
        #Obtengo los puntos candidatos
        self.objetivos=md.Busqueda_Objetivos(self,self.celdas_a_inflar,self.mapa_inflado)
        
        
        list(set(self.objetivos))
        
        
        #Aqui hago el clustering con k means
        if len(self.objetivos)!=0:

                centroide_final=[(0,0),(0,0)]
                centroides=md.inicializar_centroides(self.objetivos,k=2)
                while centroide_final[0][0]!=centroides[0][0] or centroide_final[0][1]!=centroides[0][1] or centroide_final[1][0]!=centroides[1][0] or centroide_final[1][1]!=centroides[1][1]:
                    centroide_final=centroides
                    grupos=md.suma_del_error_al_cuadrado(self.objetivos, centroides)
                    centroides=[]
                    centroides=md.actualizar_centroide(grupos)
                    loop.sleep()#Para dormir un momento y alcanzar 20 Hz solo
                    
                
                for i in grupos:
                    md.visualizacion_objetivos(self,i,self.dato)
                    
                
        """
        for (x_goal,y_goal) in self.objetivos:
            #print(int(self.pos_x_robot), int(self.pos_y_robot),x_goal,y_goal)
            path=md.a_star(int(self.pos_y_robot), int(self.pos_x_robot),x_goal, y_goal, self.mapa_inflado, self.mapa_de_costos,self.dato)
            
            if len(path)!=0:

            
                
                md.visualizacion_objetivos(self,path,self.dato)
        
        

            #time.sleep(1)
            raw_input("Presione una tecla para continuar...")#esperar hasta que se presione una tecla
        """
        """
        costos_rutas=[]
        i=0
        for (x_goal,y_goal) in self.objetivos:
            i+=1
            costo_ruta,path=md.a_star(int(self.pos_y_robot), int(self.pos_x_robot),x_goal, y_goal, self.mapa_inflado, self.mapa_de_costos,self.dato)
            #path=md.get_smooth_path(path, alpha=0.9, beta=0.1)
            heapq.heappush(costos_rutas,(i,costo_ruta))
        
        costo_max=np.max(costos_rutas[:][1])
        punto_objetivo=np.where(costos_rutas[:][0]==costo_max)[0][0]
        print(punto_objetivo)
        
            if len(path)!=0:

            
                
                md.visualizacion_objetivos(self,path,self.dato)
                raw_input("Presione una tecla para continuar...")#esperar hasta que se presione una tecla
        
        #print(len(self.objetivos),len(self.costos_rutas),self.costos_rutas)
        print("Termine")
        """
        
        
        
         
        

          
        #------------------------------------------------------------------
        
        
    
if __name__ == "__main__":
    nodo=Nodo(Twist())
    loop=rospy.Rate(20)#20 Hz
    rospy.Subscriber('/odom',Odometry,nodo.posicion_robot_callback,queue_size=1)
    rospy.Subscriber('/map',OccupancyGrid,nodo.map_callback,queue_size=100)
    rospy.spin()

    
      
        
    
