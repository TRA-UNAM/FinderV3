#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda


import rospy
from exploration.srv import Datos_rviz_mapeo,Inflar_mapa
from nav_msgs.msg import OccupancyGrid
import numpy as np

def mapa_inflado(self,grafo):
    num_celdas_inflar=3
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
    mapa_ocupacion.header.seq=self.dato.seq
    mapa_ocupacion.header.stamp=self.dato.stamp
    mapa_ocupacion.header.frame_id=self.dato.frame_id
    mapa_ocupacion.info.resolution=self.dato.resolution
    mapa_ocupacion.info.width = self.dato.width
    mapa_ocupacion.info.height = self.dato.height
    mapa_ocupacion.info.origin.position.x =self.dato.posicion_x
    mapa_ocupacion.info.origin.position.y =self.dato.posicion_y
    mapa_ocupacion.info.origin.orientation.x =self.dato.orientacion_x
    mapa_ocupacion.info.origin.orientation.y =self.dato.orientacion_y
    mapa_ocupacion.info.origin.orientation.z =self.dato.orientacion_z
    mapa_ocupacion.info.origin.orientation.w =self.dato.orientacion_w
    mapa_ocupacion.data= np.ravel(np.reshape(mapa_inflado, (len(self.dato.mapa), 1)))
    pub_inflated.publish(mapa_ocupacion)
    
    
    return mapa_inflado


class Nodo:

    def __init__(self):
        self.init_node=rospy.init_node("Exploration_and_Mapping", anonymous=True)
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
        self.seq=0
        self.stamp=0
        self.frame_id=0
        self.resolution=0
        self.width=0
        self.height=0
        self.pos_x=0
        self.pos_y=0
        self.ori_x=0
        self.ori_y=0
        self.ori_z=0
        self.ori_w=0

    

    def prueba_exploracion(self):
        #---------------Obtencion de mapa------------------------------------
        print("Esperando al servicio_del_mapa")
        rospy.wait_for_service('/servicio_mapa')#Espero hasta que el servicio este habilitado
        try:
            cliente_mapa=rospy.ServiceProxy('/servicio_mapa',Datos_rviz_mapeo)#Creo un handler para poder llamar al servicio
            self.dato=cliente_mapa()#Llamo el servicio
            
        
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        
        print("Ya obtuve el mapa")
        
        #----------------------------------------------------------------------
        
        #----------------Obtencion_mapa_inflado--------------------------------
        
        self.mapa=np.array(self.dato.mapa).reshape((self.dato.height, self.dato.width))
        self.mapa_inflado=mapa_inflado(self,self.mapa)
        print("Se debe reflejar el mapa inflado")
        #---------------------------------------------------------------------
        




if __name__== "__main__":
    nodo=Nodo()
    nodo.prueba_exploracion()