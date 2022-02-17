#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda


import rospy
from exploration.srv import GetInflatedMap, GetBoundaryPoints, GetPointsVisualization, Objetivo, Mover_robot
import os
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
import math
import tf


class Node:

    def __init__(self):
        self.init_node=rospy.init_node("autonomous_exploration_client")
        self.data_mp=0
        self.data_im=0
        self.data_bp=0
        self.data_pr=0
        self.data_o=0
        self.data_v=0
        self.data_centroids=0
        self.inflated_cells=0.2#metros a inflar, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
        self.last_pos_x_robot=-10
        self.last_pos_y_robot=-10
        self.last_robot_a=2
        self.client_map=0
        self.client_pos_robot=0
        self.client_inflated_map=0
        self.client_boundary_points=0
        self.client_visualization=0
        self.client_centroids=0
        self.client_objetive=0
        self.cliente_mr=0
        self.centroid_x=[]
        self.centroid_y=[]
        self.last_obj_x=0
        self.last_obj_y=0
        self.robot_a=0
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.listener=tf.TransformListener()
        
    def getPosRobot(self,map_origin_pos_x,map_origin_pos_y):

        
        ([x, y, z], rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
        self.robot_a = 2*math.atan2(rot[2], rot[3])
        if self.robot_a > math.pi:
            self.robot_a = self.robot_a- 2*math.pi
        elif self.robot_a<=-math.pi:
            self.robot_a = self.robot_a+ 2*math.pi

        
        self.pos_x_robot = (abs(map_origin_pos_x))+(x)+(0.7*math.cos(self.robot_a))
        self.pos_y_robot = (abs(map_origin_pos_y))+(y)+(0.7*math.sin(self.robot_a))
        
    

    def autonomous_exploration_client(self):

        
        #os.system("clear")#Limpiar terminal

        #---------------Map Client------------------------------------
        print("Establishing the connection with Map Server")
        rospy.wait_for_service('/navigation/mapping/get_map')#Espero hasta que el servicio este habilitado
        try:
            if self.client_map==0:
                self.client_map=rospy.ServiceProxy('/navigation/mapping/get_map',GetMap)#Creo un handler para poder llamar al servicio

            self.data_mp=self.client_map()#Llamo el servicio
            
        
        except rospy.ServiceException as e:
            print("The request for the map server failed: %s"%e)

        
        print("Already we get the data related with the GetMap service\n")


    
        
        #----------------------------------------------------------------------
        
        if self.pos_x_robot!=self.last_pos_x_robot or self.robot_a!=self.last_robot_a or self.pos_y_robot!=self.last_pos_x_robot:
            

        
            #----------------Inflated Map Client--------------------------------
            print("Establishing the connection with Inflated Map Server")
            rospy.wait_for_service('/navigation/mapping/get_inflated_map')#Espero hasta que el servicio este habilitado
            try:
                if self.client_inflated_map==0:
                    self.client_inflated_map=rospy.ServiceProxy('/navigation/mapping/get_inflated_map',GetInflatedMap)#Creo un handler para poder llamar al servicio
                self.data_im=self.client_inflated_map(inflated_cells=self.inflated_cells, map=self.data_mp.map)
                
            
            except rospy.ServiceException as e:
                print("The request for the inflated map server failed: %s"%e)

            
            print("Already we get the data related with the GetInflatedMap service\n")

        
            #-------------------------------------------------------------------------
            """
            #----------------Boundary Points Client--------------------------------
            print("Establishing the connection with Boundary Points Server")
            rospy.wait_for_service('/navigation/mapping/get_boundary_points')#Espero hasta que el servicio este habilitado
            try:
                if self.client_boundary_points==0:
                    self.client_boundary_points=rospy.ServiceProxy('/navigation/mapping/get_boundary_points',GetBoundaryPoints)#Creo un handler para poder llamar al servicio
                
                self.data_bp=self.client_boundary_points(map=self.data_im.inflated_map)
                
        
            except rospy.ServiceException as e:
                print("The request for the boundary points server failed: %s"%e)

            
            print("Already we get the data related with the GetBoundaryPoints service\n")
            print("The number of Boundary Points founded are "+str(len(self.data_bp.points))+"\n")


            
            
            #---------------------------------------------------------------------

        
            
        
            
            
            #----------------Points Visualization Client--------------------------------
            print("Establishing the connection with Points Visualization Server")
            rospy.wait_for_service('/navigation/mapping/get_points_visalization')#Espero hasta que el servicio este habilitado
            try:
                if self.client_visualization==0:
                    self.client_visualization=rospy.ServiceProxy('/navigation/mapping/get_points_visalization',GetPointsVisualization)#Creo un handler para poder llamar al servicio
                    
                    
                self.data_v=self.client_visualization(map_origin_pos_x=self.data_mp.map.info.origin.position.x,map_origin_pos_y=self.data_mp.map.info.origin.position.y,pos_x_robot=self.data_pr.pos_x_robot,pos_y_robot=self.data_pr.pos_y_robot,points=self.data_bp.points)
                
            
            except rospy.ServiceException as e:
                print("The request for the points visualization server failed: %s"%e)

            
            print("Already we get the visualizarion related with the GetPointsVisualization service\n")
        
        
            #---------------------------------------------------------------------
        
            """
        
            #----------------K-Means Client--------------------------------
            print("Establishing the connection with K-Means Server")
            rospy.wait_for_service('/navigation/mapping/get_boundary_points_clustered')#Espero hasta que el servicio este habilitado
            try:
                if self.client_centroids==0:
                    self.client_centroids=rospy.ServiceProxy('/navigation/mapping/get_boundary_points_clustered',GetBoundaryPoints)#Creo un handler para poder llamar al servicio
                
                self.data_centroids=self.client_centroids(map=self.data_im.inflated_map)
                
            
            except rospy.ServiceException as e:
                print("The request for the centorids server failed: %s"%e)

            
            print("Already we get {0} clusters from the boundary points\n".format(self.data_centroids.k)) 
            
            
            
            #---------------------------------------------------------------------
            

            
            #----------------Points Visualization Client--------------------------------
            print("Establishing the connection with Points Visualization Server")
            rospy.wait_for_service('/navigation/mapping/get_points_visalization')#Espero hasta que el servicio este habilitado
            try:
                if self.client_visualization==0:
                    self.client_visualization=rospy.ServiceProxy('/navigation/mapping/get_points_visalization',GetPointsVisualization)#Creo un handler para poder llamar al servicio
                    
                    
                self.data_v=self.client_visualization(map_origin_pos_x=self.data_mp.map.info.origin.position.x,map_origin_pos_y=self.data_mp.map.info.origin.position.y,pos_x_robot=self.pos_x_robot,pos_y_robot=self.pos_y_robot,points=self.data_centroids.points)
                
            
            except rospy.ServiceException as e:
                print("The request for the points visualization server failed: %s"%e)

            
            print("Already we get the visualizarion related with the GetPointsVisualization service\n")
        
        
            #---------------------------------------------------------------------
        
            """
            
            #----------------Obtener el punto objetivo--------------------------------
            print("Esperando al servicio punto objetivo")
            rospy.wait_for_service('/servicio_objetivo')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_objetivo==0:
                    self.cliente_objetivo=rospy.ServiceProxy('/servicio_objetivo',Objetivo)#Creo un handler para poder llamar al servicio

                self.dato_o=self.cliente_objetivo(centroides_x=self.dato_km.centroides_x,centroides_y=self.dato_km.centroides_y,posicion_x_robot=self.dato_pr.posicion_x_robot,posicion_y_robot=self.dato_pr.posicion_y_robot,robot_a=self.dato_pr.robot_a,obj_ant_x=self.obj_ant_x,obj_ant_y=self.obj_ant_y) 
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud de obtener un punto objetivo: %s"%e)

            
            print("Ya se tienen el punto objetivo: {},{}".format(self.dato_o.obj_x,self.dato_o.obj_y)) 
            #self.centroides_x=self.dato_o.centroides_x
            #self.centroides_y=self.dato_o.centroides_y
            obj_x=[self.dato_o.obj_x]
            obj_y=[self.dato_o.obj_y]
            
            
            
            

            
            #---------------------------------------------------------------------
        
      

           
            
            #----------------Mover al Robot--------------------------------
                
            
            print("Esperando al servicio_mover_robot")
            rospy.wait_for_service('/servicio_mover_robot')#Espero hasta que el servicio este habilitado
            try:
                if self.cliente_mr==0:
                    self.cliente_mr=rospy.ServiceProxy('/servicio_mover_robot',Mover_robot)#Creo un handler para poder llamar al servicio
                    
                    
                dato_mr=self.cliente_mr(posicion_x=self.dato.posicion_x,posicion_y=self.dato.posicion_y,obj_x=obj_x,obj_y=obj_y,width=self.dato.width,height=self.dato.height,resolution=self.dato.resolution)
                
            
            except rospy.ServiceException as e:
                print("Fallo la solicitud de mover el robot: %s"%e)

            
            print("Ya movi el robot hasta el punto seleccionado\n") 
            
        

        
            
        
        
            
            #---------------------------------------------------------------------
            
        """
            #-----------------------Refresco la posicion del robot anterior----------------------------------------
            self.pos_x_robot_ant=self.pos_x_robot
            self.pos_y_robot_ant=self.pos_y_robot
            self.robot_a_ant=self.robot_a
            #self.obj_ant_x=self.dato_o.obj_x
            #self.obj_ant_y=self.dato_o.obj_y

            #---------------------------------------------------------------------
            #--------------------------GetRobotPos---------------------------------
        
            self.getPosRobot(self.data_mp.map.info.origin.position.x,self.data_mp.map.info.origin.position.y)

            print("Already we get the data related with the GetPosRobot service\n")
            print("The position of the robot in [m] is: "+str((self.pos_x_robot,self.pos_y_robot))+"\n")
            #----------------------------------------------------------------------
                
        """
        #----------------Pos Robot Client [m]--------------------------------

        
        print("Establishing the connection with Pos Robot Server")
        rospy.wait_for_service('/navigation/localization/get_pos_robot')#Espero hasta que el servicio este habilitado
        try:
            if self.client_pos_robot==0:
                self.client_pos_robot=rospy.ServiceProxy('/navigation/localization/get_pos_robot',GetPosRobot)#Creo un handler para poder llamar al servicio
            
            self.data_pr=self.client_pos_robot(map_origin_pos_x=self.data_mp.map.info.origin.position.x,map_origin_pos_y=self.data_mp.map.info.origin.position.y)

        except rospy.ServiceException as e:
            print("The request for the pos robot server failed: %s"%e)

        
        print("Already we get the data related with the GetPosRobot service\n")
        print("The position of the robot in [m] is: "+str((self.data_pr.pos_x_robot,self.data_pr.pos_y_robot))+"\n")

        #---------------------------------------------------------------------
        """    
    

if __name__== "__main__":
    node=Node()
    loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        node.autonomous_exploration_client()
        loop.sleep()
