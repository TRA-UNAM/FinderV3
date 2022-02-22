#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import math
import tf
import os
import rospy
from exploration.srv import GetInflatedMap, GetBoundaryPoints, GetGoalPoint
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool as Flag
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker



class Node:

    def __init__(self):
        self.init_node=rospy.init_node("autonomous_exploration_client")
        self.move_to_goal=rospy.Publisher('/navigation/move_base_simple/goal',Point,queue_size=10)
        self.data_mp=0
        self.data_im=0
        self.data_bp=0
        self.data_pr=0
        self.data_o=0
        self.data_v=0
        self.data_centroids=0
        self.inflated_cells=0.2#metros a inflar, depende de la resolucion, pero usualmente cada celda son 0.05 m o 5 cm
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
        self.last_objective=Point()
        self.last_objective.x=100
        self.last_objective.y=100
        self.pos_a_robot=0
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.flag=False
        self.listener=tf.TransformListener()
        
    def visualization_points(self,points):

        pub =rospy.Publisher('/visualization_marker',Marker, queue_size=100)
        markers=Marker(ns="points",type=Marker.POINTS,action=Marker.ADD,lifetime=rospy.Duration(),id=0)
        markers.header.stamp=rospy.Time()
        markers.header.frame_id="/map"
        markers.pose.orientation.w=1.0
        #Esta es la posicion de la marca
        markers.pose.position.x=0 
        markers.pose.position.y=0
        markers.pose.position.z=0
        #markers
        markers.scale.x=0.2#Tamaño de los markers
        markers.scale.y=0.2
        markers.scale.z=0.2
        #Los markers seran Azules
        #markers.color.r=1.0#Color  
        markers.color.r=1.0#Color 
        markers.color.a=1.0#Nitidez
        
        markers.points=points
        
        
        
        pub.publish(markers)   
        
        
    def getPosRobot(self):

        try:
            ([self.pos_x_robot, self.pos_y_robot, z], rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            self.pos_a_robot = 2*math.atan2(rot[2], rot[3])
            if self.pos_a_robot > math.pi:
                self.pos_a_robot = self.pos_a_robot- 2*math.pi
            elif self.pos_a_robot<=-math.pi:
                self.pos_a_robot = self.pos_a_robot+ 2*math.pi

        except:
            pass

    def callback_move_robot_response(self,msg):
        self.flag=msg.data



    def autonomous_exploration_client(self):

        
        if self.flag==False:
            os.system("clear")#Limpiar terminal

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

        
            """
            #--------------------------GetRobotPos---------------------------------
        
            self.getPosRobot()
            print("Already we get the data related with the GetPosRobot service\n")
            print("The position of the robot in [m] is: "+str([self.pos_x_robot,self.pos_y_robot])+"\n")
            #----------------------------------------------------------------------

            
            
            
            

            #----------------centroids Client--------------------------------
            print("Establishing the connection with Centroids Server")
            rospy.wait_for_service('/navigation/mapping/get_boundary_points_clustered')#Espero hasta que el servicio este habilitado
            try:
                if self.client_centroids==0:
                    self.client_centroids=rospy.ServiceProxy('/navigation/mapping/get_boundary_points_clustered',GetBoundaryPoints)#Creo un handler para poder llamar al servicio
                
                self.data_centroids=self.client_centroids(map=self.data_im.inflated_map)
                
            
            except rospy.ServiceException as e:
                print("The request for the centorids server failed: %s"%e)

            
            print("Already we get {0} clusters from the boundary points\n".format(self.data_centroids.k)) 
            
            
            
            
            
            #----------------Goal Point Client--------------------------------
            print("Establishing the connection with Goal Point Server")
            rospy.wait_for_service('/navigation/mapping/get_goal_point')#Espero hasta que el servicio este habilitado
            try:
                if self.client_objetive==0:
                    self.client_objetive=rospy.ServiceProxy('/navigation/mapping/get_goal_point',GetGoalPoint)#Creo un handler para poder llamar al servicio

                self.data_o=self.client_objetive(pos_x_robot=self.pos_x_robot,pos_y_robot=self.pos_y_robot,pos_a_robot=self.pos_a_robot,points=self.data_centroids.points,last_objective=self.last_objective,method="angle_and_distance")
                
            
            except rospy.ServiceException as e:
                print("The request for the objective point server failed: %s"%e)

            
            print("Already we get the objective: [{} , {}]".format(self.data_o.goal.x,self.data_o.goal.y)) 
            
            #self.visualization_points([self.data_o.goal])
            
            #----------------Move Robot by Potential Fields--------------------------------
        
            self.flag=True
            self.move_to_goal.publish(self.data_o.goal)
            print("\nEstablishing the connection with Potential Fields Node")
            print("Wating to reach the goal")
            
    
            #---------------------------------------------------------------------
                
                
            
            
            #-----------------------Update the last pose----------------------------------------
            self.last_objective=self.data_o.goal
            
            
            #---------------------------------------------------------------------
                
    

if __name__== "__main__":
    
    node=Node()
    loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            node.autonomous_exploration_client()
            rospy.Subscriber('/move_base_simple/goal_response',Flag, node.callback_move_robot_response)
            loop.sleep()
        except:
            pass 
    
