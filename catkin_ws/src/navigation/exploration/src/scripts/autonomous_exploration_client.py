#!/usr/bin/env python
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda

import math
from sys import float_repr_style
import tf
import os
import rospy
from exploration.srv import GetInflatedMap, GetBoundaryPoints, GetPointsVisualization, GetObjectivePoint
from nav_msgs.srv import GetMap
from exploration.msg import PotentialFields
from std_msgs.msg import Bool as Flag
from geometry_msgs.msg import Point



class Node:

    def __init__(self):
        self.init_node=rospy.init_node("autonomous_exploration_client")
        self.move_to_goal=rospy.Publisher('/navigation/move_base_simple/reach_goal',PotentialFields,queue_size=100)
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
        self.last_obj_x=0
        self.last_obj_y=0
        self.robot_a=0
        self.pos_x_robot=0
        self.pos_y_robot=0
        self.flag=False
        self.listener=tf.TransformListener()
        self.pot_fields=PotentialFields()
        
        
    def getPosRobot(self,map_origin_pos_x,map_origin_pos_y):

        try:
            ([x, y, z], rot) = self.listener.lookupTransform('map', 'base_link', rospy.Time(0))
            self.robot_a = 2*math.atan2(rot[2], rot[3])
            if self.robot_a > math.pi:
                self.robot_a = self.robot_a- 2*math.pi
            elif self.robot_a<=-math.pi:
                self.robot_a = self.robot_a+ 2*math.pi

            
            self.pos_x_robot = (abs(map_origin_pos_x))+(x)+(0.7*math.cos(self.robot_a))
            self.pos_y_robot = (abs(map_origin_pos_y))+(y)+(0.7*math.sin(self.robot_a))
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
        
            self.getPosRobot(self.data_mp.map.info.origin.position.x,self.data_mp.map.info.origin.position.y)

            print("Already we get the data related with the GetPosRobot service\n")
            print("The position of the robot in [m] is: "+str([self.pos_x_robot,self.pos_y_robot])+"\n")
            #----------------------------------------------------------------------

            
            
            
            

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
            """
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
            
            #----------------Objective Point Client--------------------------------
            print("Establishing the connection with Objective Point Server")
            rospy.wait_for_service('/navigation/mapping/get_objective_point')#Espero hasta que el servicio este habilitado
            try:
                if self.client_objetive==0:
                    self.client_objetive=rospy.ServiceProxy('/navigation/mapping/get_objective_point',GetObjectivePoint)#Creo un handler para poder llamar al servicio

                self.data_o=self.client_objetive(pos_x_robot=self.pos_x_robot,pos_y_robot=self.pos_y_robot,robot_a=self.robot_a,points=self.data_centroids.points,last_obj_x=self.last_obj_x,last_obj_y=self.last_obj_y)
                
            
            except rospy.ServiceException as e:
                print("The request for the objective point server failed: %s"%e)

            
            print("Already we get the objective: [{} , {}]".format(self.data_o.goal.x,self.data_o.goal.y)) 
            goal=[self.data_o.goal]
            
            
            """
                
            #----------------Points Visualization Client--------------------------------
            print("Establishing the connection with Points Visualization Server")
            rospy.wait_for_service('/navigation/mapping/get_points_visalization')#Espero hasta que el servicio este habilitado
            try:
                if self.client_visualization==0:
                    self.client_visualization=rospy.ServiceProxy('/navigation/mapping/get_points_visalization',GetPointsVisualization)#Creo un handler para poder llamar al servicio
                    
                    
                self.data_v=self.client_visualization(map_origin_pos_x=self.data_mp.map.info.origin.position.x,map_origin_pos_y=self.data_mp.map.info.origin.position.y,pos_x_robot=self.pos_x_robot,pos_y_robot=self.pos_y_robot,points=goal)
                
            
            except rospy.ServiceException as e:
                print("The request for the points visualization server failed: %s"%e)

            
            print("Already we get the visualizarion related with the GetPointsVisualization service\n")
        
        
            #---------------------------------------------------------------------
            """
            
        
            #----------------Move Robot by Potential Fields--------------------------------
        
            self.flag=True
            self.pot_fields.map_origin_pos_x=self.data_mp.map.info.origin.position.x
            self.pot_fields.map_origin_pos_y=self.data_mp.map.info.origin.position.y
            self.pot_fields.goal=self.data_o.goal
            self.move_to_goal.publish(self.pot_fields)
            print("\nEstablishing the connection with Potential Fields Node")
            print("Wating to reach the goal")
            
    
            #---------------------------------------------------------------------
                
                
                
            
            #-----------------------Update the last pose----------------------------------------
            self.last_obj_x=self.data_o.goal.x
            self.last_obj_y=self.data_o.goal.y
            
            #---------------------------------------------------------------------
                
    

if __name__== "__main__":
    
    node=Node()
    loop=rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            node.autonomous_exploration_client()
            rospy.Subscriber('/navigation/move_base_simple/reach_goal_response',Flag, node.callback_move_robot_response)
            loop.sleep()
        except:
            pass 
    
