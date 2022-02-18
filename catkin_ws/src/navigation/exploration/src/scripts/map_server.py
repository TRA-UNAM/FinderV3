#!/usr/bin/env python3
# coding=utf-8
#Autor: Axel Javier Rojas Mosqueda


import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap,GetMapResponse



class Server:

    def __init__(self):
       self.map=0
       
       


    def map_callback(self,dato):
        
        self.map=dato

        try:
            self.GetMap_server()
        except:
            
            pass
        
        
            

    def handle_GetMap(self,req):
        
        print("We already get the nav_msgs/OccupancyGrid message\n")
        
        return GetMapResponse(map=self.map)



    def GetMap_server(self):
        
        rospy.Service('/navigation/mapping/get_map', GetMap, self.handle_GetMap)
        print("The Map Server is ready for the request")
        
        

            
            #------------------------------------------------------------------   
    
if __name__ == "__main__":
    rospy.init_node('maps_erver')
    server=Server()
    rospy.Subscriber('/map',OccupancyGrid,server.map_callback,queue_size=1)
    rospy.spin()
    
    
    

    
      
        
    
