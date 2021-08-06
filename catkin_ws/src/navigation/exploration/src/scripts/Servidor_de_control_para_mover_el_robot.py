#!/usr/bin/env python
#!/usr/bin/env python3
# coding=utf-8
import sys
import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from exploration.srv import Posicion_robot
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Servicio:

    def __init__(self,path,posicion_x,posicion_y,width,height,resolution,cliente):

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_markers = rospy.Publisher('/navigation/visualization_markers', Marker, queue_size=100)
        self.loop=rospy.Rate(20)
        self.robot_a=0
        self.robot_x=0
        self.robot_y=0
        self.path=path
        self.posicion_x=posicion_x
        self.posicion_y=posicion_y
        self.width=width
        self.height=height
        self.resolution=resolution
        self.cliente_posicion_robot=cliente
        self.laser=[]
        self.follow_path()
        self.angle_increment=0


    def get_force_marker(self,robot_x, robot_y, force_x, force_y, color, id):
        mrk = Marker()
        mrk.header.frame_id = "map"
        mrk.header.stamp    = rospy.Time.now()
        mrk.ns = "visualizations"
        mrk.id = id
        mrk.type   = Marker.ARROW
        mrk.action = Marker.ADD
        mrk.color.r, mrk.color.g, mrk.color.b, mrk.color.a = color
        mrk.scale.x, mrk.scale.y, mrk.scale.z = [0.07, 0.1, 0.15]
        mrk.points.append(Point())
        mrk.points.append(Point())
        mrk.points[0].x, mrk.points[0].y = robot_x,  robot_y
        mrk.points[1].x, mrk.points[1].y = robot_x - force_x, robot_y - force_y
        return mrk




    def draw_force_markers(self,robot_x, robot_y, attr_x, attr_y, rej_x, rej_y, res_x, res_y, pub_markers):
        pub_markers.publish(self.get_force_marker(robot_x, robot_y, attr_x, attr_y, [0,0,1,1]  , 0))
        pub_markers.publish(self.get_force_marker(robot_x, robot_y, rej_x,  rej_y,  [1,0,0,1]  , 1))
        pub_markers.publish(self.get_force_marker(robot_x, robot_y, res_x,  res_y,  [0,0.6,0,1], 2))



    def callback_scan(self,msg):
        global laser_readings
        laser_readings = [[0,0] for i in range(len(msg.ranges))]
        for i in range(len(msg.ranges)):
            laser_readings[i] = [msg.ranges[i], msg.angle_min + i*msg.angle_increment]


    def obtener_pos_robot(self):
        print("Esperando al servicio_posicion_robot")
        rospy.wait_for_service('/servicio_posicion_robot')#Espero hasta que el servicio este habilitado
        try:
            posicion_robot=self.cliente_posicion_robot(posicion_x=self.posicion_x,posicion_y=self.posicion_y,width=self.width,height=self.height,resolution=self.resolution)
            self.robot_y=posicion_robot.posicion_x_robot
            self.robot_x=posicion_robot.posicion_y_robot
            self.robot_a=posicion_robot.robot_a
        except rospy.ServiceException as e:
            print("Fallo la solicitud del servidor posicion del robot: %s"%e)
            
        rospy.Subscriber('/scan',LaserScan,self.callback_scan,queue_size=10)


    def calculate_control(self,robot_x, robot_y, robot_a, goal_x, goal_y):
        cmd_vel = Twist()
        
        #
        # Implement the control law given by:
        #
        error_a=(math.atan2(goal_x-robot_x,goal_y-robot_y))-robot_a#Obtengo el error de angulo
        alpha=0.5
        beta=0.5
        if error_a>math.pi:
            error_a=error_a-2*math.pi
        
        elif error_a<=-math.pi:
            error_a=error_a+2*math.pi


        v = 0.5*math.exp(-error_a*error_a/alpha)
        w = 0.5*(2/(1 + math.exp(-error_a/beta)) - 1)
        #
        # where error_a is the angle error and
        # v and w are the linear and angular speeds taken as input signals
        # and v_max, w_max, alpha and beta, are tunning constants.
        # Store the resulting v and w in the Twist message cmd_vel
        # and return it (check online documentation for the Twist message).
        # Remember to keep error angle in the interval (-pi,pi]
        #
        

        cmd_vel.linear.x=v
        cmd_vel.angular.z=w

        return cmd_vel



    def attraction_force(self,robot_x, robot_y, goal_x, goal_y):
        #
        # Calculate the attraction force, given the robot and goal positions.
        # Return a tuple of the form [force_x, force_y]
        # where force_x and force_y are the X and Y components
        # of the resulting attraction force w.r.t. map.
        #
        intensidad_atraccion=1
        force_x=(intensidad_atraccion/math.sqrt((robot_x-goal_x)**2+(robot_y-goal_y)**2))*(robot_x-goal_x)
        force_y=(intensidad_atraccion/math.sqrt((robot_x-goal_x)**2+(robot_y-goal_y)**2))*(robot_y-goal_y)

        return [force_x, force_y]

    def rejection_force(self,robot_x, robot_y, robot_a, laser_readings):
        #
        # Calculate the total rejection force given by the average
        # of the rejection forces caused by each laser reading.
        # laser_readings is an array where each element is a tuple [distance, angle] el angulo es con respecto al robot
        # both measured w.r.t. robot's frame.
        # See lecture notes for equations to calculate rejection forces.
        # Return a tuple of the form [force_x, force_y]
        # where force_x and force_y are the X and Y components
        # of the resulting rejection force w.r.t. map.
        #
        force_x=0
        force_y=0
        d0=1#A partir de 0.8 metros del robot no voy a contar la resolusion
        intensidad_repulsion=10
        i=0
        
        for lectura_ls in laser_readings:
            d=lectura_ls[0]#Corresponde a la distancia d, desde el robot hasta el obstaculo
            
            
            if d>0 and d<d0 and np.isfinite(d) and not np.isnan(d):
                i+=1  
                theta_obs=robot_a+lectura_ls[1]#Obtengo el angulo del obstaculo con respecto al sistema de referencia general, ya que al angulo del robot le sumo el angulo del obstaculo con respecto al robot
                punto_obs_x=robot_x+d*math.cos(theta_obs)
                punto_obs_y=robot_y+d*math.sin(theta_obs)
                force_x+=((intensidad_repulsion*math.sqrt((1/d)-(1/d0)))/d)*(punto_obs_x-robot_x)
                force_y+=((intensidad_repulsion*math.sqrt((1/d)-(1/d0)))/d)*(punto_obs_y-robot_y)
            else:
                pass

        if i!=0:
            force_x=force_x/i
            force_y=force_y/i
        
        return [force_x, force_y]




    def follow_path(self):
        #id0=0
        idx=0
        #[robot_x,robot_y]=self.path[id0]
        self.obtener_pos_robot()
        [global_x,global_y]=self.path[-1]
        [goal_x,goal_y]=self.path[idx]#Se trata del punto objetivo
        epsilon=0.5
        
        global_error=math.sqrt((global_x-self.robot_x)**2+(global_y-self.robot_y)**2)#Calculo el error global
        local_error=math.sqrt((goal_x-self.robot_x)**2+(goal_y-self.robot_y)**2)#Calculo el error de la posicion del robot al punto siguiente
    
        while global_error>0.1:
            
            [fax, fay] = self.attraction_force(self.robot_x, self.robot_y, goal_x, goal_y)#Calculamos la fuerza de atraccion
            [frx, fry] = self.rejection_force (self.robot_x, self.robot_y, self.robot_a, self.laser)#Calculamos la fuerza de repulsion
            [fx,fy]=[fax+frx,fay+fry]#Obtenemos la fuerza resultante
            [px,py]=[self.robot_x-epsilon*fx,self.robot_y-epsilon*fy]#Obtenemos los puntos objetivo locales con la fuerza neta restada multiplicada por epsilon
            msg_cmd_vel=self.calculate_control(self.robot_y,self.robot_x,self.robot_a,px,py)
            self.pub_cmd_vel.publish(msg_cmd_vel)
            self.draw_force_markers(self.robot_x, self.robot_y, fax, fay, frx, fry, fx, fy, self.pub_markers)#Para dibujar las fuerzas
            self.loop.sleep()
            self.obtener_pos_robot()
            local_error=math.sqrt((goal_x-self.robot_x)**2+(goal_y-self.robot_y)**2)#Calculo el error de la posicion del robot al punto siguiente
            if local_error<0.3:
                idx+=1
                if idx>=len(self.path):
                    idx=len(self.path)-1

                [goal_x,goal_y]=self.path[idx]
            global_error=math.sqrt((global_x-self.robot_x)**2+(global_y-self.robot_y)**2)#Calculo el error global

        self.pub_cmd_vel.publish(Twist())
        


def mover_robot(path,posicion_x,posicion_y,width,height,resolution,servicio):
    
    servicio=Servicio(path,posicion_x,posicion_y,width,height,resolution,servicio)
    
    




