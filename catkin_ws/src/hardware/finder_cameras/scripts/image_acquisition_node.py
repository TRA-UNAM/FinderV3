#!/usr/bin/env python
import rospy
#import numpy as np
import cv2
#import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError



def image_acquisition_node():
	#Inicializacion de camara
	cap = cv2.VideoCapture(0)
	#Inicializacion del convertidor de imagenes ROS-CV
	bridge = CvBridge()
	#Inicializacion del nodo y publicador
	pub = rospy.Publisher('/hardware/cam', Image, queue_size=10)
	rospy.init_node('image_acquisition_node', anonymous=True)
	#Frecuencia de muestreo de imagenes
	rate = rospy.Rate(30) # 10 fps
	#inicializacion de mensaje vacio
	img = Image()
	count=0
	#cv2.namedWindow("ads", 1)
        while not rospy.is_shutdown():
		#Se obtiene una imagen de la camara y se guarda en el mensaje
		cmp, image = cap.read()
		#La imagen obtenida se transforma al formato que utiliza ROS
		#Utilizando CVbridge  COLOR_BGR2GRAY
		#image2 = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		#cv2.imshow('ads',image2)
		img = bridge.cv2_to_imgmsg(image, "bgr8")
                #print(image.shape)
                if count == 10000: break
                pub.publish(img)
                count+=1
                rate.sleep()


if __name__ == '__main__':
        try:
                image_acquisition_node()
        except rospy.ROSInterruptException:
                pass

