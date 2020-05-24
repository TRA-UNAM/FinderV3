#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import pickle
import sys
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
image = []
nameList = [255, 255, 255, 255]

#Lee las imagenes de la base de datos
with open(sys.path[0] + '/database_full.pkl','rb') as database_full:
        database_full = pickle.load(database_full)
database = database_full[0]
name_database = database_full[1]
MISSINGNO = cv2.imread(sys.path[0] + '/MISSINGNO.jpg')
#database_full = []

#Bandera que indica si realizar el reconocimiento
flag = False

def callback(Names):
  global nameList
  nameList = list(Names.data)

def flag_callback(currentFlag):
  global flag
  flag = currentFlag.data

def label_display_node():
  global nameList
  global flag
  rospy.init_node('label_display_node')
  rospy.Subscriber('/vision/detected_labels', Int16MultiArray, callback)
  rospy.Subscriber('/vision/label_flag', Bool, flag_callback)
  pub = rospy.Publisher('/vision/detected_labels_image_raw', Image, queue_size = 10)
  #fps
  rate = rospy.Rate(2)
  while not rospy.is_shutdown():
    detectedLabelsImage = []
    currentList = nameList
    currentList.sort()
    #Si la bandera esta arriba, verifica los nombres en el mensaje publicado, crea la imagen y la publica
    if flag:
      imageArray = np.empty(4, dtype=object)
      for c in range(len(currentList)):
        if currentList[c] != 255:
          imageArray[c] = database[currentList[c]]
        else:
          imageArray[c] = MISSINGNO
      #Crea el cuadro de 2x2 imagenes
      detectedLabelsImage = cv2.vconcat([cv2.hconcat([imageArray[0],imageArray[1]]),cv2.hconcat([imageArray[2],imageArray[3]])])
    #Si no hay bandera, publica una imagen vacia
    else:
      detectedLabelsImage = cv2.resize(MISSINGNO,(800,800))
    pub.publish(bridge.cv2_to_imgmsg(detectedLabelsImage, encoding="bgr8"))
    rate.sleep()

if __name__ == '__main__':
        try:
                label_display_node()
        except rospy.ROSInterruptException:
                pass
