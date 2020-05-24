#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import sys
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
image = []
coords = []
flag = False

#Imagen con numeros para colocar en la imagen (en la misma carpeta) y obtener una mascara
numeros = cv2.imread(sys.path[0] + '/Numeros.png')
MISSINGNO = cv2.imread(sys.path[0] + '/MISSINGNO.jpg')
mask_inv = cv2.threshold(cv2.cvtColor(numeros, cv2.COLOR_BGR2GRAY), 200, 255, cv2.THRESH_BINARY)[1]
mask = cv2.bitwise_not(mask_inv)
#Area minima del area que se considera movimiento
contourMinArea = 5000

def movement_display_node():
  global image
  global coords
  global flag
  rospy.init_node('movement_display_node')
  rospy.Subscriber('/vision/movement_flag', Bool, flag_callback)
  rospy.Subscriber('/vision/movement_detected', Int32MultiArray, callback)
  rospy.Subscriber('/cam0/image_raw_local', Image, image_callback)
  pub = rospy.Publisher('/vision/movement_detected_image_raw', Image, queue_size = 10)
  #fps
  rate = rospy.Rate(15)
  while not rospy.is_shutdown():
    #Si la bandera esta arriba lee los mensajes
    if flag:
      if image == [] or coords == []:
        pub.publish(bridge.cv2_to_imgmsg(MISSINGNO, encoding="bgr8"))
        continue
      currentimage = image.copy()
      for c in range(9):
        if coords[c*4] == 100000:
          continue
        currentimage = cv2.rectangle(currentimage, (coords[c*4], coords[c*4 + 1]), (coords[c*4] + coords[c*4 + 2], coords[c*4 + 1] + coords[c*4 + 3]), (255, 0, 0), 2)
      currentimage = print_num(currentimage,coords[36])
      pub.publish(bridge.cv2_to_imgmsg(currentimage, encoding="bgr8"))
    else:
      pub.publish(bridge.cv2_to_imgmsg(MISSINGNO, encoding="bgr8"))
    rate.sleep()

def callback(Coords):
  global coords
  coords = Coords.data

def image_callback(img):
  global image
  image = bridge.imgmsg_to_cv2(img, "bgr8")

def flag_callback(currentFlag):
  global flag
  flag = currentFlag.data

def print_num(original,num=0):
  new_img = original.copy()
  #Dimensiones de cada numero en la imagen
  rows = 68
  cols = 50
  #Dimensiones de la imagen original
  rows_o, cols_o, ncha = original.shape
  #Mascara del numero a colocar
  mask_n = mask[0:rows,num*cols:(num*cols)+cols]
  mask_inv_n = mask_inv[0:rows,num*cols:(num*cols)+cols]
  #Cuadro del numero a colocar
  numero_act = numeros[0:rows,num*cols:(num*cols)+cols]
  #Region Of Interest (ROI) para que el numero este' en la esquina inferior derecha
  roi = new_img[rows_o-rows-20:rows_o-20,cols_o-cols-20:cols_o-20]
  #Obtener la region donde estara el numero, con espacio para el numero
  roi_clear = cv2.bitwise_and(roi,roi,mask = mask_inv_n)
  #Obtener region del numero de la imagen
  num_imag = cv2.bitwise_and(numero_act,numero_act,mask = mask_n)
  #Se coloca el numero en la ROI y se coloca en la imagne completa
  dst = cv2.add(roi_clear,num_imag)
  new_img[rows_o-rows-20:rows_o-20,cols_o-cols-20:cols_o-20] = dst
  return new_img


if __name__ == '__main__':
  try:
    movement_display_node()
  except rospy.ROSInterruptException:
    pass
