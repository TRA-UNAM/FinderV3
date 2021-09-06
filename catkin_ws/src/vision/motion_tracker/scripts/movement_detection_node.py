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
fondo = []
flag = False

#Area minima del area que se considera movimiento
contourMinArea = 4500

def movement_detection_node():
  global image
  rospy.init_node('movement_detection_node')
  rospy.Subscriber('/camera0/usb_cam0/image_raw', Image, callback)
  rospy.Subscriber('/vision/movement_flag', Bool, flag_callback)
  pub = rospy.Publisher('/vision/movement_detected', Int32MultiArray, queue_size=10)
  #fps
  rate = rospy.Rate(30)
  rate.sleep()
  fondo = image
  #print('entrando al loop')
  while fondo == [] and not rospy.is_shutdown():
    #print(fondo)
    fondo = image
  #print('fuera del loop')
  fondo = cv2.cvtColor(fondo, cv2.COLOR_BGR2GRAY)
  fondo = cv2.GaussianBlur(fondo,(5,5),0)
  rate.sleep()
  cuenta_frames = 0
  buffer_obj = 0
  prom_obj = 0
  while not rospy.is_shutdown():
    if not flag:
      rate.sleep()
      continue
    #Se duplica la imagen de la camara para dibujar los resultados sobre la copia
    img = image.copy()
    #Se obtiene imagen en b/n y se aplica un filtro paso bajas
    gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gris = cv2.GaussianBlur(gris,(5,5),0)
    #Diferencia entre imagen anterior y reciente
    resta = cv2.absdiff(fondo, gris)
    # La imagen se pasa a blanco y negro con un umbral
    umbral = cv2.threshold(resta, 4, 255, cv2.THRESH_BINARY)[1]
    #umbral = cv2.adaptiveThreshold(resta,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)
    #Erosionamos el umbral para quitar ruido
    umbral = cv2.erode(umbral, None, iterations=2)
    # Dilatamos el umbral para tapar agujeros
    umbral = cv2.dilate(umbral, None, iterations=20)
    # Copiamos el umbral para detectar los contornos
    contornosimg = umbral.copy()
    #Se buscan los contornos de los elementos encontrados
    contornos, hier = cv2.findContours(contornosimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2:]
    num_targets = 0
    totalContornos =[
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000,
100000, 100000, 100000, 100000
,0]


    for c in contornos:
      # Eliminamos los contornos mas pequenos
      if cv2.contourArea(c) < contourMinArea:
        continue
      # Obtenemos el bounds del contorno, el rectangulo mayor que engloba al contorno
      (x, y, we, hi) = cv2.boundingRect(c)
      totalContornos[num_targets * 4 : num_targets * 4 + 4] = [x, y, we, hi]
      # Dibujamos el rectangulo del bounds
      img = cv2.rectangle(img, (x, y), (x + we, y + hi), (255, 0, 0), 2)
      #Contamos el numero de objetivos
      num_targets += 1
      if num_targets > 8:
        break
    totalContornos[36] = num_targets
    PubMsg = Int32MultiArray()
    PubMsg.data = totalContornos
    pub.publish(PubMsg)

    #Establece como fondo la imagen anterior
    fondo = gris
    #Pausa en funcion de los fps
    rate.sleep()

def callback(img):
  global image
  #np_arr = np.fromstring(img.data, np.uint8)
  #image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  image = bridge.imgmsg_to_cv2(img, "bgr8")

def flag_callback(currentFlag):
  global flag
  flag = currentFlag.data

if __name__ == '__main__':
  try:
    movement_detection_node()
  except rospy.ROSInterruptException:
    pass

