#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import math
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

yl = 54
crl = 55
cbl = 55

yh =255
crh =255
cbh = 250

#Image converter
bridge = CvBridge()
image = []

def main():
  global image
  global rad
  rospy.init_node('c_pattern_detection_node')
  rospy.Subscriber('/camera0/usb_cam0/image_raw', Image, ImageCallback)
  pub = rospy.Publisher('/vision/c_pattern_info', String, queue_size=10)
  rate = rospy.Rate(10)
  rate.sleep()

  #Our operations on the frame come here
  #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

  while not rospy.is_shutdown():
    rate.sleep()
    gapAngle = String()
    #Get image from camera
    data = image.copy()
    #Converts to YCBCR colorspace
    yuvImg = cv2.cvtColor(data.copy(), cv2.COLOR_BGR2YCR_CB)
    #Blur and close filter
    fImg = prevFilter(data)
    #Black color mask
    yuvMask = maskf(fImg)
    #Edges from b/n mask result
    edges = cv2.Canny(yuvMask,50,150,apertureSize=3)
    #circles=cv2.HoughCircles(edges,cv2.HOUGH_GRADIENT,dp=1,minDist=20,param1=50,param2=30,minRadius=50,maxRadius=80)
    circlei = cv2.HoughCircles(yuvMask,cv2.HOUGH_GRADIENT,1,20, param1=100, param2=50, minRadius=0, maxRadius=0)

    imguno = yuvMask
    cv2.imshow('yvm',yuvMask)
    #If circles are detected
    if circlei is not None:
      #Sort detected circles from biggest to smallest
      circlei = sorted(circlei[0],key=lambda x:x[2],reverse=True)

      #Iterates from the biggest to smallest circle trying to find 3 inner circles with gaps
      gapFound = False
      zz = True
      for c in circlei:
        if gapFound:
           break
        #Creates Region Of Interest (ROI) with circle inside
        rad = 2.1 * (c[2] / 2)
        roi = yuvMask[int(c[1]-rad) : int(c[1]+rad), int(c[0]-rad) : int(c[0]+rad)]
        croi =   fImg[int(c[1]-rad) : int(c[1]+rad), int(c[0]-rad) : int(c[0]+rad),:]
        if not all(roi.shape):
          break

        circlesROI = False
        #Searches for at least a circles inside ROI
        localCircles = cv2.HoughCircles(roi,cv2.HOUGH_GRADIENT,1,1,param1=150,param2=55, minRadius=0,maxRadius=int(rad))
        #Searches for max 10 corners inside ROI
        corners = cv2.goodFeaturesToTrack(roi, 20, 0.01, roi.shape[0] * 0.01)

        #Searches for points of 3rd C
        if corners is not None and localCircles is not None: 
          cPoints = []
          numpoints = [0, 0, 0]
          for co in corners:
            #Gets center distance
            dist = math.sqrt( (co[0,0]-rad)**2 + (co[0,1]-rad)**2 )/rad 
            cv2.circle(croi,(co[0,0],co[0,1]),3,(2,2,255),-1)
            #Assigns point to C
            if 0.1 <= dist and dist <= 0.2:
              numpoints[0] += 1
              #Saves point
              cPoints.append(co[0])
              #Draws points
              cv2.circle(croi,(co[0,0],co[0,1]),3,(255,255,0),-1)
            elif 0.5 <= dist and dist <= 0.62:
              numpoints[1] += 1
            elif 0.9 <= dist:
              numpoints[2] += 1

          #Checks if there is at least 2 points for each C
          if min(numpoints) >= 2:
            #Takes only close points to each other from 3rd C
            for p in range(len(cPoints) - 1):
              for bb in range(len(cPoints) - p - 1):
                q = bb + p + 1
                #Relative distance between points
                dist = math.sqrt( (cPoints[p][0]-cPoints[q][0])**2 + (cPoints[p][1]-cPoints[q][1])**2 )
                if 0.10 < dist / rad < 0.18:
                   #Midpoint and angle
                   pmy = (cPoints[q][1] + cPoints[p][1]) / 2 - rad
                   pmx = (cPoints[q][0] + cPoints[p][0]) / 2 - rad
                   ang = - ( np.arctan2( pmy, pmx ) ) * 180 / np.pi 
                   gapFound = True
                   gapAngle.data = str(ang)
                   pub.publish(gapAngle)
                   break
        #cv2.imshow('roi',roi)
        #cv2.imshow('croi',croi)

    if cv2.waitKey(1) & 0xFF == ord('q'):
      return True

def ImageCallback(img):
  global image
  image = bridge.imgmsg_to_cv2(img, "bgr8")

def prevFilter(img):
  # Ventana para aplicar el filtro de dilatcion y erosion
  wKernel = np.ones((3,3),np.uint8)
  # Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
  fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
  # Filtro mediana de suavizamiento de imagen
  fImg = cv2.medianBlur(fImg, 5)
  fImg = cv2.medianBlur(fImg, 5)
  #Sharpenning image
  #fImgG = cv2.GaussianBlur(fImg,(0,0),3)
  #fImg  = cv2.addWeighted(fImg, 1.5, fImgG, -0.5, 0)
  return fImg


#  Funcion para obtener la mascara binaria, a partir del color de interes especificado
def maskf( img ):
  # Define el conjunto de valores para obtener la mascara binaria
  lowLimits = (yl,crl,cbl)
  highLimits = (yh,crh,cbh)
  lowLimits = (0,0,0)
  highLimits = (100,100,100)
  # Calculo de la mascara binaria, en funcion de los limites definidos
  mask = cv2.inRange(img, lowLimits, highLimits)
  # Ajuste de mascara
  # Se define la ventana a utilizar en el filtro morfologico
  #wKernel = np.ones((3,3),np.uint8)
  # Filtro de dilatacion y erosion a la mascara
  #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, wKernel)
  return mask

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    pass


