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

image = []

def prevFilter(img):
        # Ventana para aplicar la el filtro de dilatcion y erosion
        wKernel = np.ones((3,3),np.uint8)
        # Aplicacion del filtros morfologicos, Dilate y posteriormente erosiona
        fImg = cv2.morphologyEx(img, cv2.MORPH_CLOSE, wKernel)
        # Filtro mediana de suavizamiento de imagen
        fImg = cv2.medianBlur(fImg, 5)
        #cv2.imshow('ImgFiltering',fImg)
        return fImg


#  Funcion para obtener la mascara binaria, a partir del color de interes especificado
def maskf( img ):
        # Define el conjunto de valores para obtener la mascara binaria
        lowLimits = (yl,crl,cbl)
        highLimits = (yh,crh,cbh)
        # Calculo de la mascara binaria, en funcion de los limites definidos
        mask = cv2.inRange(img, lowLimits, highLimits)
        # Ajuste de mascara
        # Se define la ventana a utilizar en el filtro morfologico
        #wKernel = np.ones((3,3),np.uint8)
        # Filtro de dilatacion y erosion a la mascara
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, wKernel)

        return mask

def pmedio(cod_point1, cod_point2):
  if ( cod_point1[0] < cod_point2[0] ):
    xc = cod_point1[0] + ( cod_point2[0] - cod_point1[0] )*0.5
  else:
    xc = cod_point2[0] + ( cod_point1[0] - cod_point2[0] )*0.5
  
  if ( cod_point1[1] < cod_point2[1] ):
    yc = cod_point1[1] + ( cod_point2[1] - cod_point1[1] )*0.5
  else:
    yc = cod_point2[1] + ( cod_point1[1] - cod_point2[1] )*0.5
  
  return xc,yc

# Funcion para el calculo de contornos de una imagen con un solo canal. El proposito
# de esta imagen es encontrar el contorno del patron C
def canny_detector(image, sigma=0.33):
  # Calcula la intensidad media de los pixeles de la imagen de entrada
  v = np.median(image)
  # Calculo de los umbrales para el detector canny, en funcion de la media y un delta
  lower = int(max(0, (1.0 - sigma) * v))
  upper = int(min(255, (1.0 + sigma) * v))
  # Obtencion de los contornos
  edged = cv2.Canny(image, lower, upper)
  
  return edged

def recAber(colororig, original, imagen, mind,x0,y0):
  # Numero de esquinas deseadas a obtener
  Nesq = 3
  #print '----'
  # parametro del operador canny
  cal = 32
  cal=cal/255.0
  #mind = 0.1

  # numero de esquinas por buscar
  #cv2.bitwise_not ( imagen,imagen )
  #cv2.imshow('imagen',imagen)

  # Detecta las esquinas en imagen
  corners = cv2.goodFeaturesToTrack(imagen, Nesq, cal, mind)
  # Conversion del vector de coordenadas flotantes a enteros de 8 bits
  corners = np.int0(corners)
  #print corners
  
  # Inicializacion de variables
  x,y=0,0
  coor=[]

  # coloca las coordenadas de las esquinas en un solo arreglo
  for i in corners:
    # Separa los datos de cada elemento en el vector de corners
    x,y = i.ravel()
    # Junta las coordenadas de cada esquina en una lista
    coor.append((x,y))

    ## Esta parte se encarga de ver cual segmento de recta entre los tres puntos tiene la maxima
    ## distancia y almacena el valor del punto medio de la recta mayor en xc yc
    if  1<len(coor)<4:
      # Calculo de la distancia entre las esquinas detectadas de la apertura del patron
      d1 = math.sqrt( (coor[0][0] -coor[1][0] )**2 + (coor[0][1] -coor[1][1] )**2 )    #1,2

      # Calculo del punto medio de la recta que une a las dos esquinas detectadas
      xc,yc = pmedio( coor[0], coor[1] )

      ##  Dibuja en la imagen las esquinas localizadas, y el punto donde
      ##  se encuentre la abertura
      #imagenC = cv2.cvtColor(imagen, cv2.COLOR_GRAY2BGR)
      #cv2.circle(imagenC,(coor[0][0],coor[0][1]),2,(0,0,255),-1)
      #cv2.circle(imagenC,(coor[1][0],coor[1][1]),2,(0,255,0),-1)
      #cv2.circle(imagenC,(coor[2][0],coor[2][1]),2,(255,0,0),-1)

      #cv2.circle(imagenC,(int(xc),int(yc)),5,(100,100,100),-1)
      cv2.circle(colororig,(int(xc), int(yc)), 5, (0,0,255), -1)
      #cv2.circle(colororig,(int(x0),int(y0)),5,(255,0,0),-1)

      x1 = float(xc)
      y1 = float(yc)
      x0 = float(x0)
      y0 = float(y0)
      sx = abs(x0-x1)
      sy = abs(y0-y1)

      # dependiendo del cuadrante donde se encuentre la apertura, se suma o resta angulo
      if x1<x0:
        argum=sy/sx
        angulo=math.atan(argum)
        texto=str(angulo*180/math.pi)

        if y1<y0:
          angulo=math.pi-angulo
          texto=str(angulo*180/math.pi)
        elif y1>y0:
          angulo=math.pi+angulo
          texto=str(angulo*180/math.pi)
        elif y1==y0:
          angulo=math.pi
          texto=str(angulo*180/math.pi)

      elif x1>x0:
        argum=sy/sx
        angulo=math.atan(argum)
        texto=str(angulo*180/math.pi)

        if y1>y0:
          angulo=2*math.pi-angulo
          texto=str(angulo*180/math.pi)

      elif x1==x0:
        if y1<y0:
          angulo=(math.pi)/2
          texto=str(angulo*180/math.pi)

        if y1>y0:
          angulo=(math.pi/2)
          texto=str(angulo*180/math.pi)

        #else:
          #texto='c'

      font = cv2.FONT_HERSHEY_SIMPLEX
      #coloca el texto en la imagen original
      cv2.putText(colororig,texto, (int(xc)+3,int(yc)+3), font, 1, (0,255,0))
  return (angulo, colororig)

def main():
    global image
    rospy.init_node('cPattern_detection_node')
    rospy.Subscriber('/camera2/usb_cam2/image_raw/compressed', CompressedImage, callback)
    pub = rospy.Publisher('/cPattern_info', String, queue_size=10)
    rate = rospy.Rate(10)
    rate.sleep()

    #cv2.imshow('data',data)
    # Our operations on the frame come here
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    while not rospy.is_shutdown():
	    rate.sleep()
	    data = image.copy()
	    yuvImg = cv2.cvtColor(data.copy(), cv2.COLOR_BGR2YCR_CB)
	    fImg = prevFilter(yuvImg)
	    yuvMask = maskf( fImg )
	    # Display the resulting frame
	    #edges=cv2.Canny(gray,50,150,apertureSize=3)
	    #circles=cv2.HoughCircles(edges,cv2.HOUGH_GRADIENT,dp=1,minDist=20,param1=50,param2=30,minRadius=50,maxRadius=80)
	    circlei = cv2.HoughCircles(yuvMask,cv2.HOUGH_GRADIENT,1,20,param1=100,param2=30,minRadius=0,maxRadius=0)
	    imguno = yuvMask

	    try:
	        #if circles!= None:   
	        circlei=sorted(circlei[0],key=lambda x:x[2],reverse=True)
	        mask0 = np.ones(yuvMask.shape,np.uint8)
	        radio = int(1.1*circlei[0][2])                                # obtiene el radio del circulo mayor
	        cv2.circle(mask0,(circlei[0][0],circlei[0][1]),radio,(0,0,0),-1)  # dibuja el circulo mayor en la mascara
	        fg1 = cv2.bitwise_or( yuvMask, mask0 )                           # segmenta la imagen YUV con la mascara
	        #cv2.imshow(fg1)
	        contours, hier = cv2.findContours(fg1,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	        #cv2.imshow('hier',hier)
	        #print('hier   ',hier)
	        if len(hier)<9:
	            for h,cnt in enumerate(contours):
	                mask = np.zeros(imguno.shape,np.uint8)
	                mask = cv2.bitwise_not(mask )
	                cv2.drawContours(mask,[cnt],0,0,-1)                   # dibuja en mask el contorno actual
	                circlei = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
	                #cv2.imshow('mask',mask)
	                try:

	                    circles=sorted(circlei[0],key=lambda x:x[2],reverse=True)
	                    rmax=500
	                    for i in circles:
	                        if i[2]<rmax:
	                            rmax=i[2]
	                            mind= rmax/10
	                            (ang, imgRes) = recAber(data.copy() ,fg1, mask, mind, i[0],i[1])
################################Publicar esta imagen
	                            ##cv2.imshow('imgRes',imgRes)

	                except Exception as e:
	                        pass
	    except Exception as e:
	        pass
	    
	    if cv2.waitKey(1) & 0xFF == ord('q'):
	        return True

def callback(img):
        global image
        np_arr = np.fromstring(img.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
	pass
