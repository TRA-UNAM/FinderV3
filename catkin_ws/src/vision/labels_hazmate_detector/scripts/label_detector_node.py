#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import pickle
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
image = []
fondo = []

#umbrales del detector de bordes
thhc = 110
thlc = 70

#Instancia el extractor de caracteristicas ORB
orb = cv2.ORB_create()

#Parametros para el comparador FLANN(Fast Library for Approximate Nearest Neighbour)
FLANN_INDEX_LSH = 6
index_params= dict(algorithm = FLANN_INDEX_LSH,
                   table_number = 6, # 12
                   key_size = 12,     # 20
                   multi_probe_level = 1) #2
search_params = dict(checks=50)
#Instancia el comparador que obtendra' matches entre las imagenes
flann = cv2.FlannBasedMatcher(index_params,search_params)

#Imagenes a identificar (database)
with open('database_full.pkl','rb') as database_full:
        database_full = pickle.load(database_full)
database = database_full[0]
kp_database = []
name_database = database_full[1]
des_database = database_full[2]

#img1 = cv2.imread('hlabel/dan_when_weh1.JPG')
#img2 = cv2.imread('hlabel/organ_peroxh.JPG')
#database = [img1, img2]


#Busca KeyPoints y Descriptores en la database
#kp1, des1 = orb.detectAndCompute(img1,None)
#kp2, des2 = orb.detectAndCompute(img2,None)
#kp_database = [kp1, kp2]
#des_database  = [des1, des2]

def callback(img):
        global image
        np_arr = np.fromstring(img.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def label_detection_node():
        global image
        rospy.init_node('label_detection_node')
        rospy.Subscriber('/camera0/usb_cam0/image_raw/compressed', CompressedImage, callback)
        pub = rospy.Publisher('/detected_labels', String, queue_size=10)
	#fps
	rate = rospy.Rate(1)
	#print(kp1[1])
	#print(des1[1])
	#print(len(kp1))
	#print(len(des1))
	rate.sleep()
        while not rospy.is_shutdown():
		#Obtencion y pre procesado
		imgc = image.copy()
		imgwg = cv2.cvtColor(imgc, cv2.COLOR_BGR2GRAY)
		clahe = cv2.createCLAHE(clipLimit=4.0, tileGridSize=(8,8))
		imgwg = clahe.apply(imgwg)
		imgwg = cv2.bilateralFilter(imgwg, 5, 150, 150)
		#Busca ejes con el algoritmo de canny
		edges = cv2.Canny(imgwg, thlc, thhc)
		# Busca informacion sobre cuadrados contenidos en la imagen
		# x,y,w,h, area por cada cuadrado
		#print('la wa partir')
		cent,totR = findRoi(edges.copy(), imgc)
		#print('centros')
		#print(cent)
		#roi = [None] * 5
		roi = np.empty(10, dtype=object)
		#kpc = [None] * 10
		kp_roi = np.empty(10, dtype=object)
		#desc = [None] * 10
		des_roi = np.empty(10, dtype=object)

		#Cuenta de los matches de cada recuadro encontrado
		#Cuenta del mejor match y registro de cual fue
		totM = np.empty(10, dtype=object)
		bestM = np.empty(10, dtype=object)
		matches = np.empty([len(cent),len(des_database)], dtype=object)

		if len(cent):
			#Crea una roi por cada uno de recuadros mas grandes
			for i in range (totR):
				#print(i)
				roi[i] = imgc[ ( cent[i][3] ):( cent[i][3]+cent[i][5] ), ( cent[i][2] ):( cent[i][2]+cent[i][4] ) ]
			# find the keypoints and descriptors with ORB
			#print('la wa checar..')

			#Barrido de todas las ROI
			for roi_index in range (len(cent)):
				#Inicializacion del numero de matches y del mejor match
				bestM[roi_index] = 0
				totM[roi_index] = 0
				#Obtencion de KeyPoints de la ROI
				kp_roi[roi_index], des_roi[roi_index] = orb.detectAndCompute(roi[roi_index],None)
				#Comparativa (matches) de los KeyPoints de la ROI con la base de datos
				for database_index in range(len(des_database)):
					matches[roi_index,database_index] = flann.knnMatch(des_roi[roi_index],des_database[database_index],k=2)
					#Recorre cada punto que hizo match de la ROI con la imagen de la database actual, y verifica que haya al menos uno
					#variable para guardar el numero de matches con la imagen actual
					totM_temp = 0
					for poss_match in enumerate(matches[roi_index,database_index]):
						if len(poss_match[1]) > 1:
							#Aumenta el numero de matches correctos si la distancia entre los puntos es menor a un valor
							if poss_match[1][0].distance < 0.7*poss_match[1][1].distance:
								totM_temp = totM_temp + 1
					#Termina de calcular los matches para una imagen y compara si es la imagen con mas match
					if totM_temp > totM[roi_index]:
						totM[roi_index] = totM_temp
						bestM[roi_index] = database_index
				#Termina de calcular los match de una ROI con la database completa
				#Obtuvo el mejor match y su indice
				if totM[roi_index] < 20:
					totM[roi_index] = None
					bestM[roi_index] = None
				if bestM[roi_index] is not None:
					Found_label = String()
					Found_label.data = name_database[bestM[roi_index]]
					pub.publish(Found_label)
					#cv2.imshow('Roi%d'%roi_index,roi[roi_index])
					cv2.imshow(name_database[bestM[roi_index]],database[bestM[roi_index]])
					cv2.waitKey(1)
					print(name_database[bestM[roi_index]])
			#Termina de evaluar las ROI
			#print('Total de coincidencias')
			#print(totM)
			#print(bestM)

#		if totM[0] is not None:
#			#cv2.imshow('Roi1',roi[0])
#			cv2.imshow('image1',database[bestM[0]])
#			cv2.waitKey(1)
#                if totM[1] is not None:
#			#cv2.imshow('Roi2',roi[1])
#			cv2.imshow('image2',database[bestM[1]])
#			cv2.waitKey(1)

		rate.sleep()
		cv2.destroyAllWindows()




#				kpc[j], desc[j] = orb.detectAndCompute(roi[j],None)
#				matches[j] = flann.knnMatch(desc[j],des1,k=2)
#				#print(matches[1])
#				for i in enumerate(matches[j]):
#					#print(i[0])
#					if len(i[1]) > 1:
#               				if i[1][0].distance < 0.7*i[1][1].distance:
#							matchesMask[i[0]]=[1,0]
#							totM[j] = totM[j] + 1

#		draw_params = dict(matchColor = (0,255,0),singlePointColor = (255,0,0),matchesMask = matchesMask,flags = cv2.DrawMatchesFlags_DEFAULT)
		#if roi[0] is not None:
		#	img3 = cv2.drawMatchesKnn(img1,kp1,roi[0],kpc[0],matches[:20],None,draw_params)
		#	plt.imshow(img3),plt.show()
		#plt.imshow(img3),plt.show()

		# Draw first 10 matches.
		#img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:20],None,flags = 2)

# Busqueda de la region de interes, para aplicar metodos de reconocimiento.
# La funcion consiste en un buscador de contornos similares a rectangulos y
# cuadrados, recibiendo como entrada la matriz que define los contornos "img"
# y la imagen original para indicar la deteccion. Regresa una lista cuyos datos
# son [centro del poligono, coordenadas del boundingBox
# x,y,w,h, area del poligono detectado]
def findRoi(img, imgt):
  rate = rospy.Rate(5)
  # Area de referencia para validar deteccion de ROI
  areaRoiRef = 10
  # Inicializa lista para guardar los centros que se calcularan
  centers = []
  #Porcentaje de diferencia permitido en la comparacion de lados que forman
  #la region de interes candidata
  perDifh = 38
  perDifv = 36

  # Inicializacion de variables
  cX = 0
  cY = 0
  totR = 0
  square = False

  # Obtencion de segmentos, basado en la imagen binaria
  #contour, hierarchy = cv2.findContours(img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
  contour, hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  #print('cuadros tentativos:',len(contour))
  # Ordenamiento de mayor a menor de todos los segmentos encontrados
  # considerando como criterio su area.
  contours = sorted(contour, key = cv2.contourArea, reverse = True) #[:1]
  #Procesa los 10 rectangulos mas grandes (cuando la cuenta llega a 10 llama un break)
  cont_count = 0
  for c in contours:
    # Calculo del perimetro
    per = cv2.arcLength(c, True)
    # Calculo de la curva cerrada que rodea al perimetro
    approx = cv2.approxPolyDP(c, 0.1*per, True)
    # Calculo del area comprendida por el contorno
    cntArea = cv2.contourArea(c)
    #print(cntArea)
    # Analisis de proporcion de los lados que forman el contorno, verifica que la curva creada sea de 4 lados
    if(len (approx) == 4):
      #Verifica que sea un rectangulo
      # Rectas verticales del contorno
      dp1p0 = abs( np.linalg.norm(approx[1]-approx[0]) )
      dp2p3 = abs( np.linalg.norm(approx[2]-approx[3]) )
      # Rectas horizontales del contorno
      dp2p1 = abs( np.linalg.norm(approx[2]-approx[1]) )
      dp3p0 = abs( np.linalg.norm(approx[3]-approx[0]) )
      # Calculo de diferencias entre el tamano de los segmentos de recta
      difh = abs (dp2p1 - dp3p0)
      difv = abs (dp2p3 - dp1p0)
      #print(approx)
      #print(approx[0])
      #print(approx[0,0])
      #print(approx[1,0][0])
      #print(approx[1,0][0])
      #print(approx[2,0][0])
      #print(approx[1,0][1])
      #print(approx[2,0][1])

      # Calculo del angulo de los lados
      if float(approx[1,0][1]-approx[0,0][1]) == 0:
	ang_a = 90
      else:
	ang_a = math.degrees(math.atan(abs(float(approx[1,0][0]-approx[0,0][0])/float(approx[1,0][1]-approx[0,0][1]))))

      if float(approx[2,0][1]-approx[1,0][1]) == 0:
        ang_b = 90
      else:
        ang_b = math.degrees(math.atan(abs(float(approx[2,0][0]-approx[1,0][0])/float(approx[2,0][1]-approx[1,0][1]))))
      #print(ang_b)

      if float(approx[3,0][1]-approx[2,0][1]) == 0:
        ang_c = 90
      else:
        ang_c = math.degrees(math.atan(abs(float(approx[3,0][0]-approx[2,0][0])/float(approx[3,0][1]-approx[2,0][1]))))

      if float(approx[0,0][1]-approx[3,0][1]) == 0:
        ang_d = 90
      else:
        ang_d = math.degrees(math.atan(abs(float(approx[0,0][0]-approx[3,0][0])/float(approx[0,0][1]-approx[3,0][1]))))

      if( difh <= perDifh and difv <= perDifv):
        square = True
      else:
        square = False
	
#Si es cuadrado y si cumple el area minima
    if( square and (cntArea >= areaRoiRef) ):
      #cv2.imshow('ajosp',img)
      M = cv2.moments(c)
      cX = int(M["m10"]/ M["m00"])
      cY = int(M["m01"]/ M["m00"])
      #use = 1
      x,y,w,h = cv2.boundingRect(c)
      #cv2.drawContours(imgt, [approx], -1, (0,255,0), 4)
      centers.append((cX,cY,x,y,w,h,cntArea))
      totR += 1
      #centers =
    cont_count = cont_count + 1
    if cont_count == 10:
	break
  print ('No. of rects: %d'%(totR))
  return centers,totR


if __name__ == '__main__':
        try:
                label_detection_node()
        except rospy.ROSInterruptException:
                pass
