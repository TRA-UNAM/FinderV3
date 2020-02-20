#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
image = []
fondo = []

#Imagen con numeros para colocar en la imagen (en la misma carpeta) y obtener una mascara
numeros = cv2.imread('Numeros.png')
mask_inv = cv2.threshold(cv2.cvtColor(numeros, cv2.COLOR_BGR2GRAY), 200, 255, cv2.THRESH_BINARY)[1]
#mask_inv = cv2.erode(mask_inv, None, iterations=10)
mask = cv2.bitwise_not(mask_inv)

def movement_detection_node():
	global image
	rospy.init_node('movement_detection_node')
	rospy.Subscriber('/camera0/usb_cam0/image_raw/compressed', CompressedImage, callback)
	pub = rospy.Publisher('/movimiento', CompressedImage, queue_size=10)
	#pub = rospy.Publisher('/movimiento', Image, queue_size=10)
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
		#Se duplica la imagen de la camara para dibujar los resultados sobre la copia
		img = image.copy()
		#Se obtiene imagen en b/n y se aplica un filtro paso bajas
		gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		gris = cv2.GaussianBlur(gris,(5,5),0)
		#Diferencia entre imagen anterior y reciente
		resta = cv2.absdiff(fondo, gris)
		# La imagen se pasa a blanco y negro con un umbral
		umbral = cv2.threshold(resta, 10, 255, cv2.THRESH_BINARY)[1]
		#Erosionamos el umbral para quitar ruido
		umbral = cv2.erode(umbral, None, iterations=2)
		# Dilatamos el umbral para tapar agujeros
		umbral = cv2.dilate(umbral, None, iterations=20)
		# Copiamos el umbral para detectar los contornos
		contornosimg = umbral.copy()
		#Se buscan los contornos de los elementos encontrados
		contornos, hier = cv2.findContours(contornosimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		num_targets = 0

		for c in contornos:
			# Eliminamos los contornos mas pequenos
			if cv2.contourArea(c) < 5000:
				continue
			# Obtenemos el bounds del contorno, el rectangulo mayor que engloba al contorno
			(x, y, we, hi) = cv2.boundingRect(c)
			# Dibujamos el rectangulo del bounds
			img = cv2.rectangle(img, (x, y), (x + we, y + hi), (255, 0, 0), 2)
			#Contamos el numero de objetivos
			num_targets = num_targets+1

		#Se dibuja el numero de objetivos encontrados cada 3 frames
		#cuenta_frames = cuenta_frames +1
		#if (cuenta_frames < 4):
		#	buffer_obj = buffer_obj + num_targets
		#else:
		#	prom_obj = buffer_obj/cuenta_frames
		#	cuenta_frames = 0
		#	buffer_obj = 0

		if(num_targets > 9):
			num_targets = 9
		#Imprime el numero de objetos reconocidos en la imagen
		img = print_num(img,num_targets)

		#Para publicar imagen imgmsg
		#showimg = bridge.cv2_to_imgmsg(img, "bgr8")
		#Publicar umbral b/n
		#showimg = bridge.cv2_to_imgmsg(umbral, "mono8")
		#pub.publish(showimg)

		#Para publicar imagen comprimida
		msg = bridge.cv2_to_compressed_imgmsg(img)
		pub.publish(msg)
		#Establece como fondo la imagen anterior
		fondo = gris
		#Pausa en funcion de los fps
		rate.sleep()



def callback(img):
	global image
	np_arr = np.fromstring(img.data, np.uint8)
	image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

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

def codificar_descod(img):
	#Para codificar a compressedimagemessage
	#y decodificar a cv2 (solo para verificar posibilidad de
	#descompresion de la imagen)
	msg = bridge.cv2_to_compressed_imgmsg(img)
	np_arr = np.fromstring(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	return img

if __name__ == '__main__':
	try:
		movement_detection_node()
	except rospy.ROSInterruptException:
		pass
