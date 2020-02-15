#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()
image = []
fondo = []
import numpy as np

def callback(img):
	global image
	np_arr = np.fromstring(img.data, np.uint8)
	image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def movement_detection_node():
	global image
	rospy.init_node('movement_detection_node')
	#rospy.Subscriber('/camera0/usb_cam0/image_raw/compressed', CompressedImage, callback)
	pub = rospy.Publisher('/movimiento', CompressedImage, queue_size=10)
	pub = rospy.Publisher('/movimiento', Image, queue_size=10)
	rate = rospy.Rate(5)
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

	while not rospy.is_shutdown():
		img = image
		gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		gray = cv2.GaussianBlur(gray,(5,5),0)
		gris = gray
		resta = cv2.absdiff(fondo, gris)
		frame=fondo
		# La imagen se pasa a blanco y negro con un umbral
		umbral = cv2.threshold(resta, 10, 255, cv2.THRESH_BINARY)[1]
		#Erosionamos el umbral para quitar ruido
		umbral = cv2.erode(umbral, None, iterations=5)
		# Dilatamos el umbral para tapar agujeros
		umbral = cv2.dilate(umbral, None, iterations=20)
		# Copiamos el umbral para detectar los contornos
		contornosimg = umbral.copy()

		# Buscamos contorno en la imagen
		contornos, hier = cv2.findContours(contornosimg,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		for c in contornos:
			# Eliminamos los contornos mas pequenos
			if cv2.contourArea(c) < 5000:
				continue
			#print len(c)
			# Obtenemos el bounds del contorno, el rectangulo mayor que engloba al contorno
			(x, y, we, hi) = cv2.boundingRect(c)
			# Dibujamos el rectangulo del bounds
			cv2.rectangle(frame, (x, y), (x + we, y + hi), (0, 255, 0), 2)

		#showimg = bridge.cv2_to_imgmsg(frame, "mono8")
		#pub.publish(showimg)

		#msg = CompressedImage()
		#msg.header.stamp = rospy.Time.now()
		#msg.format = "jpeg"
		#msg.data = np.array(cv2.imencode('.jpeg', image)[1]).tostring()

		msg = bridge.cv2_to_compressed_imgmsg(image,dst_format='jpg')
		pub.publish(msg)
		rate.sleep()
		fondo = gray

if __name__ == '__main__':
	try:
		movement_detection_node()
	except rospy.ROSInterruptException:
		pass


#thresd=127
#thresh=255
#Captura una primera imagen de la camara y una variable de verificacion
#ret, frame = cap.read()
# convierte imagen a color a escala de grises
#prevgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#aplica ventana de emborronamiento (filtro paso bajas)
#grayimg = cv2.GaussianBlur(prevgray,(3,3),0)
#Dimensiones de la imagen
#h,w=grayimg.shape
#fondo=None

                #Se obtiene una nueva imagen y se pre procesa
#                ret, img = cap.read()
#                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#                gray=cv2.GaussianBlur(gray,(5,5),0)
                #Copia de la imagen para su procesamiento
#                imguno = gray;
                #Se toma como fondo la imagen inicial

