#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import argparse
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import zbar

bridge = CvBridge()
scanner = zbar.ImageScanner()
scanner.parse_config('enable')

image=[]

def qr_barcode_detection_node():
	global image
	prev_data = '.'
        rospy.init_node('qr_detection_node')
        rospy.Subscriber('/camera0/usb_cam0/image_raw/compressed', CompressedImage, callback)
	qr_pub = rospy.Publisher('/qr_barcode_results', String, queue_size=10)
	#fps
	rate = rospy.Rate(1)
	rate.sleep()
	while image == [] and not rospy.is_shutdown():
		rate.sleep()
	while not rospy.is_shutdown():
		#Toma la imagen y la convierte al formato de zbar
		img = image.copy()
		img_g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		height, width = img_g.shape
		raw = str(img_g.data)
		zbar_img = zbar.Image(width,height,'Y800', raw)
		#Escanea la imagen con Zbar
		scanner.scan(zbar_img)
		for barcode in zbar_img:
			#Publica el tipo de dato (qr/barra) y lo que contiene
			result = str(barcode.type) + '-->' + barcode.data
			qr_pub.publish(result)
		rate.sleep()

def callback(img):
        global image
        np_arr = np.fromstring(img.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

if __name__ == '__main__':
        try:
                qr_barcode_detection_node()
        except rospy.ROSInterruptException:
                pass

