#!/usr/bin/env python
import cv2
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':

	cap = cv2.VideoCapture(0)
	ret, frame = cap.read()                                          # captura imagen
	prevgray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	grayimg = cv2.GaussianBlur(prevgray,(3,3),0)
	print('mostrando imagen')
	cv2.imshow('dsa',grayimg)
	print('img mostrada')
