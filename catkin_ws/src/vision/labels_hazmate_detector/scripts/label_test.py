import numpy as np
import cv2
from matplotlib import pyplot as plt

img = cv2.imread('hlabel/dan_when_we.jpg')

# Initiate STAR detector
orb = cv2.ORB_create()

kp, des = orb.detectAndCompute(img,None)

# draw only keypoints location,not size and orientation
img2 = cv2.drawKeypoints(img,kp,None,color=(0,255,0), flags=0)
plt.imshow(img2),plt.show()
