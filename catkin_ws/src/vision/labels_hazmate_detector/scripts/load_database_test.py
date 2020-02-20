#!/usr/bin/env python
import cv2
import glob
import pickle

database_full = []
database_names = []
kp_database = []
des_database = []

with open('database_full.pkl','rb') as database_full:
	database_full = pickle.load(database_full)
print(len(database_full))
print(len(database_full[2]))
print(database_full[2][0])
print(database_full[1])
cv2.imshow('123',cv2.resize(database_full[0][0],(500,500)))
cv2.waitKey(0)
