#!/usr/bin/env python
import cv2
import glob
import pickle
import os
import sys

database = []
database_names = []
kp_database = []
des_database = []

#Instancia el extractor de caracteristicas ORB
orb = cv2.ORB_create()

#Genera la database con:
#Archivo, nombre y caracteristicas
Total = len(glob.glob(sys.path[0] + '/Hazmat2020_jpg/*.jpg'))
Actual = 0
print('Creando base de datos. Sólo es necesario hacerlo una vez')
for filename in glob.glob(sys.path[0] + '/Hazmat2020_jpg/*.jpg'):
	Actual = Actual+1
	print('Procesando: %d / %d' %(Actual,Total))
	im = cv2.resize(cv2.imread(filename),(400,400))
	#cv2.imshow('asd',im)
	#cv2.waitKey(0)
	#print(len(im))
	#Busca KeyPoints y Descriptores en el archivo
	kp_temp, des_temp = orb.detectAndCompute(im,None)
	#Coloca los archivos en la database
	database.append(im)
	database_names.append(filename)
	#kp_database.append(kp_temp)
	des_database.append(des_temp)
#Borra la base de datos anterior
os.remove(sys.path[0] + '/database_full.pkl')
#Guarda los objetos
print('Guardando en base de datos')
with open(sys.path[0] + '/database_full.pkl', 'wb') as database_full:
    pickle.dump([database, database_names, des_database], database_full)
print('Base de datos completada')
