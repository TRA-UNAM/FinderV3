#!/bin/bash


#Link para instalar OPENCV 
#https://cyaninfinite.com/installing-opencv-in-ubuntu-for-python-3/
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python3.6-dev

#Librerías para visión
sudo apt-get install opencv-data
sudo apt-get install python-opencv
sudo apt-get install python3-opencv
sudo apt-get install ros-melodic-cv-camera
#Librería código QR
sudo apt-get install python-zbar

sudo apt-get install python3-numpy
sudo apt-get install python3-matplotlib
cd ~/
echo "done"

#File to install the eBUS_SDK for thermal camera (Preguntar a Nacho sobre el proceso completo)
cd ~/FinderV3/ToInstall/
sudo ./eBUS_SDK_4.1.7.3988_Ubuntu-14.04-x86_64.run
