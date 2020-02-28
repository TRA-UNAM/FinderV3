#!/bin/bash


#Link para instalar OPENCV 
#https://cyaninfinite.com/installing-opencv-in-ubuntu-for-python-3/
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python3.6-dev
echo "Check the following paths..."
python3.6-config --includes
cd ~/Desktop/
mkdir OpenCV-tmp
cd OpenCV-tmp
git clone https://github.com/Itseez/opencv.git
#Agregar aqui comando para renombrar el directorio "opencv" a "opencv-3"
echo "Rename from opencv to opencv-3"
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ../opencv-3
make -j $(nproc --all)
sudo make install
sudo apt-get install python3-numpy
sudo apt-get install python3-matplotlib

