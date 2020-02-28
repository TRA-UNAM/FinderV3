#!/bin/bash


#Link para instalar OPENCV 
#https://cyaninfinite.com/installing-opencv-in-ubuntu-for-python-3/
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python3.6-dev
echo "Check the following paths...(need to be the same)"
python3.6-config --includes
cd ~/
mkdir OpenCV-tmp
cd OpenCV-tmp
git clone https://github.com/Itseez/opencv.git
mv opencv opencv-3
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ../opencv-3
make -j $(nproc --all)
sudo make install
sudo apt-get install python3-numpy
sudo apt-get install python3-matplotlib
cd ~/
echo "done"

sudo apt-get install ros-melodic-cv-camera

#File to install the eBUS_SDK for thermal camera (Preguntar a Nacho sobre el proceso completo)
cd ~/FinderV3/ToInstall/
sudo ./eBUS_SDK_4.1.7.3988_Ubuntu-14.04-x86_64.run
