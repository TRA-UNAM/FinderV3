#!/bin/bash


#Link para instalar OPENCV 
#https://cyaninfinite.com/installing-opencv-in-ubuntu-for-python-3/
sudo apt-get update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install python3.6-dev

#Paquetes para visión
sudo apt-get install ros-melodic-usb-cam
sudo apt-get install opencv-data
sudo apt-get install python-opencv
sudo apt-get install python3-opencv
sudo apt-get install ros-melodic-cv-camera #1

#Paquetes audio
sudo apt-get install python-tqdm
sudo apt-get install python-pip
sudo apt-get install python-pyaudio
sudo apt-get install python-matplotlib
sudo apt-get install python-pip
pip install tensorflow
pip install keras
pip install wave
pip install essentia
pip install -q pyyaml h5py
pip install gdown

#Paquetes código QR
sudo apt-get install python-zbar
sudo apt-get install python3-numpy
sudo apt-get install python3-matplotlib

#Paqutes interfaz grafica
sudo apt-get install qtmultimedia5-dev #3

#Paquetes para simulador
sudo apt-get install ros-melodic-urdf ros-melodic-xacro ros-melodic-effort-controllers ros-melodic-ros-controllers

#Paquetes para SLAM
sudo apt-get install ros-melodic-map-server ros-melodic-gmapping ros-melodic-hector-slam

#Serial devices settings
sudo cp ~/FinderV3/ToInstall/90-finder.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

#Paquetes de navegación
sudo apt-get install ros-melodic-amcl
sudo apt-get install ros-melodic-base-local-planner
sudo apt-get install ros-melodic-move-base

#Descarga modelo de IA entrenado para detección de audio
gdown --id 19Bopb9qhcGjwvkn2pmlFt2xWOQX3lxWz --output ~/FinderV3/catkin_ws/src/audio/number_detection/src/scripts/mfcc_cnn_model_all.h5

cd ~/
echo "done"


#File to install the eBUS_SDK for thermal camera (Preguntar a Nacho sobre el proceso completo)
cd ~/FinderV3/ToInstall/
sudo ./eBUS_SDK_4.1.7.3988_Ubuntu-14.04-x86_64.run
