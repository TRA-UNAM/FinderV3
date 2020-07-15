#!/usr/bin/env python
import rospy
import numpy as np
import pyaudio
import struct
import wave
import math
import sys
import os

import random
import tensorflow as tf
#from tensorflow.keras.models import Sequential
#from tensorflow.keras.layers import Dense, Dropout, Activation, Flatten
#from tensorflow.keras.layers import Conv2D, MaxPooling2D

import keras.backend as K
from keras import optimizers
from keras.models import Sequential
from keras.layers import Dropout, Flatten, Dense
from keras.layers import Conv2D, MaxPooling2D, BatchNormalization

from keras.preprocessing import image
from keras_preprocessing.image import ImageDataGenerator

#Para creacion de imagen desde archivo wav
import scipy.io.wavfile as wv
from matplotlib import pyplot as plt
import matplotlib.cm as cm
import scipy.signal as ss
from pylab import plot, show, figure, imshow


from essentia.standard import *
import shutil

nrow = 200
ncol = 200

BLOCKSIZE = 128

RATE = 44100 #22050
WIDTH = 2
CHANNELS = 1
LEN = 2 * RATE   # n seconds
MAX_SILENCE_START = 40 * 5
MAX_SILENCE_END = 50 * 5

THRESHOLD = 1
flag = True
THRESHOLD = 1

audioBuffer = []

from keras import models
model = models.load_model(sys.path[0] + '/mfcc_cnn_model_all.h5')
nrow = 450
ncol = 450

#Image creator
def validation():
  baseImageDir = sys.path[0] + '/dataSetNumbersValidation'
  print('Start prediction')

  for number in range(10):
    print('Current number: %s' % (number))
    currentImageDir = baseImageDir + '/' + str(number)
    #Iterates trough image samples
    tot  = 0
    pool = 0
    perc = 0
    prom = 0
    for numberAudio in os.listdir(currentImageDir):
      #deletes tmp
      for f in os.listdir(sys.path[0] + '/tmp/tmp'):
        os.remove(sys.path[0] + '/tmp/tmp/' + f)
      #copy image to tmp
      os.popen('cp ' + currentImageDir + '/' + numberAudio + ' ' + sys.path[0] + '/tmp/tmp')
      #prediction
      i, yts = predict(currentImageDir + '/' + numberAudio)
      print('Predicted as %s' % (i))
      if i == number:
        tot += 1.0
        pool += yts
    if tot > 0:
      prom = pool / tot
      perc = (tot / len(os.listdir(currentImageDir))) * 100
    print('Number %s: %s/100 Predicted correctly. Reliability of: %s/1' % (number, perc, prom))
    tot = 0
    pool = 0


def predict(image_file):
  test_datagen = ImageDataGenerator(
            rescale=1./255,
            shear_range=0,
            zoom_range=0,
            horizontal_flip=False)
  test_generator = test_datagen.flow_from_directory(
            sys.path[0] + '/tmp',
            target_size=(nrow, ncol),
            batch_size=1,
            class_mode='sparse')

  # Load the model
  Xts, _ = test_generator.next()

  # Predict the probability of each class
  yts = model.predict(Xts)
  if np.max(yts) < 0.5:
    res = None
    maxv = 0
  else:
    # Choose the most likely class
    res = np.argmax(yts)
    maxv = yts[0,res]
  return res,maxv



if __name__ == '__main__':
  try:
    validation()
  except rospy.ROSInterruptException:
    pass


