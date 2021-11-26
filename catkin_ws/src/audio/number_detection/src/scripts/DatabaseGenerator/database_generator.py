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
#Tensorflow
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras import models
from tensorflow.keras import optimizers
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dropout, Flatten, Dense
from tensorflow.keras.layers import Conv2D, MaxPooling2D, BatchNormalization
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import tensorflow.keras.backend as K
K.clear_session()

#import keras.backend as K
#from keras import optimizers
#from keras.models import Sequential
#from keras.layers import Dropout, Flatten, Dense
#from keras.layers import Conv2D, MaxPooling2D, BatchNormalization

#from keras.preprocessing import image
#from keras_preprocessing.image import ImageDataGenerator



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

#BLOCKSIZE = 128
#RATE = 44100 #22050
RATE = 8000 #44100
BLOCKSIZE = int( RATE/333 ) #128
WIDTH = 2
CHANNELS = 1
LEN = 2 * RATE   # n seconds
MAX_SILENCE_START = 40 * 5
MAX_SILENCE_END = 50 * 5

THRESHOLD = 1
flag = True
THRESHOLD = 1

audioBuffer = []

#Image creator
def image_creator():
  print('Creating images from WAV files')
  baseAudioDir = sys.path[0] + '/dataSetNumbers'
  baseImageDir = sys.path[0] + '/dataSetNumbersPNG'
  baseAudioValid = sys.path[0] + '/samples'
  baseImageValid = sys.path[0] + '/samplesPNG'
  #baseValidDir = sys.path[0] + '/dataSetNumbersValidation'
  if os.path.isdir(baseImageValid) == 0:
    os.mkdir(baseImageValid)
  if os.path.isdir(baseImageDir) == 0:
    os.mkdir(baseImageDir)
  #Iterates 10 numbers
  for number in range(10):
    ##Base Images
    print('Current number: %s' % (number))
    print('Base set')
    #If image folder doesn't exists, creates it
    currentImageDir = baseImageDir + '/' + str(number)
    currentAudioDir = baseAudioDir + '/' + str(number)
    if os.path.isdir(currentImageDir) == 0:
      print("Creating: %s" % str(currentImageDir))
      os.mkdir(currentImageDir)
    #Takes number audio files on number folder and makes them images
    totalIm = len(os.listdir(currentAudioDir))
    currentIm = 1
    for numberAudio in os.listdir(currentAudioDir): #[:n] for n samples
      print("%s: %s of %s" % (str(number),str(currentIm), str(totalIm)))
      currentIm = currentIm + 1
      audioPath = currentAudioDir + '/' + numberAudio
      imagePath = currentImageDir + '/' + numberAudio + '.png'
      if not os.path.isfile(imagePath):
        to_png(audioPath, imagePath)

    ##Validation images
    print('Validation set')
    #If image folder doesn't exists, creates it
    currentImageDir = baseImageValid + '/' + str(number)
    currentAudioDir = baseAudioValid + '/' + str(number)
    if os.path.isdir(currentImageDir) == 0:
      print("Creating: %s" % str(currentImageDir))
      os.mkdir(currentImageDir)
    #Takes number audio files on number folder and makes them images
    totalIm = len(os.listdir(currentAudioDir))
    currentIm = 1
    for numberAudio in os.listdir(currentAudioDir): #[:n] for n samples
      print("%s: %s of %s" % (str(number),str(currentIm), str(totalIm)))
      currentIm = currentIm + 1
      audioPath = currentAudioDir + '/' + numberAudio
      imagePath = currentImageDir + '/' + numberAudio + '.png'
      if not os.path.isfile(imagePath):
        to_png(audioPath, imagePath)

  print('Training set Done')

  '''
  #Creates validation subset from images
  print('Creating validation set')
  for number in range(10):
    print('Current number: %s' % (number))
    #If validation folder doesn't exists, creates it
    currentImageDir = baseImageDir + '/' + str(number)
    currentValidDir = baseValidDir + '/' + str(number)
    if os.path.isdir(currentValidDir) == 0:
      os.mkdir(currentValidDir)
    #Empties current validation folder
    for f in os.listdir(currentValidDir):
      filePath = currentValidDir + '/' + f
      try:
        os.remove(filePath)
      except Exception as e:
        print('Failed to delete %s. %s' % (filePath,e))
    #Takes random 20% of image files
    images = os.listdir(currentImageDir)
    np.random.seed = (0)
    for f in range(int(len(images) * 0.2)):
      flag = False
      while not flag:
        index = np.random.randint(len(images))
        np.random.seed = index
        currentImageFile = currentImageDir + '/' + images[index]
        currentValidFile = currentValidDir + '/' + images[index]
        #If file is already on folder, skips
        if os.path.isfile(currentValidFile):
          pass
        #If not, tries copying it to validation folder
        else:
          try:
            flag = True
            os.popen('mv ' + currentImageFile + ' ' + currentValidFile)
          except Exception as e:
            print('Failed to move %s. %s' % (images[index],e))
  print('Validation set Done')
  '''

#Database creator
def database_generator():
  baseImageDir = sys.path[0] + '/dataSetNumbersPNG'
  baseValidDir = sys.path[0] + '/samplesPNG'
  nrow = 450
  ncol = 450
  input_shape = (nrow, ncol, 3)
  K.clear_session()
  # Create a new model
  model = Sequential()

  model.add(Conv2D(16, (7, 7), activation='relu', padding='same', input_shape=input_shape))
  model.add(MaxPooling2D((3, 3), strides=(2, 2), padding='same'))
  model.add(BatchNormalization())

  model.add(Conv2D(32, (5, 5), activation='relu', padding='same', input_shape=input_shape))
  model.add(MaxPooling2D((3, 3), strides=(2, 2), padding='same'))
  model.add(BatchNormalization())

  model.add(Conv2D(64, (3, 3), activation='relu', padding='same', input_shape=input_shape))
  model.add(MaxPooling2D((3, 3), strides=(2, 2), padding='same'))
  model.add(BatchNormalization())

  model.add(Conv2D(128, (3, 3), activation='relu', padding='same', input_shape=input_shape))
  model.add(MaxPooling2D((3, 3), strides=(2, 2), padding='same'))
  model.add(BatchNormalization())

  model.add(Conv2D(128, (3, 3), activation='relu', padding='same', input_shape=input_shape))
  model.add(MaxPooling2D((3, 3), strides=(2, 2), padding='same'))
  model.add(BatchNormalization())

  model.add(Conv2D(256, (3, 3), activation='relu', padding='same', input_shape=input_shape))
  model.add(MaxPooling2D((3, 3), strides=(2, 2), padding='same'))
  model.add(BatchNormalization())

  model.add(Flatten())
  model.add(Dense(1024, activation='relu'))
  model.add(BatchNormalization())
  model.add(Dropout(0.5))

  model.add(Dense(10, activation='softmax'))

  model.summary()
  #Train data
  train_data_dir = baseImageDir
  batch_size_tr = 32
  train_datagen = ImageDataGenerator(rescale=1./255,
                                     shear_range=0,
                                     zoom_range=0,
                                     horizontal_flip=False)
  train_generator = train_datagen.flow_from_directory(
                          train_data_dir,
                          target_size=(nrow,ncol),
                          batch_size=batch_size_tr,
                          class_mode='sparse')
  #Validation Data
  test_data_dir = baseValidDir
  batch_size_ts = 5
  test_datagen = ImageDataGenerator(rescale=1./255,
                                     shear_range=0,
                                     zoom_range=0,
                                     horizontal_flip=False)
  test_generator = test_datagen.flow_from_directory(
                          test_data_dir,
                          target_size=(nrow,ncol),
                          batch_size=batch_size_ts,
                          class_mode='sparse')

  model.compile(optimizer=optimizers.Adam(lr=0.0005),
                loss='sparse_categorical_crossentropy',
                metrics=['accuracy'])

  steps_per_epoch =  train_generator.n // batch_size_tr
  validation_steps =  test_generator.n // batch_size_ts

  nepochs = 10  # Number of epochs

  model.fit_generator(
      train_generator,
      steps_per_epoch = steps_per_epoch,
      epochs = nepochs,
      validation_data = test_generator,
      validation_steps = validation_steps)

  # Save the model
  model.save(sys.path[0] + '/mfcc_cnn_model_all.h5')
  return


def save_sample(sample):
  output_value = np.array(sample)
  output_value = output_value.astype(int)
  output_string = struct.pack('h' * BLOCKSIZE, *output_value)
  return output_string

def input_detected_end(data, THRESHOLD):
  sum_squares = 0.0
  for a in data:
    n = a / 32768.0
    sum_squares += n*n
  return math.sqrt(sum_squares / (BLOCKSIZE / 2)) > THRESHOLD

def input_detected_start(currentBuffer, THRESHOLD):
  samples = 5
  bufferCopy = currentBuffer[:]
  volume = 0
  for i in range (samples):
    data = bufferCopy.pop()
    sum_squares = 0.0
    for a in data:
      n = a / 32768.0
      sum_squares += n*n
    volume += math.sqrt(sum_squares / (BLOCKSIZE / 2))
  volume = volume / samples
  return volume > THRESHOLD

def def_threshold(thresholdBuffer):
  samples = len(thresholdBuffer)
  amp = 0
  while len(thresholdBuffer) > 0:
    data = thresholdBuffer.pop(0)
    sum_squares = 0.0
    for a in data:
      n = a / 32768.0
      sum_squares += n*n
    amp += math.sqrt(sum_squares / (BLOCKSIZE / 2))
  amp = amp / samples
  return (amp) * ( 1 + (2 * ( 1 - amp )))
  #return math.sqrt(sum_squares / (BLOCKSIZE / 2)) * 2

def flag_callback(currentFlag):
  global flag
  flag = currentFlag.data

def audio_callback(currentSample):
  global audioBuffer
  global flag
  if flag:
    audioBuffer.append(currentSample.data)
    if len(audioBuffer) > 800:
            audioBuffer.pop(0)


#Make PNG file with spectrogram and wave
def signal_interval(SampleRate, data_f, step, step_coef):
  #Points where input is over std deviation times 0.6
  over_std = np.where( abs(data_f) > data_f.std() * 0.8 )[0]
  N, = over_std.shape

  #Array of 0-1, where 0 is a gap
  over_std_cont = np.zeros ((len(data_f),1))

  n = 0
  while (n < N-step):
    #If there isn't a gap larger than step*step_coef
    if over_std[n+step] - over_std[n] <= step*step_coef:
      over_std_cont[over_std[n]:over_std[n+step]] = 1
    n+=1
  diff = over_std_cont[:-1] - over_std_cont[1:]
  starts = np.where (diff==-1)[0] 
  ends = np.where (diff==1)[0]
    
  if len(starts)<len(ends):
    #starts.insert(0, 0) #first start will be the beginning of the trace
    starts = np.insert(starts, 0, 0)
  elif len(starts)>len(ends):
    ends.insert(len(data), -1) #last start is end of trace
  durations = ends-starts
  start_p = starts[durations.argmax()]
  end_p = ends[durations.argmax()]    
  return start_p, end_p


def to_png(wav_file, image_file):
  SAMPLE_SIZE = 2048
  HOP_RATE = 16; 
  HOP = SAMPLE_SIZE / HOP_RATE

  #HOP = BLOCKSIZE
  #HOP_RATE = 16
  #SAMPLE_SIZE = HOP * HOP_RATE

  #Instantiate functions
  w = Windowing(type = 'hann')
  spectrum = Spectrum()
  mfcc = MFCC(numberBands = 120, numberCoefficients = 120,highFrequencyBound = 8192)
  loader = MonoLoader(filename = wav_file)
  logNorm = UnaryOperator(type='log')

  #Load audio
  audio = loader()
  print(len(audio))
  #Audio trimm
  start_p, end_p = signal_interval(RATE, audio, 10, 80)
  audio = audio[start_p:end_p]

  #Iterative process
  frame = []
  complete_spec = []
  total_samples = (len(audio)  - HOP * (HOP_RATE - 1)) / HOP
  if total_samples < 1:
    return
  for num in range(total_samples):
  #for frame in FrameGenerator(audio,frameSize = 1102, hopSize = 441, startFromZero = True, validFrameThresholdRatio = 1):
    frame = audio[num * HOP : num * HOP + SAMPLE_SIZE]
    #MFCC extraction for each frame
    spec = spectrum(w(frame))
    mfcc_bands, mfcc_coeffs = mfcc(spec)
    mfcc_bands_log = logNorm(mfcc_bands)
    #Determining max and min frecquency
    
    #Saving frame's MFCCs
    complete_spec.append(mfcc_bands_log)
  #print(len(complete_spec[0]),len(complete_spec))
  #Plotting
  num_samples = np.mgrid[0:len(complete_spec)]
  num_frec = np.mgrid[0:len(complete_spec[0])]
  plt.pcolormesh(num_samples, num_frec, np.transpose(complete_spec))
  #Margins
  plt.axis('off')
  plt.gca().set_axis_off()
  plt.subplots_adjust(top = 1, bottom = 0, right = 1, left = 0, hspace = 0, wspace = 0)
  plt.margins(0.0)
  plt.gca().xaxis.set_major_locator(plt.NullLocator())
  plt.gca().yaxis.set_major_locator(plt.NullLocator())
  plt.xlim(num_samples[0],num_samples[-1])
  plt.ylim(num_frec[0],num_frec[-25])
  #Save Image
  plt.savefig(image_file,bbox_inches='tight',pad_inches = 0)
  plt.close('all')
  return(1)
  
  

if __name__ == '__main__':
  try:
    image_creator()
    database_generator()
  except rospy.ROSInterruptException:
    pass
  
