#!/usr/bin/env python
import rospy
import numpy as np
import pyaudio
import struct
import wave
import math
import sys

import random
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Activation, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D

from keras.preprocessing import image
from keras_preprocessing.image import ImageDataGenerator

#Para creacion de imagen desde archivo wav
import scipy.io.wavfile as wv
from matplotlib import pyplot as plt
import matplotlib.cm as cm
import scipy.signal as ss


#ROS MSGS
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from std_msgs.msg import Bool


import keras.backend as K
K.clear_session()

#import librosa.display
#import librosa.feature
#from keras import models
#import matplotlib.pyplot as plt
#from keras.preprocessing.image import ImageDataGenerator

nrow = 200
ncol = 200

BLOCKSIZE = 128

RATE = 22050
WIDTH = 2
CHANNELS = 1
LEN = 2 * RATE   # n seconds
MAX_SILENCE_START = 40
MAX_SILENCE_END = 50

#Import Database
model = tf.keras.models.load_model(sys.path[0] + '/sp_recog.h5')

THRESHOLD = 1
flag = True
THRESHOLD = 1

audioBuffer = []
pred_dict = {0:0, 1:1, 2:10, 3:11, 4:12, 5:2, 6:3, 7:4, 8:5 , 9:6 , 10:7, 11:8, 12:9}

def number_recognition_node():
  global flag
  global audioBuffer
  sample = None
  stop = False
  rospy.init_node('number_recognition_node')
  pub = rospy.Publisher('/audio/detected_numbers', Int16, queue_size=1)
  rospy.Subscriber('/audio/mic_samples', Int16MultiArray, audio_callback)
  rospy.Subscriber('/audio/detection_flag', Bool, flag_callback)
  #Block samples / second
  rate = rospy.Rate(int(2*(RATE/BLOCKSIZE)))
  #Constantly publishes number detected
  while not rospy.is_shutdown():
    #If the audio recognition flag is up, starts recognition
    if flag and audioBuffer != []:
      #Published message
      message = Int16()
      #Starts defining threshold value with 20 samples
      print('Defining threshold')
      prevAudioBuffer = []
      while len(prevAudioBuffer) < 20 and not rospy.is_shutdown():
        rate.sleep()
        if audioBuffer != []:
          prevAudioBuffer.append(audioBuffer.pop(0))
      THRESHOLD = def_threshold(prevAudioBuffer)
      print('Threshold done')
      # Wait until voice detected. Once detected, breaks the loop
      # Also saves up to n prevSamples samples before threshold is reached
      prevAudioBuffer = []
      while True and not rospy.is_shutdown():
        rate.sleep()
        if audioBuffer != []:
          sample = audioBuffer.pop(0)
          prevAudioBuffer.append(sample)
          if len(prevAudioBuffer) > MAX_SILENCE_START + 5:
            prevAudioBuffer.pop(0)
            #Test 5 last samples with threshold to detect input
            if input_detected_start(prevAudioBuffer, THRESHOLD):
              break
      print('Input detected')
      #Once an input is detected, threshold is adjusted down 
      THRESHOLD = THRESHOLD * 0.75
      #Output wave file is oppened
      output_wf = wave.open(sys.path[0] + '/myNumber.wav', 'wb')
      output_wf.setframerate(RATE)
      output_wf.setsampwidth(WIDTH)
      output_wf.setnchannels(CHANNELS)
      #All samples are saved
      while len(prevAudioBuffer) > 1:
        output_wf.writeframes(save_sample(prevAudioBuffer.pop(0)))
      #Starts recording until sound is under threshold for a fixed time
      silence_count = 0
      for n in range(0, int(LEN / BLOCKSIZE)):
        #Takes current sample and saves it 
        output_wf.writeframes(save_sample(sample))
        #Waiting for next sample
        while audioBuffer == [] and not rospy.is_shutdown():
          pass
        #Takes new sample
        sample = audioBuffer.pop(0)
        #Verifies volume of new sample for MAX_SILENCE_END samples up to LEN seconds
        if not input_detected_end(sample, THRESHOLD):
          silence_count += 1
          if silence_count == MAX_SILENCE_END:
            print('Input stopped')
            break
        else:
          silence_count = 0
      #CNN prediction
      try:
        SampleRate,data = wv.read(sys.path[0] + '/myNumber.wav')
        to_png(SampleRate, data, sys.path[0] + '/myNumber.png')
        message.data = predict(sys.path[0] + '/myNumber.png')
        print(message.data)
        pub.publish(message)
      except:
        print('Short sample')
      #Closes WAV file
      output_wf.close()
      #Cleans audio buffer
      audioBuffer = []

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
def signal_interval(SampleRate, data_f, step, step_coef, file):
    over_std = np.where(abs(data_f)>data_f.std())[0]
    N, = over_std.shape
    
    over_std_cont = np.zeros ((len(data_f),1))
    n = 0
    while (n<N-step):
      if over_std[n+step]-over_std[n]<=step*step_coef: #where is no gap
        over_std_cont[over_std[n]:over_std[n+step]]=1
      n+=1
    diff = over_std_cont[:-1]-over_std_cont[1:]
    starts = np.where (diff==-1)[0] 
    ends = np.where (diff==1)[0]
    
    if len(starts)<len(ends):
      starts.insert(0, 0) #first start will be the beginning of the trace
    elif len(starts)>len(ends):
      ends.insert(len(data), -1) #last start is end of trace
    durations = ends-starts
    start_p = starts[durations.argmax()]
    end_p = ends[durations.argmax()]    
    return start_p, end_p

def to_png(SampleRate, data, file):
    #filtering
    f_cutoff = [float(500), float(3200)]
    Wn = [f_cutoff[0] / (SampleRate/2) , f_cutoff[1] / (SampleRate/2)]
    b, a = ss.iirfilter(1, Wn, ftype='butter', btype='bandpass')
    data_f = ss.filtfilt(b, a, data)
    #start and end of signal
    start_p, end_p = signal_interval(SampleRate, data_f, 10, 80, file)

    ###resampling###
    Resampling = 10000
    data_res = ss.resample(data_f[start_p:end_p], Resampling)
    SampleRate_res = Resampling/(end_p-start_p)*SampleRate

    #spectrogramm###
    plt.subplot(1,2,1)
    f, t, Sxx = ss.spectrogram(data_res, fs = SampleRate_res)
    print(SampleRate_res)
    #f, t, Sxx = ss.spectrogram(data_f, fs = )

    plt.pcolormesh(t, f, Sxx, cmap=cm.gray)
    plt.xlim(t[0],t[-1])
    plt.ylim(500,3200)
    plt.yscale('log')
    plt.axis('off')
    plt.margins(0.0)
    ###trace###
    plt.subplot(1,2,2)
    plt.plot(data_res, 'k')
    #plt.plot(data_f, 'k')
    plt.axis('off')
    plt.margins(0.0)
    ##save image###
    plt.savefig(file, bbox_inches="tight", cmap='gray')
    plt.close('all')

#Funcion para identificar una imagen
def predict(image_file):
    img = image.load_img(image_file, target_size=(200, 200))
    x = image.img_to_array(img)
    x = np.expand_dims(x, axis=0)
    prediction = model.predict(x, batch_size=1).argmax()
    real_pred = pred_dict[prediction]
    return real_pred


'''
def extract_mfcc(file, fmax, nMel):
    y, sr = librosa.load(file)
    
    plt.figure(figsize=(3, 3), dpi=100)
    
    S = librosa.feature.melspectrogram(y=y, sr=sr, n_mels=nMel, fmax=fmax)
    librosa.display.specshow(librosa.core.amplitude_to_db(S), fmax=fmax)
    
    plt.xticks([])
    plt.yticks([])
    plt.tight_layout()
    plt.savefig(sys.path[0] + '/tmp/tmp/myImg.png', bbox_inches='tight', pad_inches=-0.1)
    
    plt.close()  
    return


def predict():
    # MFCCs of the test audio
    extract_mfcc(sys.path[0] + '/myNumber.wav', 8000, 256)
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
#    print (yts * 100)
    if np.max(yts) < 0.1:
        print ('Cannot Recognize!')

    # Choose the most likely class
    res = np.argmax(yts)
    print (res)
    
    return
'''


if __name__ == '__main__':
  try:
    number_recognition_node()
  except rospy.ROSInterruptException:
    pass
  
