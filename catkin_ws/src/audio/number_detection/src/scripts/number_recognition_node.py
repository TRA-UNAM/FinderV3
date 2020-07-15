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
from matplotlib import pyplot as plt
import matplotlib.cm as cm
from pylab import plot, show, figure, imshow

#ROS MSGS
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from std_msgs.msg import Bool


from essentia.standard import *
import keras.backend as K
K.clear_session()


from keras import models

nrow = 450
ncol = 450
RATE = 8000 #44100
BLOCKSIZE = int( RATE/333 ) #128
WIDTH = 2
CHANNELS = 1
LEN = 1.5 * RATE   # n seconds
MAX_SILENCE_START = 40
MAX_SILENCE_END = 40
#Import Database
#model = tf.keras.models.load_model(sys.path[0] + '/sp_recog.h5')
model = models.load_model(sys.path[0] + '/mfcc_cnn_model_all.h5')

flag = True
THRESHOLD = 1
THRESHOLD_O = 1

audioBuffer = []

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
      print('Defining volume threshold..')
      prevAudioBuffer = []
      while len(prevAudioBuffer) < 20 and not rospy.is_shutdown():
        rate.sleep()
        if audioBuffer != []:
          prevAudioBuffer.append(audioBuffer.pop(0))
      THRESHOLD, THRESHOLD_O = def_threshold(prevAudioBuffer)
      print('Start talking')
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
      if rospy.is_shutdown(): break
      #Once an input is detected, threshold is adjusted down 
      THRESHOLD = THRESHOLD_O
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

      #Closes WAV file
      output_wf.close()
      #Cleans audio buffer
      audioBuffer = []

      #CNN prediction
      try:
        to_png(sys.path[0] + '/myNumber.wav', sys.path[0] + '/tmp/tmp/myNumber.png')
        message.data = predict(sys.path[0] + '/myNumber.png')
        print('Predicted as: %s' % (message.data))
        pub.publish(message)
      except:
        print('Short sample')


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
  bufferCopy = currentBuffer[:]
  samples = len(currentBuffer)
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
  #return (amp) * ( 1 + (2 * ( 1 - amp )))
  #return math.sqrt(sum_squares / (BLOCKSIZE / 2)) * 2
  return 1 - (amp - 1) * (amp - 1), amp * 1.1

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
  #Points where input is over std deviation times 0.8
  over_std = np.where( abs(data_f) > data_f.std() * 0.5 )[0]
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
  if np.max(yts) < 0.8:
    res = None
  else:
    # Choose the most likely class
    res = np.argmax(yts)
  return res


def to_png(wav_file, image_file):
  SAMPLE_SIZE = 2048
  HOP_RATE = 16; 
  HOP = SAMPLE_SIZE / HOP_RATE
  #Instantiate functions
  w = Windowing(type = 'hann')
  spectrum = Spectrum()
  mfcc = MFCC(numberBands = 120, numberCoefficients = 120,highFrequencyBound = 8192)
  bandPass = BandPass(bandwidth = 100, cutoffFrequency = 130, sampleRate = RATE)
  loader = MonoLoader(filename = wav_file)
  logNorm = UnaryOperator(type='log')

  #Load audio
  audio = loader()
  #Filter audio BP 130 Hz
  #audio = bandPass(audio)
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
    number_recognition_node()
  except rospy.ROSInterruptException:
    pass
  
