#!/usr/bin/env python
import rospy
import numpy as np
import pyaudio
import struct
import wave
import math
import sys
import os

#ROS MSGS
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
from std_msgs.msg import String
from std_msgs.msg import Bool

RATE = 8000 #44100
BLOCKSIZE = int( RATE/333 ) #128

#RATE = 44100 #22050
#BLOCKSIZE = 128#int(RATE/100) #128

WIDTH = 2
CHANNELS = 1

flag = False
bufferflag = False
audioBuffer = []
pyAudio = None
audioStream = None

def audio_stream_node():
  global flag
  global bufferflag
  global audioBuffer
  global audioStream
  global pyAudio

  rospy.init_node('audio_stream_node')
  rospy.Subscriber('/audio/mic_samples', Int16MultiArray, audio_callback,queue_size=800)
  rospy.Subscriber('/audio/stream_flag', Bool, flag_callback)
  pyAudio = pyaudio.PyAudio()
  audioStream = pyAudio.open(format = pyAudio.get_format_from_width(WIDTH),
                  channels = CHANNELS,
                  rate = RATE,
                  input = False,
                  output = True)
  rate = rospy.Rate(int(2*RATE))
  #rate = rospy.Rate(int(RATE))
  while not rospy.is_shutdown():
    if flag:
      #Checks for buffer size to start stream
      if (not bufferflag):
        print("Buffering")
        #print(len(audioBuffer))
        rate.sleep()
        if len(audioBuffer) > 60:
          bufferflag = True
          print("Done")
      elif len(audioBuffer) > 0:
        audioStream.write(audioBuffer.pop(0))
        print(len(audioBuffer))
        rate.sleep()
    else:
      rate.sleep()
  #Closes output stream
  audioStream.stop_stream()
  audioStream.close()
  pyAudio.terminate()



def flag_callback(currentFlag):
  global flag
  global bufferflag
  global audioBuffer
  flag = currentFlag.data
  if flag:
    bufferflag = False
    audioBuffer = []


def audio_callback(currentSample):
  global audioBuffer
  global flag
  if flag:
    #Append sample to buffer
    audioBuffer.append(struct.pack('h' * BLOCKSIZE, *currentSample.data))
  return

if __name__ == '__main__':
  try:
    audio_stream_node()
  except rospy.ROSInterruptException:
    pass
  
