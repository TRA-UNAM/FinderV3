#!/usr/bin/env python
import rospy
import numpy as np
import pyaudio
import struct
import math

from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool

nrow = 200
ncol = 200

BLOCKSIZE = 128

RATE = 22050
WIDTH = 2
CHANNELS = 1
#Detection flag
flag = True;

def microphone_listener_node():
  global flag
  rospy.init_node('microphone_listener_node')
  pub = rospy.Publisher('/audio/mic_samples', Int16MultiArray, queue_size=800)
  rospy.Subscriber('/audio/detection_flag', Bool, flag_callback)
  #Block samples / second
  rate = rospy.Rate(int(RATE/BLOCKSIZE))
  #Start audio stream from microphone
  p = pyaudio.PyAudio()
  stream = p.open(format = p.get_format_from_width(WIDTH),
                  channels = CHANNELS,
                  rate = RATE,
                  input = True,
                  output = False)
  sample = Int16MultiArray()
  while not rospy.is_shutdown():
    if flag:
      #Samples microphone input
      input_string = stream.read(BLOCKSIZE, exception_on_overflow = False)
      input_value = struct.unpack('h' * BLOCKSIZE, input_string)
      sample.data = input_value
      pub.publish(sample)
    else:
      rate.sleep()
  #Closes input stream
  stream.stop_stream()
  stream.close()
  p.terminate()

def flag_callback(currentFlag):
  global flag
  flag = currentFlag.data

if __name__ == '__main__':
  try:
    microphone_listener_node()
  except rospy.ROSInterruptException:
    pass
