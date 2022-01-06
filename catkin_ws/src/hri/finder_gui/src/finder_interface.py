#!/usr/bin/env python 
#-*- coding: utf-8 -*-
#Transforms .ui file to .py
import struct
import sys
from wave import Error
import numpy as np
import os
import copy
import inspect
import atexit

from numpy.core.records import array
os.system("pyuic5 -x ~/FinderV3/catkin_ws/src/hri/finder_gui/src/Interface_ui.ui -o ~/FinderV3/catkin_ws/src/hri/finder_gui/src/Interface_ui.py")
#Qt5 dependencies
from Interface_ui import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
#ROS dependencies
import cv2
import rosgraph
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import rviz


masterUserName = 'roberto'
masterPassword = '' #You need to copy an atuhorized ssh key to master
logoLocation = 'resources/logo-tra.png'
        
class MainWindowDesign(QtWidgets.QMainWindow, Ui_MainWindowDesign):
  def __init__(self, *args, **kwargs):
    ###Display UI
    QtWidgets.QMainWindow.__init__(self, *args, **kwargs)
    self.setupUi(self)
    #self.setWindowIcon(QIcon(logoLocation))
    #self.setWindowTitle('Finder GUI')
    ###Initial Functions
    #Start on initial tab
    self.tabWidget.setCurrentIndex(0)
    ###Master connection objects
    self.coreProcess = QtCore.QProcess()
    self.environmentProcess = QtCore.QProcess()
    #Get environment variables
    rosMaster, rosHost, rosIP = self.GetEnvironmentVariables()
    #SSH Objects
    endMaster = rosMaster.index('1')
    endMaster = rosMaster.index(':11311')
    masterIP = rosMaster[7:(endMaster)]
    masterAddress = "ssh " + masterUserName + "@" + masterIP
    self.line_edit_ssh_dest.setText(masterAddress)
    self.line_edit_ssh_psswd.setText(masterPassword)
    ###UI Connection
    #Master
    self.button_connect.released.connect(self.MasterConnect)
    self.button_disconnect.released.connect(self.MasterDisconnect)
    self.checkBoxCore.stateChanged.connect(self.MasterChangedOptions)
    #self.coreProcess.readyReadStandardOutput.connect(self.CorePrint)  #Manually enabled on connection
    self.coreProcess.readyReadStandardError.connect(self.CoreError)
    #self.coreProcess.readyReadStandardError.connect(self.MasterDisconnect)
    #SSH Connection
    self.checkBoxSSH.stateChanged.connect(self.line_edit_ssh_dest.setEnabled)
    self.checkBoxSSH.stateChanged.connect(self.line_edit_ssh_psswd.setEnabled)
    #Text
    self.core_text_error.setTextColor(QtGui.QColor("#FF0000"))
    self.button_master_clear.released.connect(self.core_text.clear)
    self.button_master_clear.released.connect(self.core_text_error.clear)
    
    ###Launch objects
    self.totalLaunch = 10
    self.launchProcess = range(self.totalLaunch)
    self.launchCommand = [
      "roslaunch finder_bringup finder_cameras.launch",
      "roslaunch finder_bringup finder_cameras_base_station.launch",
      "roslaunch finder_fake finder_fake.launch",
      "roslaunch finder_bringup finder_microphone.launch",
      "roslaunch finder_bringup finder_microphone_base.launch",
      "","","","",""]
    #0 = Local; 1 = master 
    self.launchLocationIndex = [1,0,1,1,0,0,0,0,0,0]
    self.launchLocation = range(self.totalLaunch)
    self.launchModifyCheck = range(self.totalLaunch)
    self.launchCommandLine = range(self.totalLaunch)
    self.launchButton = range(self.totalLaunch)
    self.launchButtonStop = range(self.totalLaunch)
    self.launchOutput = range(self.totalLaunch)
    self.launchOutputError = range(self.totalLaunch)
    #Organize launch UI elements on arrays
    for i in range(self.totalLaunch):
      self.launchProcess[i] = QtCore.QProcess()
      exec("self.launch_command_%s.setText('%s')" % (str(i),self.launchCommand[i]))
      exec("self.launch_location_%s.setCurrentIndex(%d)" % (str(i),self.launchLocationIndex[i]))
      exec("self.launchCommandLine[i] = self.launch_command_%s" % str(i))
      exec("self.launchModifyCheck[i] = self.launch_modify_%s" % str(i))
      exec("self.launchButton[i] = self.launch_start_%s" % str(i))
      exec("self.launchButtonStop[i] = self.launch_stop_%s" % str(i))
      exec("self.launchLocation[i] = self.launch_location_%s" % str(i))
      exec("self.launchOutput[i] = self.launch_output_%s" % str(i))
      exec("self.launchOutputError[i] = self.launch_output_error_%s" % str(i))
    #UI connection
    for i in range(self.totalLaunch):
      self.launchModifyCheck[i].stateChanged.connect(self.LaunchEnable)
      self.launchButton[i].released.connect(self.LaunchStart)
      self.launchButtonStop[i].released.connect(self.LaunchStop)
      self.launchOutputError[i].setTextColor(QtGui.QColor('#FF0000'))
    self.launch_run_all.released.connect(self.LaunchStartAll)
    self.launch_stop_all.released.connect(self.LaunchStopAll)
    self.pushButton_RViz.released.connect(self.RVizRun)


    ###ROS objects
    ###Video objects
    self.totalVideoStream = 7
    self.totalVideoWindows = 7
    self.totalVideoChecks = 3
    #UI elements
    self.videoStream = range(self.totalVideoStream)
    self.videoMenu = range(self.totalVideoStream)
    self.videoReload = range(self.totalVideoStream)
    self.videoStretch = range(self.totalVideoStream)
    self.videoStretchState = range(self.totalVideoStream)
    self.videoDetectChecks = range(self.totalVideoChecks)
    self.videoDetectPub = range(self.totalVideoChecks)
    #Video Subscribers
    self.videoSubscribers = range(self.totalVideoStream)
    #Initialconfigs
    self.videoCurrentChannel = [0,1,2,3,2,5,6]
    self.videoChannelsDescription = ["Video 0","Video 1","Video 2:","Video 3:","Video 4:",
      "Detected Labels","Motion detection"]
    self.videoChannels = ["/cam0/image_raw_local","/cam1/image_raw_local","/cam2/image_raw_local",
    "/cam3/image_raw_local","/cam4/image_raw_local","/vision/detected_labels_image_raw","/vision/movement_detected_image_raw"]
    self.videoDetections = ["/vision/label_flag","/vision/movement_flag","/vision/qr_flag","",
                            "","",""]
    #Organize UI elements on arrays
    for i in range(self.totalVideoStream):
      exec("self.videoStream[i] = self.video_stream_%s" % str(i))
      exec("self.videoMenu[i] = self.video_menu_%s" % str(i))
      exec("self.videoReload[i] = self.video_reload_%s" % str(i))
      exec("self.videoStretch[i] = self.video_stretch_%s" % str(i))
      self.videoMenu[i].addItems(self.videoChannelsDescription)
      self.videoMenu[i].setCurrentIndex(self.videoCurrentChannel[i])
      self.videoStretchState[i] = self.videoStretch[i].checkState() != 0
    self.currentTime = np.zeros(self.totalVideoStream)
    #UI connection
    self.video_start.released.connect(self.RosVideoStart)
    self.video_stop.released.connect(self.RosVideoStop)
    for i in range(self.totalVideoStream):
      self.videoStretch[i].stateChanged.connect(self.VideoStretch)
    self.bridge = CvBridge()
    self.frameRate = 30
    
    ##Subscribers
    self.totalSubscribers = 2
    self.topicSubscribersText = ["/vision/qr_barcode_results", "/audio/detected_numbers"]
    self.topicSubscribers = range(self.totalSubscribers)
    self.topicSubscribersTextBox = range(self.totalSubscribers)
    for i in range(self.totalSubscribers):
      exec("self.topicSubscribersTextBox[i] = self.subscriber_text_%s" % (str(i)))
    
    ###Audio Stream
    self.totalAudioChecks = 2
    self.audioChecks = range(self.totalAudioChecks)
    self.audioChecksState = range(self.totalAudioChecks)
    self.audioPublishers = range(self.totalAudioChecks)
    self.audioCheckTopics = [ "/audio/detection_flag", "/audio/stream_flag"]
    #Audio threshold
    
    self.audioThresholdKnob = self.thresholdSlider
    self.audioThresholdInd = self.thresholdInd
    #self.audioThresholdKnob.sliderMoved.connect(self.AudioThresholdSet)
    self.RATE = 8000 #22050
    self.BLOCKSIZE = int(self.RATE/333) #128
    self.WIDTH = 2
    self.CHANNELS = 1
    self.audioRate = 30#self.RATE/self.BLOCKSIZE
    self.audioThresFlag = False
    '''
    self.pyAudio = pyaudio.PyAudio()
    self.audioStream = self.pyAudio.open(format = self.pyAudio.get_format_from_width(self.WIDTH),
                  channels = self.CHANNELS,
                  rate = self.RATE,
                  input = False,
                  output = True)
                  
    self.audioSubscriber = rospy.Subscriber("/audio/mic_samples",Int16MultiArray, self.AudioCallback,queue_size=800)
    self.audioBuffer = []
    self.audioStreamFlag = False
    '''
    for i in range(self.totalAudioChecks):
      exec("self.audioChecks[i] = self.subscriber_box_audio_%s" % str(i))
      self.audioChecks[i].stateChanged.connect(self.AudioChecks)
      self.audioChecksState[i] = False#self.audioChecks[i].checkState > 0
    return
    atexit.register(self.Cleanup)

  #######################
  # ROS Core functions
  # #####################  
  def GetEnvironmentVariables(self):
    self.environmentProcess.start("printenv ROS_MASTER_URI")
    self.environmentProcess.waitForFinished()
    str1 = str(self.environmentProcess.readAllStandardOutput())
    self.line_edit_master.setText(str1)
    
    self.environmentProcess.start("printenv ROS_HOSTNAME")
    self.environmentProcess.waitForFinished()
    str2 = str(self.environmentProcess.readAllStandardOutput())
    self.line_edit_host.setText(str2)
    
    self.environmentProcess.start("printenv ROS_IP")
    self.environmentProcess.waitForFinished()
    str3 = str(self.environmentProcess.readAllStandardOutput())
    self.line_edit_ip.setText(str3)
    
    return str1, str2, str3
    
  def MasterConnect(self):
    #Set environment variables
    os.environ['ROS_MASTER_URI'] = str(self.line_edit_master.text())
    os.environ['ROS_HOSTNAME'] = str(self.line_edit_host.text())
    os.environ['ROS_IP'] = str(self.line_edit_ip.text())
    #Disable UI elements
    self.button_connect.setEnabled(False)
    self.button_disconnect.setEnabled(True)
    self.MasterEnableText(False)
    #SSH connection to master 
    #Flush process buffer
    self.coreProcess.readAllStandardOutput()
    self.coreProcess.start(self.line_edit_ssh_dest.text() + ' -tt')
    self.coreProcess.waitForStarted(500)
    self.coreProcess.waitForReadyRead(1500)
    if not self.coreProcess.canReadLine():
      os.system("echo 'Cant connect to Master IP'")
      self.SendErrorMessage("Can't connect to Master IP")
      self.MasterDisconnect()
    else:
      self.coreProcess.readyReadStandardOutput.connect(self.CorePrint)
      os.system('echo Starting ROS')
      #Run ROSCORE
      self.coreProcess.write('roscore\n')
  
  def MasterDisconnect(self):  
    if self.coreProcess.receivers(self.coreProcess.readyReadStandardOutput):
      self.coreProcess.readyReadStandardOutput.disconnect()
    self.coreProcess.terminate()
    self.coreProcess.waitForFinished()
    self.coreProcess.write("exit\n")
    self.button_disconnect.setEnabled(False)
    self.button_connect.setEnabled(True)
    self.MasterEnableText(True)
  def MasterChangedOptions(self):
    if self.checkBoxCore.checkState() == 0:
      self.line_edit_master.setEnabled(False)
      self.line_edit_host.setEnabled(False)
      self.line_edit_ip.setEnabled(False)
    else:
      self.line_edit_master.setEnabled(True)
      self.line_edit_host.setEnabled(True)
      self.line_edit_ip.setEnabled(True)
  def MasterEnableText(self, state):
    if state:
      self.checkBoxCore.setEnabled(True)
      if self.checkBoxCore.checkState():
        self.line_edit_master.setEnabled(True)
        self.line_edit_host.setEnabled(True)
        self.line_edit_ip.setEnabled(True)
    else:
      self.line_edit_master.setEnabled(False)
      self.line_edit_host.setEnabled(False)
      self.line_edit_ip.setEnabled(False)
      self.checkBoxCore.setEnabled(False)
      
  def CorePrint(self):
    errorMessages = ["Exception: roscore cannot run as another roscore/master is already running"]
    stdMessages = ["started core service"]
    Qstr = self.coreProcess.readAllStandardOutput()
    strr = str(Qstr)
    for msg in errorMessages:
      if strr.find(msg) >= 0:
        self.core_text_error.insertPlainText(strr)
        self.MasterDisconnect()
        self.SendErrorMessage(msg)
    for msg in stdMessages:
      if strr.find(msg) >= 0:
        self.SendErrorMessage(msg)
    self.core_text.insertPlainText(strr)
  def CoreError(self):
    Qstr = self.coreProcess.readAllStandardError()
    strr = str(Qstr)
    errorBox = QtWidgets.QMessageBox ()
    errorBox.setText(strr)
    errorBox.exec_()
    self.core_text_error.insertPlainText(strr)
  def SendErrorMessage(self,ERROR):
    errorBox = QtWidgets.QMessageBox()
    errorBox.setText(ERROR)
    errorBox.exec_()
  ######################################
  # Launch Functions
  ######################################    
  def LaunchEnable(self, state):
    currentObject = self.sender()
    objName = currentObject.objectName()
    indx = int(objName[(objName.find('_',7)+1):])
    #os.system('echo ' + str(objName))
    if currentObject.checkState() == 0:
      self.launchCommandLine[indx].setEnabled(False)
    else:
      self.launchCommandLine[indx].setEnabled(True)

  def LaunchStart(self, cproc  = None):
    ##Get sender index
    if cproc:
      currentObject = cproc
    else:
      currentObject = self.sender()
    objName = currentObject.objectName()
    indx = int(objName[(objName.find('_',7)+1):])
    ##Disable UI Elements
    self.launchButton[indx].setEnabled(False)
    self.launchModifyCheck[indx].setEnabled(False)
    self.launchLocation[indx].setEnabled(False)
    self.launchCommandLine[indx].setEnabled(False)
    ###Launch on Master
    if self.launchLocation[indx].currentIndex() == 1:
      #SSH connection to master
      #Flush process buffer
      self.launchProcess[indx].readAllStandardOutput()
      os.system("echo 'SSH connecting'")
      self.launchProcess[indx].start(self.line_edit_ssh_dest.text() + ' -tt')
      self.launchProcess[indx].waitForStarted(500)
      self.launchProcess[indx].waitForReadyRead(1500)
      if not self.launchProcess[indx].canReadLine():
        os.system("echo 'Cant connect to Master IP'")
        self.SendErrorMessage("Can't connect to Master IP")
        self.LaunchStop()
      else:
        os.system("echo 'Launching...'")
        self.launchProcess[indx].readyReadStandardOutput.connect(lambda: self.LaunchPrint(indx))
        #Run Launch command
        self.launchProcess[indx].write(str(self.launchCommandLine[indx].text()) + "\n")
        os.system("echo 'Launched'")
    ###Launch on local
    elif self.launchLocation[indx].currentIndex() == 0:
      os.system("echo 'Launching local...'")
      self.launchProcess[indx].readyReadStandardOutput.connect(lambda: self.LaunchPrint(indx))
      #Run Launch command
      #self.launchProcess[indx].start(self.launchCommand[indx]) #self.launchCommandLine[indx].text()))
      #self.launchProcess[indx].start("echo hi")
      #self.launchProcess[indx].waitForStarted()
      self.launchProcess[indx].start(self.launchCommand[indx])#(str(self.launchCommandLine[indx].text()))#(self.launchCommand[indx]) #self.launchCommandLine[indx].text()))
      self.launchProcess[indx].waitForStarted()
      os.system("echo 'Launched'")

  def LaunchStop(self, cproc  = None):
    ##Get sender index
    if cproc:
      currentObject = cproc
    else:
      currentObject = self.sender()
    objName = currentObject.objectName()
    indx = int(objName[(objName.find('_',7)+1):])
    ##Disable UI Elements
    self.launchButton[indx].setEnabled(True)
    self.launchModifyCheck[indx].setEnabled(True)
    self.launchLocation[indx].setEnabled(True)
    if self.launchModifyCheck[indx].checkState():
      self.launchCommandLine[indx].setEnabled(True)
    while self.launchProcess[indx].state() == 2:
      self.launchProcess[indx].waitForFinished(1000)
      self.launchProcess[indx].terminate()
    #If process was launched on master, stop ssh connection
    if self.launchLocationIndex[indx] == 1:
      self.launchProcess[indx].waitForFinished(5000)
      self.launchProcess[indx].write("exit\n")
    if self.launchProcess[indx].receivers(self.launchProcess[indx].readyReadStandardOutput):
      self.launchProcess[indx].readyReadStandardOutput.disconnect()
    
  def LaunchPrint(self, indx):
    errorMessages = ["Exception", "Error", "ERROR"]
    stdMessages = ["started core service"]
    Qstr = self.launchProcess[indx].readAllStandardOutput()    
    strr = str(Qstr)
    for msg in errorMessages:
      if strr.find(msg) >= 0:
        self.launchOutputError[indx].insertPlainText(strr)
        #self.MasterDisconnect()
        #self.SendErrorMessage(strr)
    for msg in stdMessages:
      if strr.find(msg) >= 0:
        self.SendErrorMessage(msg)
    self.launchOutput[indx].insertPlainText(strr)

  ####Start all launch files
  def LaunchStartAll(self):
    for cproc in self.launchButton:
      self.LaunchStart(cproc)
  def LaunchStopAll(self):
    for cproc in self.launchButton:
      self.LaunchStop(cproc)

  ######################################
  # Video Functions
  ###################################### 
  def RVizRun(self):
    self.rviz_widget.deleteLater()
    he = self.rviz_widget.height()
    wi = self.rviz_widget.width()
    self.rviz_widget = RVizWidget()
    self.rviz_layout.addWidget(self.rviz_widget)
    self.rviz_widget.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.MinimumExpanding)
    self.rviz_widget.setMinimumSize(160,120)
    self.rviz_widget.setMaximumSize(1600,1200)
    self.rviz_widget.resize(int(he),int(wi))

  ######################################
  # ROS node
  ######################################   
  def RosVideoStart(self):
    if rosgraph.is_master_online(): # Checks the master uri and results boolean (True or False)
      pass
    else:
      self.SendErrorMessage("No master node running")
      return
    #Subscribers
    rospy.init_node('ros_gui_node')
    for i in range(self.totalVideoChecks):
      exec("self.videoDetectChecks[i] = self.subscriber_box_%s" % str(i))
      self.videoDetectPub[i] = rospy.Publisher(self.videoDetections[i],Bool,queue_size=1)
      self.videoDetectChecks[i].stateChanged.connect(self.DetectEnable)
    for i in range(self.totalSubscribers):
      self.topicSubscribers[i] = rospy.Subscriber(self.topicSubscribersText[i],String, self.SubscriberCallback,queue_size=5)
    self.audioThresholdSub = rospy.Subscriber("/audio/threshold_up",Bool,self.AudioThresholdCallback, queue_size=20)
    self.audioThresholdPub = rospy.Publisher("/audio/threshold_val",Int16, queue_size=1)
    for i in range(self.totalAudioChecks):
      self.audioPublishers[i] = rospy.Publisher(self.audioCheckTopics[i],Bool,queue_size=1)        
    for indx in range(self.totalVideoStream):
      print(indx)
      #self.videoSubscribers[indx] = rospy.Subscriber((self.videoChannels[self.videoMenu[indx].currentIndex()]), Image, self.videoCallback, queue_size = 1)
      self.videoSubscribers[indx] = rospy.Subscriber((self.videoChannels[indx]), Image, self.videoCallback, queue_size = 1)


    return



  def RosVideoStop(self):
    for sub in self.videoSubscribers:
      if sub:
        sub.unregister()
    return
    
  def videoCallback(self,img):
    #Gets topic from msg
    topic = False
    for t in range(self.totalVideoStream):
      if img._connection_header["topic"] == self.videoChannels[t]:
        topic = t
        #Checks for framerate
        newTime = rospy.get_time()
        if newTime - self.currentTime[topic] < (float(1) / self.frameRate):
          return
        self.currentTime[topic] = rospy.get_time()
        break
    subs = False
    #Checks if at least one stream is subscribed
    for s in range (self.totalVideoStream):
      if topic == self.videoMenu[s].currentIndex():
        subs = True
        break
    if not subs:
      return
    #Process image
    image = self.bridge.imgmsg_to_cv2(img, "rgb8")
    height, width, _ = image.shape
    bytesPerLine = 3 * width
    qImg = QImage(image.data, width, height, bytesPerLine, QImage.Format_RGB888)
    scale = 1
    #Displays image on all subscribed video streams
    for indx in range(self.totalVideoStream):
      if topic == self.videoMenu[indx].currentIndex():
        #Scales image to stream dimentions
        windowHeight = self.videoStream[indx].height()
        windowWidth = self.videoStream[indx].width()
        if self.videoStretchState[indx]:
          if (windowHeight > windowWidth):
            self.videoStream[indx].setPixmap(QPixmap(qImg.scaledToHeight(windowHeight*scale)))
          else:
            self.videoStream[indx].setPixmap(QPixmap(qImg.scaledToWidth(windowWidth*scale)))
        else:
          if (float(windowHeight)/height < float(windowWidth)/width):
            self.videoStream[indx].setPixmap(QPixmap(qImg.scaledToHeight(windowHeight*scale)))
          else:
            self.videoStream[indx].setPixmap(QPixmap(qImg.scaledToWidth(windowWidth*scale)))
    return
    
  def VideoStretch(self):
    currentObject = self.sender()
    objName = currentObject.objectName()
    indx = int(objName[(objName.find('_',7)+1):])
    self.videoStretchState[indx] = currentObject.checkState() != 0


  #####Audio Methods
  def AudioChecks(self):
    currentObject = self.sender()
    objName = currentObject.objectName()
    indx = int(objName[(objName.find('_',16)+1):])
    self.audioChecksState[indx] = currentObject.checkState() != 0
    if self.audioChecksState[indx]:
      self.audioPublishers[indx].publish(Bool(True))
    else:
      self.audioPublishers[indx].publish(Bool(False))
    return

  def AudioThresholdCallback(self, val):
    #Checks for framerate
    if self.audioThresFlag == val.data:
      return    
    self.audioThresFlag = val.data
    if val.data:
      self.audioThresholdInd.setStyleSheet("background:green")
    else:
      self.audioThresholdInd.setStyleSheet("background:red")
    return

  def AudioThresholdSet(self,val):
    self.audioThresholdPub.publish(Int16(val))
    return

  def AudioCallback(self,audiomsg):
    return
    if self.audioChecksState[1]:
      #Append sample to buffer
      aud = struct.pack('h' * self.BLOCKSIZE, *audiomsg.data)
      self.audioBuffer.append(aud)
      #Checks for buffer size to start stream
      if (not self.audioStreamFlag):
        if len(self.audioBuffer) > 5:
          self.audioStreamFlag = True
      else:
       self.audioStream.write(self.audioBuffer.pop()) 
      #self.audioStream.write(aud)
    return
  ######################################
  # Publishers
  ###################################### 
  def DetectEnable(self):
    currentObject = self.sender()
    objName = currentObject.objectName()
    indx = int(objName[(objName.find('_',11)+1):])
    if currentObject.checkState() != 0:
      self.videoDetectPub[indx].publish(Bool(True))
    else:
      self.videoDetectPub[indx].publish(Bool(False))
    return
  ######################################
  # Subscribers
  ###################################### 
  def SubscriberCallback(self,strmsg):
    #Gets topic from msg
    for indx in range(self.totalSubscribers):
      if strmsg._connection_header["topic"] == self.topicSubscribersText[indx]:
        self.topicSubscribersTextBox[indx].insertPlainText(strmsg.data + "\n")
        self.topicSubscribersTextBox[indx].verticalScrollBar().setValue(self.topicSubscribersTextBox[indx].verticalScrollBar().maximum())
        #if strmsg.data != "NULL":
        #  self.topicSubscribersTextBox[indx].insertPlainText(strmsg.data + "\n")
        pass
    return
    
    

  def __del__(self):
    #Terminates process
    print("Killing process")
    self.coreProcess.kill()
    self.coreProcess.write("exit\n")
    self.environmentProcess.kill()
    for indx in range(self.totalLaunch):
      print("Killing Launch %" % str(indx))
      self.launchProcess[indx].kill()
      if self.launchLocationIndex[indx] == 0:
        self.launchProcess[indx].waitForFinished()
        self.launchProcess[indx].write("exit\n")
      self.launchProcess[indx].kill()
    #os.system("killall -9 rosmaster")
    os.system("echo ':V'")

#####RViz
class RVizWidget( QWidget ):
    def __init__(self):
        QWidget.__init__(self)
        self.frame = rviz.VisualizationFrame()
        self.frame.setSplashPath(logoLocation)
        self.frame.initialize()
        
        reader = rviz.YamlConfigReader()
        config = rviz.Config()
        configPath = "~/FinderV3/catkin_ws/src/simulation/finder_fake/rviz/finder_fake.rviz"
        reader.readFile( config,  configPath)
        self.frame.load( config )
        #self.setWindowTitle( config.mapGetChild( "Title" ).getValue() )
        #self.frame.setMenuBar( None )
        
        self.frame.setStatusBar( None )
        self.frame.setHideButtonVisibility( False )
        self.manager = self.frame.getManager()
        self.grid_display = self.manager.getRootDisplayGroup().getDisplayAt( 0 )
        layout = QVBoxLayout()
        layout.addWidget( self.frame )
        self.setLayout( layout )
        

if __name__ == "__main__":
  app = QtWidgets.QApplication([])
  window = MainWindowDesign()
  window.show()

  sys.exit(app.exec_())
