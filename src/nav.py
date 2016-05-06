#!/usr/bin/env python

import roslib
import rospy
import cv2
import sys
import numpy
import time
import math
import random
from Tkinter import *
from std_msgs.msg import String
from sensor_msgs.msg import Image, Range
from cv_bridge import CvBridge, CvBridgeError
from naoqi_driver.naoqi_node import NaoqiNode
from naoqi import (ALBroker, ALProxy, ALModule)
from geometry_msgs.msg import Twist, Point, Quaternion
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle

class Nav(NaoqiNode):

    def setstepSize(self,event):
        self.StepSize = self.stepSizev.get()
    
    def setforwardSpeed(self,event):
        self.forwardSpeed = self.forwardSpeedv.get()

    def setrotateSpeed(self,event):
        self.rotateSpeed = self.rotateSpeedv.get()

    def setrotateForwardSpeed(self,event):
        self.rotateForwardSpeed = self.rotateForwardSpeedv.get()

    def setcannyLower(self,event):
        self.cannyLower = self.cannyLowerv.get()

    def setcannyUpper(self,event):
        self.cannyUpper = self.cannyUpperv.get()

    def setsideThreshold(self,event):
        self.sideThreshold = self.sideThresholdv.get()

    def setcenterThreshold(self,event):
        self.centerThreshold = self.centerThresholdv.get()

    def setsideDividen(self,event):
        self.sideDividen = self.sideDividenv.get()

    def setcenterDividen(self,event):
        self.centerDividen = self.centerDividenv.get()

    def setgaussianMask(self,event):   
        gaussianvalue = self.gaussianMaskv.get()
        if( gaussianvalue % 2 != 1 ):
            gaussianvalue = gaussianvalue + 1
        self.gaussianMask = gaussianvalue
        self.gaussianMaskv.set(gaussianvalue)


    def __init__(self):

    	rospy.init_node("nav")

        self.StepSize = rospy.get_param("stepSize")

        self.forwardSpeed = rospy.get_param("forwardSpeed")
        self.rotateSpeed = rospy.get_param("rotateSpeed")
        self.rotateForwardSpeed = rospy.get_param("rotateForwardSpeed")
        self.rotatingDirection = 1
        self.prevMovement = 'none'
        self.currentMovement = 'none'

        self.gaussianMask = rospy.get_param("gaussianMask")

        self.cannyLower = rospy.get_param("cannyLower")
        self.cannyUpper = rospy.get_param("cannyUpper")

        self.sideThreshold = rospy.get_param("sideThreshold")
        self.centerThreshold = rospy.get_param("centerThreshold")

        self.sideDividen = rospy.get_param("sideDividen")
        self.centerDividen = rospy.get_param("centerDividen")

        self.imageShow = rospy.get_param("imageShow")
        self.walk = rospy.get_param("walk")
        self.gui = rospy.get_param("gui")

        self.cmd_vel = Twist()
    	self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

        #self.wakeup = rospy.Publisher('/wakeup', Empty, queue_size=5)
        #self.wakeup.publish()

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/nao_robot/camera/bottom/camera/image_raw",Image,self.callback)
        
        self.sonarControl = False
        self.sonarControling = None
        self.sonarRotating = False
        #self.sonarLeft = rospy.Subscriber("/nao_robot/sonar/left/sonar",Range,self.sonarLeft)
        #self.sonarRight = rospy.Subscriber("/nao_robot/sonar/right/sonar",Range,self.sonarRight)

        #self.calibratePanel(self);

        self._cascade = cv2.CascadeClassifier('/home/gary/Desktop/cascade.xml')

        rospy.on_shutdown(self.shutdown)

        if( self.gui ):
            master = Tk()

            self.stepSizev = Scale(master, label="stepSize", from_=1, to=10, length=600,tickinterval=1 ,  orient=HORIZONTAL ,command=self.setstepSize) 
            self.stepSizev.set(self.StepSize)
            self.stepSizev.pack()

            self.forwardSpeedv = Scale(master, label="forwardSpeed", from_=0, to=1, length=600, resolution=0.1,tickinterval=0.1, orient=HORIZONTAL ,command=self.setforwardSpeed)
            self.forwardSpeedv.set(self.forwardSpeed)
            self.forwardSpeedv.pack()

            self.rotateSpeedv = Scale(master, label="rotateSpeed", from_=0, to=1, length=600, resolution=0.1,tickinterval=0.1, orient=HORIZONTAL ,command=self.setrotateSpeed)
            self.rotateSpeedv.set(self.rotateSpeed)
            self.rotateSpeedv.pack()

            self.rotateForwardSpeedv = Scale(master, label="rotateForwardSpeed", from_=0, to=1, length=600, resolution=0.1,tickinterval=0.1, orient=HORIZONTAL ,command=self.setrotateForwardSpeed)
            self.rotateForwardSpeedv.set(self.rotateForwardSpeed)
            self.rotateForwardSpeedv.pack()

            self.cannyLowerv = Scale(master, label="cannyLower", from_=1, to=100, length=600,tickinterval=9, orient=HORIZONTAL ,command=self.setcannyLower)
            self.cannyLowerv.set(self.cannyLower)
            self.cannyLowerv.pack()

            self.cannyUpperv = Scale(master, label="cannyUpper", from_=1, to=100, length=600,tickinterval=9, orient=HORIZONTAL ,command=self.setcannyUpper)
            self.cannyUpperv.set(self.cannyUpper)
            self.cannyUpperv.pack()

            self.gaussianMaskv = Scale(master, label="gaussianMask", from_=1, to=100, length=600,tickinterval=9, orient=HORIZONTAL  ,command=self.setgaussianMask)
            self.gaussianMaskv.set(self.gaussianMask)
            self.gaussianMaskv.pack()

            self.sideThresholdv = Scale(master, label="sideThreshold", from_=1, to=21, length=600 , resolution=0.5 , tickinterval=5, orient=HORIZONTAL  ,command=self.setsideThreshold)
            self.sideThresholdv.set(self.sideThreshold)
            self.sideThresholdv.pack()

            self.centerThresholdv = Scale(master, label="centerThreshold", from_=1, to=21, length=600 , resolution=0.5 ,tickinterval=5, orient=HORIZONTAL  ,command=self.setcenterThreshold)
            self.centerThresholdv.set(self.centerThreshold)
            self.centerThresholdv.pack()
    
            self.sideDividenv = Scale(master, label="sideDividen", from_=1, to=11, length=600 , resolution=1 ,tickinterval=1, orient=HORIZONTAL  ,command=self.setsideDividen)
            self.sideDividenv.set(self.sideDividen)
            self.sideDividenv.pack()

            self.centerDividenv = Scale(master, label="centerDividen", from_=1, to=11, length=600 , resolution=1 ,tickinterval=1, orient=HORIZONTAL  ,command=self.setcenterDividen)
            self.centerDividenv.set(self.centerDividen)
            self.centerDividenv.pack()

            # Code to add widgets will go here...
            master.mainloop()

    def callback(self,data):
 
        EdgeArray = []

        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
          test = cv_image
        except CvBridgeError as e:
          print(e)

        (imageheight,imagewidth,channels) = cv_image.shape
        
        imagewidth = imagewidth - 1
        imageheight = imageheight - 1

        cv_image = cv2.GaussianBlur(cv_image,(self.gaussianMask,self.gaussianMask),0)
        cv_image = cv2.Canny(cv_image, self.cannyLower, self.cannyUpper)             #edge detection

        for j in range (0,imagewidth,self.StepSize):    #for the width of image array
            for i in range(imageheight-5,0,-1):    #step through every pixel in height of array from bottom to top
                                                   #Ignore first couple of pixels as may trigger due to undistort
                if cv_image.item(i,j) == 255:       #check to see if the pixel is white which indicates an edge has been found
                    EdgeArray.append((j,i))        #if it is, add x,y coordinates to ObstacleArray
                    break                          #if white pixel is found, skip rest of pixels in column
            else:                                  #no white pixel found
                EdgeArray.append((j,0))            #if nothing found, assume no obstacle. Set pixel position way off the screen to indicate
                                                   #no obstacle detected
        if( self.imageShow ):
            for x in range (len(EdgeArray)-1):      #draw lines between points in ObstacleArray 
                cv2.line(test, EdgeArray[x], EdgeArray[x+1],(0,255,0),1) 
            for x in range (len(EdgeArray)):        #draw lines from bottom of the screen to points in ObstacleArray
                cv2.line(test, (x*self.StepSize,imageheight), EdgeArray[x],(0,255,0),1)
            
        EdgeArrayList = [x[1] for x in EdgeArray]
        calDist = EdgeArrayList.index(max(EdgeArrayList))

        if( self.walk ):
            if( EdgeArray[calDist][1] > imageheight/5):
                ave = numpy.mean(EdgeArrayList)

                arrayLen = imagewidth / self.StepSize;
                arrayLower = arrayLen / self.sideDividen
                arrayUpper = arrayLen - ( arrayLen / self.sideDividen )
                arrayCenterL = ( arrayLen / 2 ) - ( arrayLen / self.centerDividen ) / 2
                arrayCenterH = arrayCenterL + (arrayLen / self.centerDividen )
 
                if( self.imageShow ):
                    cv2.line(test, (arrayLower * self.StepSize ,0), (arrayLower * self.StepSize ,255),(0,125,0),1) 
                    cv2.line(test, (arrayUpper * self.StepSize ,0), (arrayUpper * self.StepSize ,255),(0,125,0),1) 
                    cv2.line(test, ( arrayCenterL * self.StepSize  , int(imageheight/self.centerThreshold) ), ( arrayCenterH * self.StepSize  , int(imageheight/self.centerThreshold) ),(0,125,0),1) 
                    cv2.line(test, ( imagewidth , int(imageheight/self.sideThreshold) ), ( arrayUpper * self.StepSize , int(imageheight/self.sideThreshold) ),(0,125,0),1) 
                    cv2.line(test, (0,int(imageheight/self.sideThreshold)), (arrayLower * self.StepSize,int(imageheight/self.sideThreshold)),(0,125,0),1)
                    cv2.line(test, (arrayCenterL * self.StepSize ,0), (arrayCenterL * self.StepSize ,255),(0,125,0),1) 
                    cv2.line(test, (arrayCenterH * self.StepSize ,0), (arrayCenterH * self.StepSize ,255),(0,125,0),1) 

                lowerAve = numpy.mean(EdgeArrayList[0:arrayLower])
                upperAve = numpy.mean(EdgeArrayList[arrayUpper:arrayLen])
                middleAve = numpy.mean(EdgeArrayList[arrayCenterL:arrayCenterH])

                if( middleAve > imageheight/self.centerThreshold or lowerAve > imageheight/self.sideThreshold or upperAve > imageheight/self.sideThreshold ):
                    if( lowerAve > upperAve ):
                        self.rotatingDirection = -1
                    else :            
                        self.rotatingDirection = 1

                    self.cmd_vel.linear.x = self.rotateForwardSpeed 
                    self.cmd_vel.angular.z = self.rotateSpeed * self.rotatingDirection
                    self.currentMovement = 'rotate'
                else:
                    self.cmd_vel.linear.x = self.forwardSpeed
                    self.cmd_vel.angular.z = 0
                    self.currentMovement = 'forward'
            else:
                self.cmd_vel.linear.x = self.forwardSpeed
                self.cmd_vel.angular.z = 0    
                self.currentMovement = 'forward'       
            
            if( self.currentMovement != self.prevMovement and not self.sonarControl ):
                self.prevMovement = self.currentMovement
                self.cmd_vel_pub.publish(self.cmd_vel)

        if( self.imageShow ):
            cv2.imshow("Image window", test)
            cv2.waitKey(10)

        try:
          self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))
        except CvBridgeError as e:
          print(e)

    def sonarLeft(self,data):

        #rospy.loginfo( self.sonarControl + ' ' + self.sonarControling + ' ' + self.sonarRotating )

        if(data.range < 0.4 and not self.sonarControl):
            self.sonarControl = True
            self.sonarControling = 'left'

        if( self.sonarControl and self.sonarControling=='left' and data.range < 0.4 ):
            if( not self.sonarRotating ):
                self.cmd_vel.linear.x = self.rotateForwardSpeed 
                self.cmd_vel.angular.z = self.rotateSpeed
                self.currentMovement = 'rotate'
                self.cmd_vel_pub.publish(self.cmd_vel)
                self.sonarRotating = True
        elif( self.sonarControl and self.sonarControling=='left' and data.range >= 0.4 ):
            self.cmd_vel_pub.publish(Twist())
            self.sonarControl = False
            self.sonarControling = None
            self.sonarRotating = False

    def sonarRight(self,data):
        if(data.range < 0.4 and not self.sonarControl):
            self.sonarControl = True
            self.sonarControling = 'right'

        if( self.sonarControl and self.sonarControling=='right' and data.range < 0.4 ):
            if( not self.sonarRotating ):
                self.cmd_vel.linear.x = self.rotateForwardSpeed 
                self.cmd_vel.angular.z = self.rotateSpeed * -1
                self.currentMovement = 'rotate'
                self.cmd_vel_pub.publish(self.cmd_vel)
                self.sonarRotating = True
        elif( self.sonarControl and self.sonarControling=='right' and data.range >= 0.4 ):
            self.cmd_vel_pub.publish(Twist())
            self.sonarControl = False
            self.sonarControling = None
            self.sonarRotating = False

    def shutdown(self):
        self.image_sub.unregister()
        rospy.sleep(1)

        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)      

if __name__ == '__main__':
    try:
        Nav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nav node terminated.")