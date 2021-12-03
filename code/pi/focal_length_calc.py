#Brent Misare
#EENG350

#Description: This python code implelemts a function to determine
#the focal length of the camera based upon marker size.

from picamera import PiCamera as Camera
from picamera.array import PiRGBArray
from time import sleep
import cv2 as cv
import cv2.aruco as aruco
import numpy as np

#H corresponds to 44mm height of the aruco marker being used
H = 4.703
#D distance in inches
D = 36 

#fileName = input("Please enter a file name for the image: ")
#camera = PiCamera(resolution = (1920, 1080))
#camera.iso = 100
#sleep(2)
#g = camera.awb_gains
#camera.awb_mode = 'off'
#camera.awb_gains = g
#sleep(5)
#camera.capture('/home/pi/SEEDLAB/%s' % fileName)

camera = Camera(resolution=(1920, 1080), framerate=60)
focalList = []
for i in range(10):
    camera.capture('/home/pi/Desktop/Demo2/test.jpg')
    img = cv.imread('test.jpg')
    grayImg = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)
    cornerList = list(corners)
    bottomRightY = int(cornerList[0][0][2][1])
    bottomLeftY = int(cornerList[0][0][3][1])
    topRightY = int(cornerList[0][0][1][1])
    topLeftY = int(cornerList[0][0][0][1])
    bottomRightX = int(cornerList[0][0][2][0])
    bottomLeftX = int(cornerList[0][0][3][0])
    topRightX = int(cornerList[0][0][1][0])
    topLeftX = int(cornerList[0][0][0][0])
    #PY is the average height in pixels of the aruco marker.
    PY = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2)
    #F is the calculated focal length of the camera.
    F = (PY * D) / (H)
    focalList.append(F)
    sleep(1)
print(cornerList)
print(focalList)
#focalAverage takes the average of the focal lengths calcuated to determine the focal length that will be used.
focalAverage = sum(focalList) / len(focalList)
print(focalAverage)
