#Brent Misare
#EENG350

#Description: This python code implelemts a function to detect the angle
#and distance of an aruco marker from the robot's camera.


from picamera import PiCamera
from picamera.array import PiRGBArray
from time import sleep
import cv2 as cv
import cv2.aruco as aruco
import numpy as np
import math as m
import smbus
import board
import busio
#import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
addr = 0x04

corners = 0

cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
ret, frame = cap.read()

#idDes = 1 # Aruco ID to be searched for

#Constants
#FOC   = 1137.104 #6X6 720p
FOC    = 1707.378 #6x6 1080p
HEIGHT = 4.703   # Height/width of aruco marker being used in inches (6X6)

def ReadAruco():
    # Captures video and picture
    ret, frame = cap.read()
    grayImg = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    
    # Dectect and measure marker
    arucoDictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
    arucoParameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(grayImg, arucoDictionary,parameters=arucoParameters)     
    try:
        idInd = int(np.where(ids == idDes)[0])
        #print(idInd)
    except:
        print("Id not found")
        idInd = 255
    if (idInd is not 255):
        cornerList = list(corners)
        bottomRightY =  int(cornerList[idInd][0][2][1])
        bottomLeftY =   int(cornerList[idInd][0][3][1])
        topRightY =     int(cornerList[idInd][0][1][1])
        topLeftY =      int(cornerList[idInd][0][0][1])
        bottomRightX =  int(cornerList[idInd][0][2][0])
        bottomLeftX =   int(cornerList[idInd][0][3][0])
        topRightX =     int(cornerList[idInd][0][1][0])
        topLeftX =      int(cornerList[idInd][0][0][0])
        # Calculate dist/angle values
        aveH = ((abs(topRightY - bottomRightY)) + (abs(topLeftY - bottomLeftY))) / (2) #Average height in pixels
        centerX = (((topLeftX + bottomRightX) / (2)) + ((topRightX + bottomLeftX) / (2))) / (2)       
        centerX = centerX - 41
        dist = (HEIGHT * FOC)/(aveH)
        distX = ((HEIGHT / aveH) * (960 - centerX))
        phi = m.atan((distX / dist)) * (180 / m.pi)
        # Adjustments
        
        #phi = phi / 1.015461178
        #distFromCenter = distFromCenter / 1.06042          
    else:
        phi  = 0
        dist = 0
        centerX = 0
    # Displays image for troubleshooting
    #miniFrame = cv.resize(frame, (480, 270))
    #miniFrame = cv.cvtColor(miniFrame, cv.COLOR_BGR2GRAY)
    #cv.circle(miniFrame, (int(centerX / 4), 135), 3, (0, 0, 255), -1)
    #cv.imshow('window', miniFrame)
    #cv.waitKey(1)
    return phi, ids, dist

# Sets IDs to be found
def swapIDs(idDes):
    if (idDes == 1):
        return 2
    else:
        return 1

# Main loop    
idFound = False
idDes = 1
missCounter = 0
while(1):
    ret, frame = cap.read()
    phi, ids, dist = ReadAruco()
    print("ID: " + str(idDes) + " - " + str(round(dist, 2))+"in - "+str(round(phi, 2))+"deg - Miss: " + str(missCounter))
    try:
        bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*dist))])
    except:
        print("Com Error")
        sleep(0.1)
        try:
            bus.write_i2c_block_data(addr, 0, [int(2*(phi+32)), int(round(2*dist))])
        except:
            print("Seq Com Error!")
    sleep(0.01)
    # if the desired ID has been found, and then afterwards it misses the id
    # for 6 or more loops, it switches the desired ID.
    if (ids is None):
        ids = []
    elif (idDes in ids):
        idFound = True
        
    if (idFound) and (idDes not in ids):
        missCounter += 1
    else:
        missCounter = 0
    if (idFound) and (idDes not in ids) and (missCounter > 3):
        idFound = False
        idDes = swapIDs(idDes)
        missCounter = 0
cap.release()
