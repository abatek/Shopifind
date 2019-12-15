# THE FINAL EDITION

import re
from collections import deque
import paho.mqtt.client as mqtt
import cv2
import urllib
import numpy as np
import imutils
import serial

webcam = False
bytes = ''

'''
green = ((33,80,40), (102,255,255), 'raspberry pi')
red = ((170, 100, 0), (180, 255, 255), 'wire stripper')
yellow = ((25,150,40), (35,255,255), 'tape measure')
'''

class Color:
    held = False
    held_prev = False
    down_prev = True
    # pitch roll yaw
    location = None

    def __init__(self, lower, upper, object_name):
        self.lower = lower
        self.upper = upper
        self.object_name = object_name

port = '/dev/cu.usbmodem14601'
ser = serial.Serial(port, 115200)
last_location = None


green = Color((33,80,40), (102,255,255), 'raspberry pi')
red = Color((170, 100, 40), (180, 255, 255), 'wire stripper')
yellow = Color((25,150,40), (35,255,255), 'tape measure')

pts = deque(maxlen=1024)

def checkColor(color, frame):

    # construct a mask for the color , then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, color.lower, color.upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]

    center = None
    # only proceed if at least one contour was found
    r = False
    l = False
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)



        #print(color.object_name + ': '+ str(radius))

        if radius > 50:
            # color.location = recieveHead()
            if not color.held_prev:
                color.held = True
                color.held_prev = True
                color.down_prev = False
                print('picked up ' + color.object_name + " at " + str(last_location))

            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect)  # cv2.boxPoints(rect) for OpenCV 3.x
            box = np.int0(box)
            cv2.drawContours(frame, [box], 0, color.lower, 2)
            cv2.putText(frame, color.object_name,  (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX,  1, (255,255,255), 2)

    elif not color.down_prev:
        print('put down ' + color.object_name + ', final location ' + str(last_location))
        color.down_prev = True
        color.held = False
        color.held_prev = False
        color.location = last_location

    else:
        color.held = False
        color.held_prev = False

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.


if webcam:
    cap = cv2.VideoCapture(0)
else:
    stream = urllib.urlopen('http://172.17.51.38:81/stream')

while True:
    # client.loop()

    if not webcam:
        bytes += stream.read(1024)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')
    if webcam or (a != -1 and b != -1):
        if not webcam:
            jpg = bytes[a:b + 2]
            bytes = bytes[b + 2:]
            try:
                img = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), 1)
            except:
                print('stream failed')
            # resize the frame, blur it, and convert it to the HSV
            # color space
            frame = imutils.resize(img, width=600)
        else:
            ret, frame = cap.read()

        if not webcam:
            frame = cv2.transpose(img)
            frame = cv2.flip(frame, flipCode=0)

        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        checkColor(green, frame)
        checkColor(red, frame)
        checkColor(yellow, frame)

        # show the frame to our screen

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

        # if the 'q' key is pressed, stop the loop
        if key == ord("q"):
            break

    '''
    line = ser.readline()
    spl = line.split('~')
    if len(spl) == 5:
        pitch = spl[1]
        roll = spl[2]
        yaw = spl[3]
        last_location = (pitch, roll, yaw)
    ser.flushInput()
    '''
    if green.held or red.held or yellow.held:
        ser.write('1')
    else:
        ser.write('0')




# cleanup the camera and close any open windows


