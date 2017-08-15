#!/usr/bin/env python

import cv2 #OpenCV
import time
import winsound #to play sound effects
import threading
import math

#import pyfly2
#import MotorControl as mc
s = 0
audioplay = True

#OpenCV int webcam
port = int(raw_input("Camera Port: "))
cam = cv2.VideoCapture(port)
time.sleep(0.5)
cam.release()
cam.open(port)
time.sleep(0.5)


detect = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

def play():
	global s
	while True:
		if (s == 1):
			winsound.PlaySound('detect.wav',winsound.SND_FILENAME)
			print s
		if (audioplay == False):
			break

def distanceFormula(x1, y1, x2, y2):
	return math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)

def getDistanceFromCenter(box):
	return distanceFormula(320, 240, box[0] + box[2], box[1] + box[3])

def getClosest(boxes):
	if len(boxes) == 0:
		return (0,0,0,0)
	elif len(boxes) == 1:
		return boxes[0]
	else:
		closest = boxes[0]
		distance = getDistanceFromCenter(boxes[0])
		for box in boxes:
			d = getDistanceFromCenter(box)
			if d < distance:
				distance = d
				closest = box
		return closest

#Load frame, apply front face haar classifier, and check to see if identified rectangle is in center of frame/center of gun
def processframe():
	global s
	#grab frame
	retval, frame = cam.read()
		
	#Apply the frontal classifier:
	frontal = detect.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)
	if (False):
		print("frontal contains nothing")
	else:
		 (x, y, w, h) = getClosest(frontal)
	#where (x,y) is upper vertex of rect. w/h is width/height
	#center of rectangle, draw a circle
	center = ( (x+(w/2)), (y+(h/2)))
	if x !=0:
		cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,153), 5)  
		cv2.circle(frame, (center), 3, (0, 200, 0), -1)
		
	#Grab center of rectangle (only x is needed) and test to see if it falls within
	center = ( (x+(w/2)))
	#print(center)
	#Send pysocket commands to motorcontroller/pi regarding situation
	if (center < 390 and center > 250 ):
		cv2.putText(frame, "Target Found", (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.5,(27, 204, 4), 2)
		s = 1
	elif (center > 390):
		s = 0
	elif (center < 250):
		s = 0
	cv2.imshow("hurrderr", frame)

time.sleep(1)

sound_thread = threading.Thread(target=lambda:play())
sound_thread.start()
while True:
	processframe()
	if cv2.waitKey(30) == 27:
		audioplay = False
		break
cam.release()
cv2.destroyAllWindows()
