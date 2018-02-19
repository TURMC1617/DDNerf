#!/usr/bin/env python

#------------------------------------------------------------------------------
#
# dependency info
#
#------------------------------------------------------------------------------
#
# This package depends on freenect, the open source Kinect SDK alternative.
#
# To get this on linux do the following:
#  1. apt-get install freenect
#  2. for python wrapper, follow instructions here:
#     https://github.com/OpenKinect/libfreenect/tree/master/wrappers/python
#  
#------------------------------------------------------------------------------
#
# imports
#
#------------------------------------------------------------------------------

# computer vision
# 
import cv2 #OpenCV

# kinect
# 
import freenect as fn



# general 
# 
import time
import numpy as np
import sys
import sd12
import sd12.ext
import serial
# import winsound #to play sound effects
import threading

#------------------------------------------------------------------------------
#
# globals
#
#------------------------------------------------------------------------------

# opencv (may change this program to be oop later so this doesn't have to be
# global)
# 
detect = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# kinect stuff
#
KINECT_PORT = 0

# audio
# 
PLAY_AUDIO = True

#------------------------------------------------------------------------------
#
# functions
#
#------------------------------------------------------------------------------


# function: play
#
# arguments:
#  fname: name of wav file to play
#
# return: none
#
# play audio for given wav file
# 
def play():
	global s
	while True:
		if (PLAY_AUDIO == False):
			break

		if (s == 1):
			winsound.PlaySound('fire.wav',winsound.SND_FILENAME)
			print s
	return
#
# end of funciton

# function: distanceFormula
#
# arguments:
#  x1: x coordinate of point 1
#  y1: y coordinate of point 1
#  x2: x coordinate of point 2
#  y2: y coordinate of point 2
#
# return: distance between the two points
#
# get the distance between two points
# 
def distanceFormula(x1, y1, x2, y2):
	return Math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)
#
# end of function


# function: getDistanceFromCenter
#
# arguments:
#  box: list of box vertex coordinates
#
# return: distance from the center
#
# get distance from center of box created by hard classifier
# 
def getDistanceFromCenter(box):
	return distanceFormula(320, 240, box[0] + box[2], box[1] + box[3])
#
# end of function


# function: getClosest
#
# arguments:
#  boxes: list of boxes created by hard classifier
#
# return: (bottom left x, bottom left y, width, height) of box closest to
#         center
#
# find the box closest to the center of the frame and return it
# 
def getClosest(boxes):

	#  if no boxes, return (0, 0, 0, 0)
	#  
	if len(boxes) == 0:
		return (0,0,0,0)

	# if there is one box, return it
	# 
	elif len(boxes) == 1:
		return boxes[0]

	# if there are multiple boxes, find the closest one
	# 
	else:
		closest = boxes[0]
		distance = getDistanceFromCenter(boxes[0])
		for box in boxes:
			d = getDistanceFromCenter(box)
			if d < distance:
				distance = d
				closest = box
		return closest
#
# end of function

# function: processFrame (video callback used by runloop in main)
#
# arguments:
#  device: pointer to kinect device object
#  frame: numpy rgb image
#  timestamp: integer timestamp
#
# return: none
#
# check the image for a face and get the distance to the center of that face
# 
def processFrame(device, frame, timestamp):

	# initialize globals
	# 
	global s
	global last_depth

	#Apply the frontal classifier:
	#
	frontal = detect.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=5,
									  minSize=(30, 30),
									  flags=cv2.CASCADE_SCALE_IMAGE)

	# get the box closest to the center
	# 
	(x, y, w, h) = getClosest(frontal)
		 
	# where (x,y) is upper vertex of rect. w/h is width/height
	# center of rectangle, draw a circle
	#
	center = ( (x+(w/2)), (y+(h/2)))
	if x !=0:
		cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,153), 5)  
		cv2.circle(frame, (center), 3, (0, 200, 0), -1)

	# if a face was found and there is depth information, print the distance
	# to the face
	#  
	if frontal != () and last_depth != []:
		print last_depth[center[1], center[0]]
		
	# show image
	# 
	cv2.imshow("hurrderr", frame[:,:,::-1])
	cv2.waitKey(5)
	
	# exit gracefully
	#
	return
#
# end of function

# function: storeDepth (depth callback for runloop in main)
#
# arguments:
#  device: pointer to kinect device object
#  frame: numpy rgb image
#  timestamp: integer timestamp
#  
# return: none
#
# store the depth to a list
# 
def storeDepth(dev, depth, time_stamp):

	# store the depth map
	# 
	global last_depth
	last_depth = depth

	# display depth map
	# 
	cv2.imshow("name", depth)
	cv2.waitKey(5)
#
# end of function


def manual_init():
    sdl2.ext.init()
    sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)

    window = sdl2.ext.Window("Gun Control", size=(400, 300))
    window.show()

    joystick = sdl2.SDL_JoystickOpen(0)
    arduino = serial.Serial("/dev/ttyACM4")
    arduino.write("190.0\0")
    
    return window, joystick, arduino

def manual_mainloop(window, joystick, arduino):
    running = True

    while (running):
        events = sdl2.ext.get_events()

        for event in events:
            if event.type == sdl2.SDL_QUIT:
                running = False
                break

        window.refresh()

        axis = sdl2.SDL_JoystickGetAxis(joystick, 0) / 32767.0
        angle = (axis * 90) + 90
        firing = sdl2.SDL_JoystickGetButton(joystick, 0)

        control = 1

        if (arduino.in_waiting):
            message = str(control) + str(firing) + str(angle) + "\0"
            arduino.write(bytes(message))
            arduino.readline()

def manual_quit(joystick, arduino):
    sdl2.SDL_JoystickClose(joystick)
    sdl2.ext.quit()
    arduino.close()

#------------------------------------------------------------------------------
#
# main
#
#------------------------------------------------------------------------------

def main():

	# initialize global variables
	#
	global s
	global last_depth
	s = 0
	
	# initialize the kinect
	#
	kin = fn.open_device(fn.init(), KINECT_PORT)
	fn.set_depth_mode(kin, fn.RESOLUTION_MEDIUM, fn.DEPTH_MM)

	# take an image
	#
	fn.runloop(dev=kin, depth=storeDepth, video=processFrame)

	# exit gracefully
	#
	return
#
# end of main

# begin gracefully
#
if __name__ == "__main__":
	main()
#
# end of program
