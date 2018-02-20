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

# computer vision
import cv2

# kinect
import freenect as fn

# general 
import time
import numpy as np
import scipy.misc
import sys
import sdl2
import sdl2.ext
import serial
# import winsound #to play sound effects
import threading

#------------------------------------------------------------------------------
#
# globals
#
#------------------------------------------------------------------------------

detect = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

# kinect stuff
KINECT_PORT = 0

# audio
PLAY_AUDIO = True

# window variables
window_size = (400, 300)

def distanceFormula(x1, y1, x2, y2):
	return Math.sqrt((y2 - y1) ** 2 + (x2 - x1) ** 2)


def getDistanceFromCenter(box):
	return distanceFormula(320, 240, box[0] + box[2], box[1] + box[3])


def getClosest(boxes):
	#  if no boxes, return (0, 0, 0, 0)
	if len(boxes) == 0:
		return (0,0,0,0)

	# if there is one box, return it
	elif len(boxes) == 1:
		return boxes[0]
	# if there are multiple boxes, find the closest one
	else:
		closest = boxes[0]
		distance = getDistanceFromCenter(boxes[0])
		for box in boxes:
			d = getDistanceFromCenter(box)
			if d < distance:
				distance = d
				closest = box
		return closest


def processFrame(frame, depth):
	#Apply the frontal classifier:
	frontal = detect.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=5,
									  minSize=(30, 30),
									  flags=cv2.CASCADE_SCALE_IMAGE)

	# get the box closest to the center
	(x, y, w, h) = getClosest(frontal)
		 
	# where (x,y) is upper vertex of rect. w/h is width/height
	# center of rectangle, draw a circle
	center = ( (x+(w/2)), (y+(h/2)))
	if x !=0:
		cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,153), 5)  
		cv2.circle(frame, (center), 3, (0, 200, 0), -1)

	# if a face was found and there is depth information, return
	# screen space coordinates (normalized from -1 to 1)
	if frontal != () and depth != []:
		x = ((center[0] / frame.shape[0]) - 0.5) * 2
		y = ((center[1] / frame.shape[1]) - 0.5) * 2
		return (x, y, depth[center[1], center[0]])

	return None

def get_angle():
	# Transform x, y, z coordinates using inverse of camera projection
	# matrix
	# Angle = atan(z / x)
	pass

def window_init():
	sdl2.ext.init()
	sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK)

	window = sdl2.ext.Window("Full Auto Frank", size=window_size)
	window.show()

	return window

def manual_init():
	joystick = sdl2.SDL_JoystickOpen(0)
	arduino = 0
	# arduino = serial.Serial(sys.argv[1])
	# arduino.write("190.0\0")

	camera = cv2.VideoCapture(0)
	
	return joystick, arduino, camera

def render_image(window_array, img):
	img = np.insert(img, 3, 255, axis=2)
	img = np.rot90(img)
	img = scipy.misc.imresize(img, window_size)
	np.copyto(window_array, img)

def mainloop(window, joystick, arduino, camera, kinect):
	running = True
	window_array = sdl2.ext.pixels3d(window.get_surface())
	current_angle = 0

	while (running):
		events = sdl2.ext.get_events()

		# Handle events
		for event in events:
			if event.type == sdl2.SDL_QUIT:
				running = False
				break

		# Get joystick angle
		axis = sdl2.SDL_JoystickGetAxis(joystick, 0) / 32767.0
		angle = (axis * 90) + 90
		firing = sdl2.SDL_JoystickGetButton(joystick, 0)

		control = 1

		# if (arduino.in_waiting):
		# 	message = str(control) + str(firing) + str(angle) + "\0"
		# 	arduino.write(bytes(message))
		# 	angle = float(arduino.readline())

		# Render image
		# _, im = camera.read()
		# render_image(window_array, im)

		rgb, _ = fn.sync_get_video()
		depth, _ = fn.sync_get_depth()
		
		pos = processFrame(rgb, depth)
		if (pos is not None):
			print get_angle(pos)

		render_image(window_array, rgb)

		window.refresh()

def manual_quit(joystick, arduino, camera):
	sdl2.SDL_JoystickClose(joystick)

	sdl2.SDL_DestroyTexture(texture)
	sdl2.SDL_DestroyRenderer(renderer.renderer)

	sdl2.ext.quit()

	arduino.close()


def vision_init():	
	# initialize the kinect
	kin = fn.open_device(fn.init(), KINECT_PORT)
	fn.set_depth_mode(kin, fn.RESOLUTION_MEDIUM, fn.DEPTH_MM)

	return kin

def main():
	window = window_init()
	joystick, arduino, camera = manual_init()
	kin = vision_init()
	mainloop(window, joystick, arduino, camera, kin)
	manual_quit(joystick, arduino, camera)

if __name__ == "__main__":
	main()
