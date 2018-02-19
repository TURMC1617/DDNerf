import cv2
import freenect as fn



import time
import numpy as np
import sys
import sd12
import sd12.ext
import serial

import threading


# Global Variables:
KINECT_PORT = 0
PLAY_AUDIO= True



def play():
    global sound_on
    while True:
