#!/usr/bin/env python

import cv2 #OpenCV
import time
from socket import * #Networking Stuff
import winsound #to play sound effects
import threading

#state variables regarding audio
s = 0
#audio stuff
audioplay = True
#Audio stuff
#def play():
#    global s
#    while True:
#        if (s == 1):
#            winsound.PlaySound('fire.wav',winsound.SND_FILENAME)
#            print s
#        if (audioplay == False):
#            break


#Load frame, apply front face haar classifier, and check to see if identified rectangle is in center of frame/center of gun
def processframe():
    global s
    #grab frame
    retval, frame = cam.read()
    
    #Init Haar Frontal Face Classifier
    detect = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
    #Alternative Classifier:
    #upper = cv2.CascadeClassifier('haarcascade_frontalface_alt2.xml')
    #Apply the frontal classifier:
    frontal = detect.detectMultiScale(frame,scaleFactor=1.3, minNeighbors=5, minSize=(30, 30), flags=cv2.CASCADE_SCALE_IMAGE)

    #frontal is an array full of rectangle coords regarding identified objects
    for (x,y,w,h) in frontal:
        #where (x,y) is upper vertex of rect. w/h is width/height
        #center of rectangle, draw a circle
        center = ( (x+(w/2)), (y+(h/2)))
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0,0,153), 5)	
        cv2.circle(frame, (center), 3, (0, 200, 0), -1)
        #Grab center of rectangle (only x is needed) and test to see if it falls within
        center = ( (x+(w/2)))
		#print(center)
        #Send pysocket commands to motorcontroller/pi regarding situation
        if (center < 390 and center > 250 ):
            cv2.putText(frame, "Target Found", (20, 30), cv2.FONT_HERSHEY_DUPLEX, 0.5,(27, 204, 4), 2)
            clientSocket.sendto('shoot'.encode(),serverAddress)
            s = 1
        elif (center > 390):
            clientSocket.sendto('right'.encode(),serverAddress)
            s = 0
        elif (center < 250):
            clientSocket.sendto('left'.encode(),serverAddress)
            s = 0
    cv2.imshow("hurrderr", frame)


#Pi/Motor Controller's IP and Port
targetIP = '192.168.1.100'
port = 12000


#Init socket
clientSocket = socket(AF_INET,SOCK_DGRAM)
serverAddress = (targetIP, port)






#OpenCV int webcam
port = int(raw_input("Camera Port: "))
cam = cv2.VideoCapture(port)

cam.release()
cam.open(port)
time.sleep(1)
#Initial command to get gun moving
clientSocket.sendto('right'.encode(),serverAddress)

sound_thread = threading.Thread(target=lambda:play())
sound_thread.start()
while True:
    processframe()
    if cv2.waitKey(30) == 27:
        clientSocket.sendto('stopTurning'.encode(),serverAddress)
        clientSocket.sendto('stopShooting'.encode(),serverAddress)
        audioplay = False
        break
cam.release()
cv2.DestoryAllWindows()
