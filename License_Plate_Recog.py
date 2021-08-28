import os
import cv2
import imutils
import numpy as np
import pytesseract
from PIL import Image
from cv2 import *
import RPi.GPIO as GPIO
import time

Database=["KA 03 AB 3289","HR 26 DA 2330"]

#Initializing Servo Motor
GPIO.setwarnings(False)
servo = 24
GPIO.setmode(GPIO.BOARD)
GPIO.setup(servo,GPIO.OUT)
p=GPIO.PWM(servo,50)# 50hz frequency
p.start(2.0)
print("Code by JPF07") 
def gateClose():
    p.ChangeDutyCycle(2.0)
    time.sleep(1)
    irSensor()

def gateOpen():
    time.sleep(1)
    p.ChangeDutyCycle(9.0)
    time.sleep(10)
    ircheck()
    gateClose()    
    
    
def ircheck():
    
    sensor = 16
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(sensor,GPIO.IN)
    print (" ")
    while GPIO.input(sensor)==False:
        time.sleep(.3)
    print ("Path is clear") 
    time.sleep(4)  


def recog() :

        img = cv2.imread('NewPicture.jpg',cv2.IMREAD_COLOR)

        img = cv2.resize(img, (620,480) )

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #convert to grey scale
        gray = cv2.bilateralFilter(gray, 11, 17, 17) #Blur to reduce noise
        edged = cv2.Canny(gray, 30, 200) #Perform Edge detection

        # find contours in the edged image, keep only the largest
        # ones, and initialize our screen contour
        cnts = cv2.findContours(edged.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
        screenCnt = None

        # loop over our contours
        for c in cnts:
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.018 * peri, True)
        
            # if our approximated contour has four points, then
            # we can assume that we have found our screen
            if len(approx) == 4:
                screenCnt = approx
                break

        if screenCnt is None:
            detected = 0
            print("No contour detected")
        else:
            detected = 1

        if detected == 1:
            cv2.drawContours(img, [screenCnt], -1, (0, 255, 0), 3)

        # Masking the part other than the number plate
        mask = np.zeros(gray.shape,np.uint8)
        new_image = cv2.drawContours(mask,[screenCnt],0,255,-1,)
        new_image = cv2.bitwise_and(img,img,mask=mask)

        # Now crop
        (x, y) = np.where(mask == 255)
        (topx, topy) = (np.min(x), np.min(y))
        (bottomx, bottomy) = (np.max(x), np.max(y))
        Cropped = gray[topx:bottomx+1, topy:bottomy+1]
        #Read the number plate
        text = pytesseract.image_to_string(Cropped, config='--psm 11')
        print("Detected Number is:",text)
        cv2.destroyAllWindows() # These 4 lines can be eliminated
        os.remove("NewPicture.jpg") # Removing Temporary image file 
        return text


def auth(text):

        flag=0
        for x in range (len(Database)):
            for i in range(len(Database[x])):
                if Database[x][i]==text[i]:
                    if len(Database[x])-1==i:
                        flag=1
                        print("enthada mwonuse nokkunne...")
                    continue 
                else :
                    flag=0
                    break
            if flag==0:
                continue
            else :
                break               

        if flag==0:
            print("Gate can not be opened")
            irSensor()
        else:
            print("Open the gate")
            gateOpen()
                        
def capture():
    if os.path.exists("NewPicture.jpg"):
        os.remove("NewPicture.jpg") # Removing Temporary image file 

    cap = cv2.VideoCapture(0)  
    ret, frame = cap.read()
    cv2.imwrite("NewPicture.jpg",frame)
    cap.release()
    cv2.destroyAllWindows() 
    try:
        auth(recog())
    except:
        print('No Vehicle Detected')
        irSensor()

def irSensor():
    sensor = 16
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(sensor,GPIO.IN)
    print ("IR Sensor Ready.....")
    print (" ")
    while GPIO.input(sensor):
        time.sleep(.3)
    print ("Object Detected")
    capture()
    
irSensor()
