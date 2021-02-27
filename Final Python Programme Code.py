# -*- coding: utf-8 -*-
"""
@author: muhammad helmi bin hammidon
"""


#importing packages
import RPi.GPIO as GPIO
import time
from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import cv2
from scipy.spatial import distance as dist
import numpy as np
from imutils import perspective

#servo motor change duty cycle / angle function
def servomotor(i,j,k):
    s1.ChangeDutyCycle(i)
    s2.ChangeDutyCycle(j)
    s3.ChangeDutyCycle(k)

#stepper motor rotation function
def steppermove (c,d):
    i=0
    if d==1:
       GPIO.output(stepperpinB, GPIO.HIGH)
    elif d==0:
       GPIO.output(stepperpinB, GPIO.LOW)
    else:
       GPIO.output(stepperpinB, GPIO.LOW)
    while i<=c:
        GPIO.output(stepperpinA, GPIO.HIGH)
        time.sleep(0.005)
        GPIO.output(stepperpinA, GPIO.LOW)
        time.sleep(0.005)
        i = i + 1
  
#function to determine midpoint
def midpoint(point1, point2):
    return ((point1[0] + point2[0]) * 0.5, (point1[1] + point2[1]) * 0.5)

#function for initializing reference background image
def refer_image():
    camera.capture(RAWformat, format="bgr")
    tempimage = RAWformat.array
    gray = cv2.cvtColor(tempimage, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (21, 21), 0)
    avgtemp = gray.copy().astype("float")
    return avgtemp

#servo motor declare pins and initaization
servoA = 17
servoB = 27
servoC = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoA, GPIO.OUT)
GPIO.setup(servoB, GPIO.OUT)
GPIO.setup(servoC, GPIO.OUT)
s1 = GPIO.PWM(servoA, 50) 
s2 = GPIO.PWM(servoB, 50)
s3 = GPIO.PWM(servoC, 50)
s1.start(10)
s2.start(6)
s3.start(10)

#stepper motor declare pins and initialization
stepperpinA = 15
stepperpinB = 18
GPIO.setup(stepperpinA, GPIO.OUT)
GPIO.setup(stepperpinB, GPIO.OUT)

# allow the camera to warmup, and initialize the image parameter
camera = PiCamera()
camera.resolution = (640, 480)
RAWformat = PiRGBArray(camera, size=(640, 480))
print("[INFO] warming up camera...")
time.sleep(1)
backimage = None
pixelsPerMetric = 296/57.6 
counterA=0
counterB=0

#capture and initalize reference picture
print("initialize first picture")
backimage=refer_image()
cv2.imshow("first", backimage)
print("ready for egg, press q")
key = cv2.waitKey(0) & 0xFF
cv2.destroyAllWindows()
RAWformat.truncate(0)

#main loop for grading the egg, check 'q' key is pressed, else exit
while key == ord("q"):
       
    #rotate the stepper motor, egg inserted into the slot
    #stop the egg perpendicularly with the camera
    steppermove(199,0)
    time.sleep(1)
    steppermove(99,1)
    time.sleep(1)
    steppermove(99,0)
    time.sleep(2)
    
    #capture egg photo and convert to gray image
    camera.capture(RAWformat, format="bgr")
    eggimage = RAWformat.array
    grayegg = cv2.cvtColor(eggimage, cv2.COLOR_BGR2GRAY)
    grayegg = cv2.GaussianBlur(grayegg, (21, 21), 0)

    # calculate average value between current and previous image
    # find difference between current and average
    cv2.accumulateWeighted(grayegg, backimage, 0.5)
    diffimage = cv2.absdiff(grayegg, cv2.convertScaleAbs(backimage))
   
    # threshold image to amplify object(egg) and dilate to smooth the image 
    threshimg = cv2.threshold(diffimage, 20, 255,cv2.THRESH_BINARY)[1]
    threshimg = cv2.dilate(threshimg, None, iterations=2)
       
    #find contour on thresholded image
    cnts=cv2.findContours(threshimg.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    #assign counter B equal to counter A
    counterB=counterA
    
    # loop for all contours found
    for c in cnts:
        
        #find the contour area and print it in terminal
        valuearea=cv2.contourArea(c)
        print("value contour={}".format(valuearea))
        
        # if the contour area less than 10000, ignore it
        if valuearea < 10000 :
            continue
             
        #if the egg is found, increase counter by 1 and print it
        counterA=counterA+1
        print("egg {}".format(counterA))
 
        # the grading selection depends on the contour value
        #and set the servo angle
        if valuearea <= 49500 :
            print("grade C")
            grade='C'
            servomotor(7,6,7)
        elif valuearea > 49500 and valuearea <= 53500 :
            print("grade B")
            grade='B'
            servomotor(10,6,10)
        elif valuearea > 53500 :
            grade='A'
            print("grade A")
            servomotor(11,10,10)
        
        time.sleep(2)
            
        # move the stepper motor so the egg can get into the sorting
        steppermove(199,0)
        
        # make bounding box around contour
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")
        
        # arrange box point (top left, top right, bottom right, bottom left) 
        box = perspective.order_points(box)
        
        #draw box
        display = eggimage.copy()
        cv2.drawContours(display, [box.astype("int")], -1, (0, 255, 0), 2)
        
        # draw box corner point 
        for (x, y)in box:
            cv2.circle(display, (int(x), int(y)), 5, (0, 0, 255), -1)

        # calculate midpoint between each lengths
        (topleft, topright, botright, botleft) = box
        (topleftright_X, topleftright_Y) = midpoint(topleft, topright)
        (botleftright_X, botleftright_Y) = midpoint(botleft, botright)

        # calculate midpoint between each widths
        (topbotleft_X, topbotleft_Y) = midpoint(topleft, botleft)
        (topbotright_X, topbotright_Y) = midpoint(topright, botright)

        # draw the midpoint
        cv2.circle(display, (int(topleftright_X), int(topleftright_Y)), 5, (255, 0, 0), -1)
        cv2.circle(display, (int(botleftright_X), int(botleftright_Y)), 5, (255, 0, 0), -1)
        cv2.circle(display, (int(topbotleft_X), int(topbotleft_Y)), 5, (255, 0, 0), -1)
        cv2.circle(display, (int(topbotright_X), int(topbotright_Y)), 5, (255, 0, 0), -1)

        # make line connecting midpoint
        cv2.line(display, (int(topleftright_X), int(topleftright_Y)), (int(botleftright_X), int(botleftright_Y)),(255, 0, 255), 2)
        cv2.line(display, (int(topbotleft_X), int(topbotleft_Y)), (int(topbotright_X), int(topbotright_Y)),(255, 0, 255), 2)
        
        # calculate distance between the midpoints
        length = dist.euclidean((topleftright_X, topleftright_Y), (botleftright_X, botleftright_Y))
        width = dist.euclidean((topbotleft_X, topbotleft_Y), (topbotright_X, topbotright_Y))

        # compute the approximate length and width of the object 
        length = length / pixelsPerMetric
        width = width / pixelsPerMetric
        
        # put the length and width of objects
        cv2.putText(display, "{:.3f}mm".format(length), (int(topleftright_X - 15), int(topleftright_Y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(display, "{:.3f}mm".format(width), (int(topbotright_X + 10), int(topbotright_Y)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
        cv2.putText(display, "Grade:{}".format(grade), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # initialize reference image and show the output image
        RAWformat.truncate(0)
        backimage = refer_image()
        cv2.imshow("Display Image", display)
    
    #if no egg, show output image with no egg    
    if counterB==counterA:
        RAWformat.truncate(0)
        backimage = refer_image()
        print(" no egg")
        grade='no egg'
        cv2.putText(eggimage, "Grade:{}".format(grade), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        cv2.imshow("Display Image", eggimage)
    
    #require user to put the egg into slot and press'q' key
    print(" press q for next egg")
    key = cv2.waitKey(0) & 0xFF
    cv2.destroyAllWindows()
    RAWformat.truncate(0)
       
#exit program by closing all the windows opened    
cv2.destroyAllWindows()
RAWformat.truncate(0)
