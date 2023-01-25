# import the necessary packages
import numpy as np
import argparse
import cv2
import time

import picamera.array
import picamera

camera=picamera.PiCamera()
camera.resolution=(1088,720)
rawCapture=picamera.array.PiRGBArray(camera)
for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
	cameraFeed=frame.array

	# load the image, clone it for output, and then convert it to grayscale
			
	output = cameraFeed.copy()
	gray = cv2.cvtColor(cameraFeed, cv2.COLOR_BGR2GRAY)
	
	# apply GuassianBlur to reduce noise. medianBlur is also added for smoothening, reducing noise.
	gray = cv2.GaussianBlur(gray,(5,5),0);
	gray = cv2.medianBlur(gray,5)
	
	# Adaptive Guassian Threshold is to detect sharp edges in the Image. For more information Google it.
	thres = cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
            cv2.THRESH_BINARY,11,3.5)
	cv2.imshow('gray',thres)
	kernel = np.ones((2,2),np.uint8)
	gray = cv2.erode(thres,kernel,iterations = 1)
	# gray = erosion
	
	gray = cv2.dilate(gray,kernel,iterations = 1)
	# gray = dilation

	# get the size of the final image
	# img_size = gray.shape
	# print img_size
	
	# detect circles in the image
	circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 200, param1=30, param2=45, minRadius=0, maxRadius=0)
	# print circlescv2.imshow('gray',gray)
	
	# ensure at least some circles were found
	if circles is not None:
		# convert the (x, y) coordinates and radius of the circles to integers
		circles = np.round(circles[0, :]).astype("int")
		
		# loop over the (x, y) coordinates and radius of the circles
		for (x, y, r) in circles:
			# draw the circle in the output image, then draw a rectangle in the image
			# corresponding to the center of the circle
			cv2.circle(output, (x, y), r, (0, 255, 0), 4)
			cv2.rectangle(output, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
		

	# Display the resulting frame
        #cv2.imshow('gray',gray)
        #cv2.imshow('frame',output)
    	
 	if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rawCapture.truncate()

# When everything done, release the capture

