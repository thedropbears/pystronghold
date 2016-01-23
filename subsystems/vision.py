from multiprocessing import Process
import time
import cv2
import numpy as np

import logging
video_width = 320
video_height = 240

class Vision(Process):
    def __init__(self, vision_data_array, event):
        super().__init__(args=vision_data_array)
        self.vision_data_array = vision_data_array
        self.logger = logging.getLogger("vision")
        self.cap = cv2.VideoCapture(-1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, video_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, video_height)
        self.logger.info(self.cap)
        self.running = event
        self.running.set()

    def run(self):
        #counter = 0 # FPS counter
        tm = time.time()
        while self.running.is_set():
            """ uncomment this and the counter above to get the fps
            counter += 1
            if counter >= 10:
                since = time.time()-tm
                self.logger.info("FPS: "+str(10.0/since))
                tm = time.time()
                counter = 0"""
            success, image = self.cap.read()
            if success:
                x, y, w, h, image  = self.findTarget(image)
                self.vision_data_array[0] = x
                self.vision_data_array[1] = y
                self.vision_data_array[2] = w
                self.vision_data_array[3] = h
            else:
                self.vision_data_array[:] = [0.0, 0.0, 0.0, 0.0]


    def findTarget(self, image):
        # Convert from BGR colourspace to HSV. Makes thresholding easier.
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        ##Define the colours to look for (in HSV)
        lower_colour = np.array([40, 55, 210])
        upper_colour = np.array([110, 220, 255])
        #Create a mask that filters out only those colours
        mask = cv2.inRange(hsv_image, lower_colour, upper_colour)
        #Blur and average the mask - smooth the pixels out
        blurred = cv2.GaussianBlur(mask, (3, 3), 3, 3)
        # Errode and dialate the image to get rid of noise
        kernel = np.ones((4,4),np.uint8)
        erosion = cv2.erode(blurred,kernel,iterations = 1)
        dialated = cv2.dilate(erosion,kernel,iterations = 1)
        #Get the information for the contours
        derp, contours, heirarchy = cv2.findContours(dialated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #sort the contours into a list
        areas = [cv2.contourArea(contour) for contour in contours]
        #and retrieve the largest contour in the list
        try:
            max_index = np.argmax(areas)
        except ValueError:
            result_image = image
            return 0.0, 0.0, 0.0, 0.0, result_image
        #define the largest contour
        cnt = contours[max_index]
        #get the area of the contour
        area = cv2.contourArea(cnt)
        #get the perimeter of the contour
        perimeter = cv2.arcLength(cnt, True)
        # get a rectangle and then a box around the largest countour
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        xy,wh,rotation_angle = cv2.minAreaRect(cnt)
        cv2.drawContours(image,[box],0,(0,0,255),2)
        (xy, wh, rotation_angle) = (rect[0], rect[1], rect[2])
        result_image = image
        #Converting the width and height variables to inbetween -1 and 1
        try:
            (x, y) = xy
            (w, h) = wh
        except ValueError:
            return 0.0, 0.0, 0.0, 0.0, result_image
        x = ((2*x)/video_width) - 1
        y = ((2*y)/video_height) - 1
        return x, y, w, h, result_image
