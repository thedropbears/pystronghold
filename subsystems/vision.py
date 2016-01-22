from multiprocessing import Process
import time
import cv2
import numpy as np

import logging
videoWidth = 320
videoHeight = 240

class Vision(Process):
    def __init__(self, vision_data_array, event):
        super().__init__(args=vision_data_array)
        self.vision_data_array = vision_data_array
        self.logger = logging.getLogger("vision")
        self.cap = cv2.VideoCapture(-1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, videoWidth)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, videoHeight)
        self.logger.info(self.cap)
        self.running = event
        self.running.set()

    def run(self):
        counter = 0
        tm = time.time()
        while self.running.is_set():
            counter += 1
            if counter >= 10:
                since = time.time()-tm
                self.logger.info("FPS: "+str(10.0/since))
                tm = time.time()
                counter = 0
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
        lower_colour = np.array([65, 30, 220])
        upper_colour = np.array([80, 110, 255])


        #Create a mask that filters out only those colours
        mask = cv2.inRange(hsv_image, lower_colour, upper_colour)

        #Blur and average the mask - smooth the pixels out
        blurred = cv2.GaussianBlur(mask, (3, 3), 3, 3)

        #Get the information for the contours
        derp, contours, heirarchy = cv2.findContours(blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

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

        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        xy,wh,rotation_angle = cv2.minAreaRect(cnt)

        cv2.drawContours(image,[box],0,(0,0,255),2)
            #Now that we have an OBB we can get its vital stats to return to the
            # caller. Remember that these numbers need to be independent of the size
            # of the image (we can't return them in pixels). Scale everything relative
            # to the image - between [-1, 1]. So (1,1) would be the top right of the
            # image, (-1,-1) bottom left, and (0,0) dead centre.
            ## x = ???
            ## y = ???
            ## w = ???
            ## h = ???
            ## angle = ???

            # We can return an altered image so that we can check that things are
            # working properly.
            ## result_image = <something with image and one of the intermediate steps -
            ##                 mask, blurred, contours, obb, etc>
        (xy, wh, rotation_angle) = (rect[0], rect[1], rect[2])

        result_image = image
        #Converting the width and height variables to inbetween -1 and 1
        try:
            (x, y) = xy
            (w, h) = wh

        except ValueError:
            return None, None, None, None, rotation_angle, result_image
        x = ((2*x)/videoWidth) - 1
        y = ((2*y)/videoHeight) - 1
        return x, y, w, h, result_image




