import multiprocessing
import os
import time
import cv2
import numpy as np
from wpilib import Resource
import hal

import logging
import argparse

class Vision:
    video_width = 320
    video_height = 240
    video_contrast = 0.5
    video_brightness = 0.5
    video_saturation = 0.5
    video_exposure = 5 # min 3 max 2047
    video_white_balance = 4000# min 2000 max 6500
    video_gain = 0.0
    def __init__(self):
        self._data_array = multiprocessing.Array("d", [0.0, 0.0, 0.0, 0.0, 0.0])
        self.logger = logging.getLogger("vision")
        self._vision_process = VisionProcess(self._data_array)
        self._vision_process.start()
        self.logger.info("Vision process started")
        # Register with Resource so teardown works
        Resource._add_global_resource(self)

    def free(self):
        self._vision_process.terminate()

    def get(self):
        if self._data_array[2] > 0.0 and self._data_array[4] == 1.0:
            # New value and we have a target
            return self._data_array[0:4]
        else:
            return None

    def execute(self):
        pass


class VisionProcess(multiprocessing.Process):
    def __init__(self, data_array):
        super().__init__(args=(data_array,))
        self.vision_data_array = data_array
        self.logger = logging.getLogger("vision-process")
        if hal.HALIsSimulation():
            self.cap = VideoCaptureSim()
        else:
            self.cap = setup_capture(-1)
        self.logger.info(self.cap)
        # Register with Resource so teardown works
        Resource._add_global_resource(self)

    def run(self):
        # counter = 0 # FPS counter
        tm = time.time()
        self.logger.info("Process started")
        while True:
            """ uncomment this and the counter above to get the fps
            counter += 1
            if counter >= 10:
                since = time.time()-tm
                self.logger.info("FPS: "+str(10.0/since))
                tm = time.time()
                counter = 0"""
            success, image = self.cap.read()
            if success:
                x, y, w, h, image = self.findTarget(image)
                self.vision_data_array[:] = [x, y, w, h, 1]
            else:
                self.vision_data_array[:] = [0.0, 0.0, 0.0, 0.0, 1]

    def findTarget(self, image):
        # Convert from BGR colourspace to HSV. Makes thresholding easier.
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # #Define the colours to look for (in HSV)
        lower_colour = np.array([40, 200, 20])
        upper_colour = np.array([110, 255, 150])
        # Create a mask that filters out only those colours
        mask = cv2.inRange(hsv_image, lower_colour, upper_colour)
        # Errode and dialate the image to get rid of noise
        kernel = np.ones((4, 4), np.uint8)
        erosion = cv2.erode(mask, kernel, iterations=1)
        dilated = cv2.dilate(erosion, kernel, iterations=1)
        # Get the information for the contours
        _, contours, __ = cv2.findContours(dilated, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # sort the contours into a list
        areas = [cv2.contourArea(contour) for contour in contours]
        # and retrieve the largest contour in the list
        cnt = None
        try:
            cnt = contours[np.argmax(areas)]
        except ValueError:
            result_image = image
            return 0.0, 0.0, 0.0, 0.0, result_image


        # get the area of the contour
        area = cv2.contourArea(cnt)
        # get the perimeter of the contour
        perimeter = cv2.arcLength(cnt, True)
        # get a rectangle and then a box around the largest countour
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        xy, wh, rotation_angle = cv2.minAreaRect(cnt)
        #box = [np.int0(cv2.boxPoints(cv2.minAreaRect(contour))) for contour in contours]
        cv2.drawContours(image, [box], 0, (0, 0, 255), 2)
        (xy, wh, rotation_angle) = (rect[0], rect[1], rect[2])
        result_image = image
        # Converting the width and height variables to inbetween -1 and 1
        try:
            (x, y) = xy
            (w, h) = wh
        except ValueError:
            return 0.0, 0.0, 0.0, 0.0, result_image
        x = ((2 * x) / Vision.video_width) - 1
        y = ((2 * y) / Vision.video_height) - 1
        return x, y, w, h, result_image

def setup_capture(device_idx=-1):
    cap = cv2.VideoCapture(device_idx)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, Vision.video_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Vision.video_height)
    device_string=str(device_idx)
    if device_idx<0:
        device_idx = str(0)
    os.system("v4l2-ctl -d /dev/video"+device_idx+" -c exposure_auto=1 -c exposure_auto_priority=0 -c exposure_absolute="
            + str(Vision.video_exposure) +" -c white_balance_temperature_auto=0 -c white_balance_temperature=" + str(Vision.video_white_balance))
    # On the Logitech C920 the following options cannot be set:
    # CAP_PROP_EXPOSURE
    # CAP_PROP_HUE

    # The following can be set:
    # CAP_PROP_BRIGHTNESS
    # CAP_PROP_CONTRAST
    # CAP_PROP_SATURATION
    # CAP_PROP_GAIN
    return cap

class VideoCaptureSim():
    def read(self):
        return False, None

# Allow easy capturing of sample images using same settings as on robot
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Capture sample image.')
    parser.add_argument('--device', help='capture device id', default=-1, type=int)
    parser.add_argument('--video', help='display a live video feed of the capture', action='store_true')
    parser.add_argument('file')
    args = parser.parse_args()
    cap = setup_capture(args.device)
    print("Brightness: %f" % cap.get(cv2.CAP_PROP_BRIGHTNESS))
    print("Contrast: %f" % cap.get(cv2.CAP_PROP_CONTRAST))
    print("Saturation: %f" % cap.get(cv2.CAP_PROP_SATURATION))
    print("Gain: %f" % cap.get(cv2.CAP_PROP_GAIN))
    retval, image = cap.read()
    if retval:
        x,y,w,h,image = VisionProcess.findTarget(None, image)
        cv2.imwrite(args.file, image)
    if args.video:
        window = cv2.namedWindow("preview")
        while True:
            retval,image = cap.read()
            img = VisionProcess.findTarget(None, image)
            cv2.imshow("preview", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
