from multiprocessing import Process
import cv2

class Vision(Process):
    def __init__(self, vision_data_array, event):
        super().__init__(args=vision_data_array)
        self.vision_data_array = vision_data_array
        self.cap = cv2.VideoCapture(0)
        self.running = event
        self.running.set()

    def run(self):
        while self.running:
            image = cap.read()
            x, y, w, h, image  = findTarget(image)
            self.vision_data_array[0] = x
            self.vision_data_array[1] = y
            self.vision_data_array[2] = w
            self.vision_data_array[3] = h


    def findTarget(self, image):
        x, y, w, h, image = 0.0, 0.0, 0.0, 0.0, None
        return x, y, w, h, image


