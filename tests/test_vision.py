from subsystems.vision import Vision

import csv
import cv2
import multiprocessing

def test_sample_images():
    tol = 0.05
    variables = ['x', 'y', 'w', 'h']
    vision = Vision(multiprocessing.Array("d", [0.0, 0.0, 0.0, 0.0, 0.0]), multiprocessing.Event(), multiprocessing.Lock())
    with open('sample_img/tests.csv', 'r') as csvfile:
        # filename, x, y, w, h
        testreader = csv.reader(csvfile, delimiter=',')
        for sample in testreader:
            image = cv2.imread('sample_img/' + sample[0])
            # Rescale if necessary
            scaled = cv2.resize(image, (Vision.video_width, Vision.video_height))
            results = vision.findTarget(scaled)
            for i in range(4):
                assert abs(results[i] - float(sample[i + 1])) < tol, "Incorrect %s value for image %s" % (variables[i], sample[0])
