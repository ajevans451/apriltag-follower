import cv2
import numpy as np
from apriltag import apriltag

imagepath = 'test_image.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = apriltag("tagStandard36h11")

detections = detector.detect(image)
