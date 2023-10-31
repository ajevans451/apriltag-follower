import cv2
import numpy as np
from dt_apriltags import Detector

SHOWVISUALS=True


imagepath = 'test_image.jpg'
image = cv2.imread(imagepath, cv2.IMREAD_GRAYSCALE)
detector = Detector("tag36h11")

detections = detector.detect(image)

if SHOWVISUALS==True:
    cv2.imshow("Greyscale Original",image) # greyscale image
    k=cv2.waitKey(0)
    if k==27:
        cv2.destroyAllWindows

color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
for detection in detections:
    for idx in range(len(detection.corners)):
        cv2.line(color_image, tuple(detection.corners[idx-1, :].astype(int)), tuple(detection.corners[idx, :].astype(int)), (0,255,0), 3)

if SHOWVISUALS==True:
    cv2.imshow("Tags Detected", color_image)
    k=cv2.waitKey(0)
    if k==27:
        cv2.destroyAllWindows()



