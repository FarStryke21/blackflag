#!/usr/bin/env python3

import cv2
import roboflow
from roboflow import Roboflow
import sys

#url = "http://localhost:9001/mechatronics2/1?api_key=uKIsEXbOt27HBKcS5KAo"
#model_name = "Mechatronics2"
#headers = {"Content-Type":"application/json"}

def predict(image):
    outputs = clf.predict(image, confidence = 30, overlap = 80)
    return outputs

image = sys.argv[1]
rf = Roboflow(api_key="uKIsEXbOt27HBKcS5KAo")
project = rf.workspace().project("mechatronics2")
clf = project.version(1).model
predict(image)
