#!/usr/bin/env python
import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if (cap.isOpened()== False): 
  print("Error opening video stream or file")

while(cap.isOpened()):
  ret, frame = cap.read()
  if ret == True:
    cv2.imshow('Frame',frame)
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
  else: 
    break
cap.release()
cv2.destroyAllWindows()