# -*- coding: utf-8 -*-
"""
Created on Wed Feb 20 11:38:24 2019

@author: Rohini
"""

from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
 
def decode(im) : 
  # Find barcodes and QR codes
  # thresholds image to white in back then invert it to black in white
  #   try to just the BGR values of inRange to get the best result
#  mask = cv2.inRange(im,(0,0,0),(200,200,200))
#  thresholded = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)
#  inverted = 255-thresholded
  qr_data = []
  decodedObjects = pyzbar.decode(im)
  # Print results
  for obj in decodedObjects:
      if obj.data not in qr_data:
          qr_data.append(obj.data)
#          print('Data : ', obj.data,'\n')
#    print('Type : ', obj.type)
#    print('Data : ', obj.data,'\n')
  return decodedObjects, qr_data
 
 
# Display barcode and QR code location  
def display(im, decodedObjects):
 
  # Loop over all decoded objects
  for decodedObject in decodedObjects: 
    points = decodedObject.polygon
 
    # If the points do not form a quad, find convex hull
    if len(points) > 4 : 
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else : 
      hull = points;
     
#    # Number of points in the convex hull
#    n = len(hull)
 
#    # Draw the convext hull
#    for j in range(0,n):
#      cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)
  
    # draw text    
    x = decodedObject.rect.left
    y = decodedObject.rect.top    
    w = decodedObject.rect.width 
    h = decodedObject.rect.height
    
    # draw bounding box
    cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,255),2)
    barCode = str(decodedObject.data)
    cv2.putText(im, barCode, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 1, cv2.LINE_AA)
 
   
# Main 
if __name__ == '__main__':
 
  # Read image
  im = cv2.imread('QR_codes_4.PNG') #This image contains both bar codes and QR codes, but the ouput is able to detect only the QR codes
 
  decodedObjects = decode(im)
  display(im, decodedObjects)