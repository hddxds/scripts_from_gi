import cv2
import numpy as np
 
# Create a VideoCapture object and read from input file
# If the input is the camera, pass 0 instead of the video file name

#name = '/dev/video1'
name = 1
print("Using: ", name)
cap = cv2.VideoCapture()
cap.open(name)
set_success = cap.set(3,1280.0)
print 'set_success:',set_success
set_success = cap.set(4,480.0)
print 'set_success:',set_success

#import pdb;pdb.set_trace()

# Check if camera opened successfully
if (cap.isOpened()== False): 
  print("Error opening video stream or file")
 
# Read until video is completed
while(cap.isOpened()):
  # Capture frame-by-frame
  #set_success = cap.set(3,1280)
  #print 'set_success:',set_success

  ret, frame = cap.read()

  if ret == True:
 
    # Display the resulting frame
    print cap.get(3)
    print cap.get(4)
    print frame.shape
    cv2.imshow('Frame',frame)
 
    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
 
  # Break the loop
  else: 
    break
 
# When everything done, release the video capture object
cap.release()
 
# Closes all the frames
cv2.destroyAllWindows()
