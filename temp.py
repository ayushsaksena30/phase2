#This will be the final code, right now it contains the code from GoodCanny which has perfect Canny detection but not consistent shapes.
#Right now I am trying to count shapes when it crosses a fixed line.
import cv2
import math

contours = {}
approx = []
scale = 2

def count_shape(frame):
  shape=[{'TRIANGLE': 0, 'SQUARE': 0, 'CIRCLE': 0}]
  height, width, channels = frame.shape
  line_position = height-50  # Y-coordinate for a horizontal line
  offset = 30  # Offset to prevent double counting

  # Initialize count
  crossing_count = 0

  # Create a list to store the center positions of detected objects
  object_centers = []

  cv2.line(frame, (0, line_position), (frame.shape[1], line_position), (0, 255, 0), 2)

  # Display the count on the frame
  cv2.putText(frame, f"Count: {crossing_count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

  # Show the frame
  cv2.imshow("Object Counter", frame)



def detect_shape(frame):
  gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  canny = cv2.Canny(frame,80,240,3)

  contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  for i in range(0,len(contours)):

    approx = cv2.approxPolyDP(contours[i],cv2.arcLength(contours[i],True)*0.02,True)

    if(abs(cv2.contourArea(contours[i]))<100 or not(cv2.isContourConvex(approx))):
      continue

    x, y, w, h= cv2.boundingRect(approx)
    x_mid = int(x + (w/3))
    y_mid = int(y + (h/1.5))

    coords = (x_mid, y_mid)
    colour = (0, 0, 0)
    font = cv2.FONT_HERSHEY_DUPLEX

    if(len(approx) == 3):
      cv2.putText(frame,'TRIANGLE', coords, font, 1, colour, 4)
    elif(len(approx)==4):
      cv2.putText(frame,'SQUARE', coords, font, 1, colour, 4)
    else:
      area = cv2.contourArea(contours[i])
      radius = w/2
      if(abs(1 - (float(w)/h))<=2 and abs(1-(area/(math.pi*radius*radius)))<=0.2):
        cv2.putText(frame,'CIRCLE', coords, font, 1, colour, 4)
  
  cv2.imshow('frame',frame)
  cv2.imshow('canny',canny)

def main():
  vid = cv2.VideoCapture(1)

  while True:
    ret, frame = vid.read()
    if not ret:
      break

    detect_shape(frame)
    if cv2.waitKey(1) & 0xFF == ord('d'):
      break
  
  vid.release()
  cv2.destroyAllWindows()

main()