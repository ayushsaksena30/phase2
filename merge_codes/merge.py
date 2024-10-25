from dev3 import MinimalSubscriber 
from DroneTerminal import Drone
import cv2
import threading
from time import sleep

drone = Drone(connection_string='127.0.0.1:14550')

contours = {}
approx = []
scale = 2
circle_count = 0
square_count = 0

def detect_shape(frame):
  global circle_count, square_count
  frame = cv2.GaussianBlur(frame, (3,3), 0)
  #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
  canny = cv2.Canny(frame,80,240,3)
  middle_y = frame.shape[0] // 2

  contours, hierarchy = cv2.findContours(canny,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
  for i in range(0,len(contours)):

    approx = cv2.approxPolyDP(contours[i],cv2.arcLength(contours[i],True)*0.02,True)

    if(abs(cv2.contourArea(contours[i]))<100 or not(cv2.isContourConvex(approx))):
      continue

    x, y, w, h= cv2.boundingRect(approx)
    x_mid = int(x + (w/3))
    y_mid = int(y + (h/1.5))

    middle_y = frame.shape[0] // 2
    #center_y = y + h // 2

    coords = (x_mid, y_mid)
    colour = (0, 0, 0)
    font = cv2.FONT_HERSHEY_DUPLEX

    if(len(approx)==3):
      shape = "TRIANGLE"
      cv2.putText(frame,'SQUARE', coords, font, 1, colour, 2)
      if y_mid >= middle_y - 10 and y_mid <= middle_y + 10:
        square_count+=1
        sleep(0.25)

    if(len(approx)==4):
      shape = "SQUARE"
      cv2.putText(frame,'SQUARE', coords, font, 1, colour, 2)

      if y_mid >= middle_y - 10 and y_mid <= middle_y + 10:
        square_count+=1
        sleep(0.25)
    else:
      area = cv2.contourArea(contours[i])
      radius = w/2
      if(abs(1 - (float(w)/h))<=2 and abs(1-(area/(math.pi*radius*radius)))<=0.2):
        cv2.putText(frame,'CIRCLE', coords, font, 1, colour, 2)

        if y_mid >= middle_y - 10 and y_mid <= middle_y + 10:
          circle_count+=1
          sleep(0.25)

  cv2.line(frame, (0, middle_y), (frame.shape[1], middle_y), (64, 80, 255), 2)

  text = f"Circle: {circle_count} Square: {square_count}"
  cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
  
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

def traversal(self):
  drone.arm_and_takeoff(17)
  with open('hotspot_coords.txt', 'r') as file:
    for line in file:
      data_list = [float(x.strip()) for x in line.split()]
      coord = tuple(data_list)
      
      drone.goto_gps(coord)
      sleep(0.02)
      print("Reached location : ", coord)
      sleep(1.5)

t1 = threading.Thread(target=main, args=None)
t2 = threading.Thread(target=traversal, args=None)

t1.start()
t2.start()
t1.join()
t2.join()