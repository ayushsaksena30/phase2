from DroneTerminal import Drone
import cv2
import math
import threading
from time import sleep
from datetime import datetime
import os

drone = Drone(connection_string='127.0.0.1:14550')

contours = {}
approx = []
scale = 2

circle_count = 0 #these are for counting
square_count = 0
triangle_count = 0

target = False #these 4 lines for drone centering
first_time = True

def detect_shape(frame):
  global circle_count, square_count
  frame = cv2.GaussianBlur(frame, (3,3), 0)
  detection = frame
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
      cv2.putText(detection,'TRIANGLE', coords, font, 1, colour, 2)
      target=True
      if y_mid >= middle_y - 10 and y_mid <= middle_y + 10:
        triangle_count+=1
        sleep(0.25)

    if(len(approx)==4):
      shape = "SQUARE"
      cv2.putText(detection,'SQUARE', coords, font, 1, colour, 2)
      target=True
      if y_mid >= middle_y - 10 and y_mid <= middle_y + 10:
        square_count+=1
        sleep(0.25)
    else:
      area = cv2.contourArea(contours[i])
      radius = w/2
      if(abs(1 - (float(w)/h))<=2 and abs(1-(area/(math.pi*radius*radius)))<=0.2):
        cv2.putText(detection,'CIRCLE', coords, font, 1, colour, 2)

        if y_mid >= middle_y - 10 and y_mid <= middle_y + 10:
          circle_count+=1
          coords = drone.get_gps_coords()
          take_snapshot(detection, coords)
          sleep(0.25)

  cv2.line(detection, (0, middle_y), (detection.shape[1], middle_y), (64, 80, 255), 2)

  text = f"Circle: {circle_count} Square: {square_count} Triangle: {triangle_count}"
  cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
  
  cv2.imshow('frame',frame)
  cv2.imshow('detection',detection)
  cv2.imshow('canny',canny)

def take_snapshot(self, frame, coords):
  snapshot_dir = 'snapshots'
  x, y = coords
  text_coords = f"Coords: {x},{y}"
  cv2.putText(frame, text_coords, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
  
  # Get the current time in hh:mm:ss:ms format
  now = datetime.now()
  timestamp = now.strftime("%H:%M:%S:%f")[:-3]  # Get milliseconds as well

  text_time = f"Time: {timestamp}"
  cv2.putText(frame, text_time, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

  os.makedirs(snapshot_dir, exist_ok=True)  # Create directory if it doesn't exist
  snapshot_path = os.path.join(snapshot_dir, f'snapshot_{timestamp}.png')
  cv2.imwrite(snapshot_path, frame)  # Save the frame as an image
  print(f"Snapshot taken and saved at: {snapshot_path}")


def center_drone():
  #code to center drone HERE
  check = False

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
  global target,first_time
  drone.arm_and_takeoff(17)
  with open('hotspot_coords.txt', 'r') as file:
    for line in file:
      data_list = [float(x.strip()) for x in line.split()]
      coord = tuple(data_list)
      if(target and first_time):
        center_drone()
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


#things left here-
# centering drone
# seperately displaying counter frame and snapshot frame
# perfect traversal
# half circle issue still unresolved

#Issues I saw-
# While shape detection, we can't give centers of triangles and squares seperately, 
# it'll be struck there infinitely