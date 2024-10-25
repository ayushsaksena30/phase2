from ultralytics import YOLO
from DroneTerminal import Drone
from time import sleep
import threading
from datetime import datetime
import os
import cv2 as cv
from pymavlink import mavutil

drone = Drone(connection_string='127.0.0.1:14550')
drone.speed(3)
altitude = 15

target_x=target_y = 0

gnd_speed = 1

model = YOLO('yolov8n.pt')
model.fuse()
hotspot=0

vid = cv.VideoCapture(0)

def camera():
  global target_x,target_y
  while True:
    ret,frame = vid.read()
    results = model(frame)
    annotated_frame = results[0].plot()
    height, width, _ = annotated_frame.shape

    cv.line(annotated_frame, (0, height // 2), (width, height // 2), (255, 255, 255), 2)

    center_frame_x = width // 2
    center_frame_y = height // 2

    for result in results[0].boxes:
      # Get the center coordinates of the bounding box
      x1, y1, x2, y2 = result.xyxy[0].tolist()
      center_x = int((x1 + x2) / 2)
      center_y = int((y1 + y2) / 2)

      # Get box coordinates
      cls = result.cls[0]  # Class ID
      if(cls!=29):
          continue
      # # Draw bounding box on the image

      target_x=center_x-center_frame_x
      target_y=center_y-center_frame_y
      label = f'Class: {int(cls)}'
      cv.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (255, 0, 0), 2)
      cv.putText(frame, label, (int(x1), int(y1) - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

      # # Mark the center of the detected object
      if center_y >= center_frame_y - 10 and center_y <= center_frame_y + 10:
          hotspot+=1
          coords = drone.get_gps_coords()
          take_snapshot(annotated_frame,coords)
          sleep(1.5)

    text = f"Counter: {hotspot}"
    cv.putText(frame, text, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    
    cv.imshow("Image", annotated_frame)
    cv.imshow('Camera', frame)
    cv.waitKey(0)
  vid.release()
  cv.destroyAllWindows()

def take_snapshot(frame, coords):
  snapshot_dir = 'snapshots'
  x, y = coords
  text_coords = f"Coords: {x},{y}"
  cv.putText(frame, text_coords, (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
  
  # Get the current time in hh:mm:ss:ms format
  now = datetime.now()
  timestamp = now.strftime("%H:%M:%S:%f")[:-3]  # Get milliseconds as well

  text_time = f"Time: {timestamp}"
  cv.putText(frame, text_time, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

  os.makedirs(snapshot_dir, exist_ok=True)  # Create directory if it doesn't exist
  snapshot_path = os.path.join(snapshot_dir, f'snapshot_{timestamp}.png')
  cv.imwrite(snapshot_path, frame)  # Save the frame as an image
  print(f"Snapshot taken and saved at: {snapshot_path}")

def set_velocity_body(vx, vy, vz):
  """ Remember: vz is positive downward!!!
  http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html

  Bitmask to indicate which dimensions should be ignored by the vehicle 
  (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
  none of the setpoint dimensions should be ignored). Mapping: 
  bit 1: x,  bit 2: y,  bit 3: z, 
  bit 4: vx, bit 5: vy, bit 6: vz, 
  bit 7: ax, bit 8: ay, bit 9:


  """
  msg = drone.vehicle.message_factory.set_position_target_local_ned_encode(
          0,
          0, 0,
          mavutil.mavlink.MAV_FRAME_BODY_NED,
          0b0000111111000111, #-- BITMASK -> Consider only the velocities
          0, 0, 0,        #-- POSITION
          vx, vy, vz,     #-- VELOCITY
          0, 0, 0,        #-- ACCELERATIONS
          0, 0)
  drone.vehicle.send_mavlink(msg)
  drone.vehicle.flush()

def traversal():
  drone.arm_and_takeoff(altitude)
  sleep(1)

  if drone.vehicle.parameters['WP_YAW_BEHAVIOR'] != 1:
    drone.vehicle.parameters['WP_YAW_BEHAVIOR'] = 1
    print("Changed the Vehicle's WP_YAW_BEHAVIOR parameter")

  print("Starting journey...")
  with open('coordinate.txt', 'r') as file:
    coordinates = [tuple(map(float, line.split())) for line in file]

    for coord in coordinates:
      drone.goto_gps(coord)
      
      while True:
        x, y = drone.get_gps_coords()
        errorx = abs(round((coord[0] - x) * 10**6, 3))
        errory = abs(round((coord[1] - y) * 10**6, 3))
        sleep(0.5)
        if target_x < 20:
          set_velocity_body(1,0,0)
          sleep(0.5)
        if target_y < 20:
          set_velocity_body(0,1,0)
          sleep(0.5)

        if errorx + errory < 12:
          print("Lock acquired! Sleeping for 5 seconds...")
          sleep(5)
          break
  drone.rtl()

t1 = threading.Thread(target=camera)
t2 = threading.Thread(target=traversal)

t1.start()
t2.start()
t1.join()
t2.join()