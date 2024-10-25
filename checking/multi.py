import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
from DroneTerminal import Drone
from time import sleep
import threading
from dronekit import connect, VehicleMode, Vehicle, LocationGlobalRelative

drone = Drone(connection_string='127.0.0.1:14550')
model = YOLO("yolov10n.pt")
detected = False
feed_coords = [0,0,0,0]

class MinimalSubscriber(Node):

  def __init__(self):
    print("Initialising...")

    super().__init__('minimal_subscriber')
    self.subscription = self.create_subscription(
        Image,
        '/camera',
        self.captured_frame_callback,
        10)
    self.subscription  # prevent unused variable warning
    self.bridge = CvBridge()
    self.coords = [0,0,0,0]
    print("Initialisation Complete!")

  def captured_frame_callback(self, msg):
    frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imshow("Image", frame)
    cv2.waitKey(1)  # Display the image for 1 millisecond

    # DO YOUR AI WITH THE VARIABLE "frame"...
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the range for red color in HSV
    lower_red_1 = np.array([0, 100, 100])
    upper_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([160, 100, 100])
    upper_red_2 = np.array([180, 255, 255])

    # Create masks for red color
    mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
    mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
    mask = cv2.bitwise_or(mask1, mask2)

    # Bitwise-AND mask and original frame
    red_objects = cv2.bitwise_and(frame, frame, mask=mask)

    # Resize for faster processing (optional)
    red_objects_resized = cv2.resize(red_objects, (640, 480))
    results = model(red_objects_resized)

    # Render results on the frame
    annotated_frame = results[0].plot()

    # Get frame dimensions
    height, width, _ = annotated_frame.shape

    # Draw lines to create quadrants
    cv2.line(annotated_frame, (width // 2, 0), (width // 2, height), (255, 255, 255), 2)  # Vertical line
    cv2.line(annotated_frame, (0, height // 2), (width, height // 2), (255, 255, 255), 2)  # Horizontal line

    # Calculate center of the frame
    center_frame_x = width // 2
    center_frame_y = height // 2

    # Process detected objects
    for result in results[0].boxes:
      # Get the center coordinates of the bounding box
      x1, y1, x2, y2 = result.xyxy[0].tolist()
      center_x = int((x1 + x2) / 2)
      center_y = int((y1 + y2) / 2)

      # Mark the center of the detected object
      cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 255, 0), -1)  # Green circle at the center

      # Draw line from the center of the frame to the center of the detected object
      cv2.line(annotated_frame, (center_frame_x, center_frame_y), (center_x, center_y), (0, 255, 255), 2)  # Yellow line
      if results[0].cls == :
        feed_coords=[center_x, center_y,center_frame_x, center_frame_y]
        detected=True

    # Display the frame with annotations
    cv2.imshow("Webcam Feed - YOLO", annotated_frame)

def camera(args=None):
  while True:
    rclpy.init(args=args)
    cam_feedback = MinimalSubscriber()
    rclpy.spin(cam_feedback)
    cam_feedback.destroy_node()
    rclpy.shutdown()

def hover_gps(coords: tuple):

  # First) Going to GPS at same altitude
  start = drone.get_gps_coords()
  print(start)
  init_dist = drone.distance(start, coords)

  drone.vehicle.simple_goto(LocationGlobalRelative(coords[0], coords[1], 10))

  while True:
    # Call get_gps_coords() to get current coordinates as a tuple
    now = drone.get_gps_coords()  # This line is fixed
    print("Co-ords:", now)
    if drone.distance(now, coords) <= 0.015 * init_dist:
      print("Reached coords:", coords)
      break
    
    if detected:
      moveToObject(feed_coords)


def saveCoord(x,y):
  print("hi I was called")

def moveToObject(feed):
  center_x, center_y,center_frame_x,center_frame_y = feed
  current_loc = Drone.get_gps_coords()
  flag_x=flag_y=False
  if(abs(center_y-center_frame_y)/center_frame_y >0.05):
    current_loc[1]+=(3/111320)
    Drone.hover_gps(current_loc)
  else:
    flag_y=True
  
  if(abs(center_x-center_frame_x)/center_frame_x >0.05):
    current_loc[0]+=(3/111320)
    Drone.hover_gps(current_loc)
  else:
    flag_x=True

  if(flag_x and flag_y):
    saveCoord(center_x,center_y)

  detected=False
  feed_coords=[0,0,0,0]

def traversal():
  drone.arm_and_takeoff(17)
  with open('hotspot_coords.txt', 'r') as file:

    for line in file:
      data_list = [float(x.strip()) for x in line.split()]
      # Convert the list to a tuple
      coord = tuple(data_list)
      
      hover_gps(coord)
      sleep(0.02)
      print("Reached location : ", coord)
      sleep(1.5)

t1 = threading.Thread(target=camera)
t2 = threading.Thread(target=traversal)

t1.start()
t2.start()
t1.join()
t2.join()