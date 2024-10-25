#using geofence coords and real-time gps to create a new cartasian system.
#Drone has to do its traversal without going to the hotspot.
#once a shape is detected, we use __ to calculate its coords and then convert it into our cartasian system

from merge_codes.DroneTerminal import Drone
import rclpy
from siddharth import calculate_distance
from time import sleep
from dronekit import connect, VehicleMode, Vehicle, LocationGlobalRelative
drone = Drone(connection_string='127.0.0.1:14550')

def traversal():
  drone.arm_and_takeoff(17)
  with open('hotspot_coords.txt', 'r') as file:
    for line in file:
      data_list = [float(x.strip()) for x in line.split()]
      coords = tuple(data_list)

      start = drone.get_gps_coords()
      print(start)
      init_dist = drone.distance(start, coords)

      drone.vehicle.simple_goto(LocationGlobalRelative(coords[0], coords[1], 10))

      while True:
          now = drone.get_gps_coords()
          print("Co-ords:", now)
          if drone.distance(now, coords) <= 0.015 * init_dist:
            print("Reached coords:", coords)
            break
          sleep(2)

          if foundCircle():
            x,y=0
            calculate_distance(x,y)