from merge_codes.DroneTerminal import Drone
from time import sleep

drone = Drone(connection_string='127.0.0.1:14550')
target = False #Checks if target is found and is updated by yolo thread
first_time = True #makes sure the drone is going for the first time

x,y=0 #center of Target (to be updated by yolo thread)
frame_x,frame_y=0 #center of frame (to be updated)

def main():
  drone.arm_and_takeoff(10)

  with open('hotspot_coords.txt', 'r') as file:
    for line in file:
      data_list = [float(x.strip()) for x in line.split()]
      # Convert the list to a tuple
      coord = tuple(data_list)
      
      hover_gps(coord)
      sleep(0.02)
      print("Reached location : ", coord)
      sleep(1.5)

  print("RTL!")
  drone.rtl()
  drone.close_vehicle()

def hover_gps(coord):
  global target,x,y,frame_x,frame_y
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

    if(target and first_time):
      go_back=drone.get_gps_coords()

      drone.vehicle.mode = VehicleMode("LOITER") #supposedly, it stops drone
      
      while drone.vehicle.groundspeed != 0:
        sleep(2)

      lat,lon = drone.get_gps_coords()

      while center_y-y<10:
        lat+=1/111320
        drone.hover_gps((lat,lon))

      while center_x-x<10:
        lon+=1/111320
        drone.hover_gps((lat,lon))

      #code to drop payload
      first_time = False
      drone.hover_gps(go_back)

    sleep(2)