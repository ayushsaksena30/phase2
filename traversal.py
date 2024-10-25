from DroneTerminal import Drone
from time import sleep

drone = Drone(connection_string='127.0.0.1:14550')

drone.arm_and_takeoff(10)

with open('hotspot_coords.txt', 'r') as file:
    for line in file:
        data_list = [float(x.strip()) for x in line.split()]
        # Convert the list to a tuple
        coord = tuple(data_list)
        
        drone.hover_gps(coord)
        sleep(0.02)
        print("Reached location : ", coord)
        sleep(1.5)

print("RTL!")
drone.rtl()
drone.close_vehicle()