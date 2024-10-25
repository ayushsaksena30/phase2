from DroneTerminal import Drone
from time import sleep

drone = Drone(connection_string='127.0.0.1:14550')

drone.arm_and_takeoff(17)

#             #code to move center of frame to centre of object to store coords
#             current_loc = Drone.get_gps_coords()
#             flag_x=flag_y=False
#             if(abs(center_y-center_frame_y)/center_frame_y >0.05):
#                 current_loc[1]+=(3/111320)
#                 Drone.hover_gps(current_loc)
#             else:
#                 flag_y=True
                
            
#             if(abs(center_x-center_frame_x)/center_frame_x >0.05):
#                 current_loc[0]+=(3/111320)
#                 Drone.hover_gps(current_loc)
#             else:
#                 flag_x=True

#             if(flag_x and flag_y):
#                 current_loc = Drone.get_gps_coords()
#                 #code to save coords to json file here
#                 with open('output.json', 'w') as file:
#                     json.dump(current_loc, file)


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