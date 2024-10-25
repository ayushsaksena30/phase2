#!/usr/bin/env python3

from dronekit import connect, VehicleMode, Vehicle, LocationGlobalRelative
from time import sleep
from math import sqrt

class Drone:
    '''Creates a terminal to interact with the drone connected (tries connecting to the 'connection_string' parameter)'''
    
    def __init__(self, connection_string = "/dev/ttyACM0") -> None:
        
        # sitl = dronekit_sitl.start_default()

        # connection_string = "127.0.0.1:14550"
        # connection_string = "/dev/ttyUSB0"
        self.connection_string = connection_string
        self.OVERALL_TOLERANCE = 0.9
        print(f"\nAttempting to connect on {connection_string}...")
        self.vehicle = connect(self.connection_string, wait_ready=True)
        print("Connection successful!")

    def get_vehicle(self): return self.vehicle


    # ----------------------
    def get_gps_coords(self) -> tuple:
        '''Returns the Lattitude and Longitude of the vehicle e.g. -> (23.1781203, 80.0227357)'''
        return (
            self.vehicle.location.global_relative_frame.lat, 
            self.vehicle.location.global_relative_frame.lon
        )
    
    def get_altitude(self) -> float: return self.vehicle.location.global_relative_frame.alt
    def get_vehicle_mode_name(self): return self.vehicle.mode.name
    
    def get_vitals(self) -> float:
        '''Returns the following in sequence:
        last_heartbeat
        battery.level
        '''
        return (
            self.vehicle.last_heartbeat,
            self.vehicle.battery.level
        )

    #-----------------------


    def arm_and_takeoff(self, targetAltitude):
        """
        Arms vehicle and flies to targetAltitude.
        """

        print("Basic pre-arm checks")
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            sleep(1)

        print("Arming motors")
        self.vehicle.mode    = VehicleMode("GUIDED")    # Copter should arm in GUIDED mode
        self.vehicle.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print("Waiting for arming...")
            sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(targetAltitude) # Take off to target altitude

        while True:
            print(f"\r{self.vehicle.location.global_relative_frame.alt}", end="", flush=True)
            
            if self.vehicle.location.global_relative_frame.alt>=targetAltitude*0.9:
                print("\nReached target altitude")
                break
            sleep(0.5)

    def goto_gps(self, coords : tuple):
        self.vehicle.simple_goto(LocationGlobalRelative(
            coords[0],
            coords[1],
            self.vehicle.location.global_relative_frame.alt,
        ))
###############################################################################################################
#my addition
    def distance(self, p1: tuple, p2: tuple):
        """
        This function calculates the Euclidean distance between two points in n-dimensional space.

        Args:
            p1: A tuple representing the first point.
            p2: A tuple representing the second point.

        Returns:
            The Euclidean distance between the two points.
        """
        # if len(p1) != len(p2):
        #     raise ValueError("Tuples must have the same length")
        return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

    def hover_gps(self, coords: tuple):

        # First) Going to GPS at same altitude
        start = self.get_gps_coords()
        print(start)
        init_dist = self.distance(start, coords)

        self.vehicle.simple_goto(LocationGlobalRelative(coords[0], coords[1], 10))

        while True:
            # Call get_gps_coords() to get current coordinates as a tuple
            now = self.get_gps_coords()  # This line is fixed
            print("Co-ords:", now)
            if self.distance(now, coords) <= 0.015 * init_dist:
                print("Reached coords:", coords)
                break
            sleep(2)

        # #second) lowering altitude
        # self.vehicle.simple_goto(LocationGlobalRelative(coords[0], coords[1], 5))
        # while True:
        #     print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
        #     #Break and return from function just below target altitude.
        #     if self.vehicle.location.global_relative_frame.alt<=5*1.15:
        #         print("Reached target altitude: 5m")
        #         break
        #     sleep(2)

        # #third) raising altitude
        # self.vehicle.simple_goto(LocationGlobalRelative(coords[0], coords[1], 10))
        # while True:
        #     print(" Altitude: ", self.vehicle.location.global_relative_frame.alt)
        #     #Break and return from function just below target altitude.
        #     if self.vehicle.location.global_relative_frame.alt>=10*0.95:
        #         print("Reached target altitude: 10m")
        #         break
        #     sleep(2)

###############################################################################################################
    
    def rtl(self):
        self.vehicle.mode = VehicleMode("RTL")


    #-----------------------
    def land_and_disarm(self):
        print("Intiating Landing sequence...")
        self.vehicle.mode = VehicleMode("LAND")

    def close_vehicle(self):
        self.vehicle.close()