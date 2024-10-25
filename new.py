while True:
                x, y = drone.get_gps_coords()
                errorx = abs(round((coord[0] - x) * 10**6, 3))
                errory = abs(round((coord[1] - y) * 10**6, 3))
                if target_x is not None and target_y is not None:
                #     if drone.vehicle.parameters['WP_YAW_BEHAVIOR']== 0:
                #         drone.vehicle.parameters['WP_YAW_BEHAVIOR'] = 1
                #         print("Changed the Vehicle's WP_YAW_BEHAVIOR parameter")
                    # Calculate offsets
                    offset_x = (target_x - frame_width // 2)
                    offset_y = (target_y - frame_height // 2)

                    # Check if the offsets are within the tolerance
                    if abs(offset_x) > TOLERANCE or abs(offset_y) > TOLERANCE:
                        # Get current drone location
                        current_lat = drone.vehicle.location.global_relative_frame.lat
                        current_lon = drone.vehicle.location.global_relative_frame.lon

                        # Convert pixel offsets to GPS coordinates
                        # Adjust only the latitude and longitude, leaving yaw unchanged
                        new_lat = current_lat + (offset_y / frame_height) * 0.000001  # Adjust for vertical plane
                        new_lon = current_lon + (offset_x / frame_width) * 0.000001  # Adjust for horizontal plane
                        
                        # Command the drone to go to the new GPS coordinates
                        drone.goto_gps((new_lat, new_lon))
                        sleep(0.2)  # Allow time for movement
                    else:
                        # If within tolerance, you might want to hover or do nothing
                        print("Drone centered on target.")
                sleep(0.5)