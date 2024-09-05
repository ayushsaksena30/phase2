import cv2
import numpy as np
import time

# Function to detect red circles
def detect_red_circles(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    new = cv2.imshow('hsv',hsv)
    # Define range for red color in HSV (two ranges to cover the hue wrap-around) 
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    # Create masks for red color
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = mask1 | mask2  # Combine both masks

    # Find contours from the red mask
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    detected_circles = []

    for contour in contours:
        # Approximate the contour
        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        area = cv2.contourArea(contour)
        
        if len(approx) > 1 and area > 100:  # Heuristic to filter out non-circular shapes
            # Compute the bounding box and radius
            ((x, y), radius) = cv2.minEnclosingCircle(contour)
            if radius > 1:  # Minimum radius to avoid noise
                detected_circles.append((int(x), int(y), int(radius)))

    return detected_circles

# Initialize counts
red_circle_count = 0

# Timestamp of the last detected circle
last_detection_time = time.time()

# Start capturing video
cap = cv2.VideoCapture(0)  # Use '0' for the default camera or '1' for an external camera

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect red circles in the current frame
    circles = detect_red_circles(frame)
    
    current_time = time.time()
    
    # Process the detected circles
    for (x, y, r) in circles:
        # Check if it's time to count a new circle (5 seconds delay)
        if current_time - last_detection_time > 5:
            red_circle_count += 1
            last_detection_time = current_time

        # Draw the circle and center
        cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
        cv2.circle(frame, (x, y), 2, (0, 255, 0), 3)
    
    # Display the resulting frame
    cv2.putText(frame, f"Red Circles Count: {red_circle_count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Red Circle Detection", frame,)
    
    # Break the loop on 'd' key press
    if cv2.waitKey(1) & 0xFF == ord('d'):
        break

# Release video capture object and close windows 
cap.release()
cv2.destroyAllWindows()

# Output the count of red circles 
print(f"Total Red Circles Detected: {red_circle_count}")