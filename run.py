import cv2
from ultralytics import YOLO

# Load the YOLOv8n model (replace 'your_model_path.pt' with the path to your trained model)
model = YOLO('/Users/ayushsaksena/Desktop/phase2/yolov10n.pt')  # You can also load 'yolov8n.pt' for a pre-trained model

# Initialize webcam
cap = cv2.VideoCapture(1)  # Use 0 for the default camera

# Check if the webcam is opened correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

        # Run YOLO model on the current frame
    results = model(frame)

    # Draw the results on the frame
    annotated_frame = results[0].plot()  # results[0] contains the result for the first image/frame

    # Display the resulting frame
    cv2.imshow('YOLOv10n Webcam', annotated_frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the webcam and close windows
cap.release()
cv2.destroyAllWindows()