import cv2
import numpy as np
import time

net = cv2.dnn.readNet("/Users/ayushsaksena/Desktop/phase2/yolo_v4_tiny_optimizations/Yolo v4-tiny/yolov4-tiny.weights", "/Users/ayushsaksena/Desktop/phase2/yolo_v4_tiny_optimizations/Yolo v4-tiny/yolov4-tiny.cfg")

with open("/Users/ayushsaksena/Desktop/phase2/yolo_v4_tiny_optimizations/Yolo v4-tiny/coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# Set backend and target to use with Jetson Nano for better performance (if necessary)
# net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
# net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)

# Get the output layer names from YOLO
layer_names = net.getLayerNames()
output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers()]

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    
    if not ret:
        break

    height, width, channels = frame.shape

    blob = cv2.dnn.blobFromImage(frame, 1/255.0, (416, 416), (0, 0, 0), swapRB=True, crop=False)
    net.setInput(blob)
    
    start_time = time.time()
    
    detections = net.forward(output_layers)
    
    end_time = time.time()
    
    latency = (end_time - start_time) * 1000
    print(f"Latency: {latency:.2f} ms")
    
    class_ids = []
    confidences = []
    boxes = []

    for output in detections:
        for detection in output:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            
            if confidence > 0.5:
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    # Apply Non-Maximum Suppression to filter the boxes
    indices = cv2.dnn.NMSBoxes(boxes, confidences, score_threshold=0.5, nms_threshold=0.4)

    if len(indices) > 0:
        for i in indices.flatten():
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            color = (0, 255, 0)

            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, f"{label} {confidence:.2f}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    cv2.imshow("YOLOv4-Tiny Object Detection", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()