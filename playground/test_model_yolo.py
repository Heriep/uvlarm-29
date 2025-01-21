import cv2
import torch
from ultralytics import YOLO
from IPython.display import Image, display

# Load the model
path = "/train/ros_project_long/weights/best.pt"
#model = YOLO(path, 'yolov5s')
#model = torch.hub.load('.', 'custom', path='C:/Users/pierr/yolov5/runs/train/yolo_road_det/weights/best.pt', source='local')
model = torch.hub.load('/train/yolo_road_det', 'yolov5s')

# Set webcam input
cam = cv2.VideoCapture(0)

while True:
    # Read frames
    ret, img = cam.read()

    # Perform object detection
    results = model(img)

    # Display predictions
    results.show()
    
    # Press 'q' or 'Esc' to quit
    if (cv2.waitKey(1) & 0xFF == ord("q")) or (cv2.waitKey(1)==27):
        break

# Close the camera
cam.release()
cv2.destroyAllWindows()