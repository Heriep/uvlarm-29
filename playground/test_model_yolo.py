import cv2
import torch
from ultralytics import YOLO
from IPython.display import Image, display
import pathlib

pathlib.WindowsPath = pathlib.PosixPath

# Chemin absolu vers le modèle et yolov5
model_path = '/home/o2p9/ros_space/playground/train/ros_project_long/weights/best.pt'
yolov5_path = '/home/o2p9/yolov5'

# Chargez le modèle avec torch.hub.load
model = torch.hub.load(yolov5_path, 'custom', path=model_path, source='local', force_reload=True)

# Set webcam input
cam = cv2.VideoCapture(0)

while True:
    # Read frames
    ret, img = cam.read()

    # Perform object detection
    results = model(img)
    # Récupérer les boîtes de détection, les scores et les classes
    boxes = results.xywh[0]  # coordonnées des boîtes englobantes [x_center, y_center, width, height, confidence, class]
    classes = results.names  # Les classes d'objets détectées
    confidences = boxes[:, 4].tolist()  # Liste des scores de confiance
    labels = boxes[:, 5].tolist()  # Liste des indices des classes détectées
    coordinates = boxes[:, :4].tolist() 
    print(coordinates)
    # Display predictions
    #results.show()
    
    # Press 'q' or 'Esc' to quit
    if (cv2.waitKey(1) & 0xFF == ord("q")) or (cv2.waitKey(1)==27):
        break

# Close the camera
cam.release()
cv2.destroyAllWindows()