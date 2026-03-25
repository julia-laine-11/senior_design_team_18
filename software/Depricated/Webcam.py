#Required Libraries
#pip install ultralytics opencv-python
#For CUDA enabled GPU (NVIDIA)
#pip install torch torchvision --extra-index-url https://download.pytorch.org/whl/cu118


import cv2
from ultralytics import YOLO

def main():
    # Load YOLO model (YOLOv8n is fastest, YOLOv8s/m/l are more accurate)
    model = YOLO("yolov8n.pt")


    # Open webcam
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


    if not cap.isOpened():
        print("Error: Cannot access webcam")
        return
    
    ret, frame = cap.read()
    frame = cv2.resize(frame, (480, 360)) 

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Perform detection + tracking
        results = model.track(source=frame, persist=True, stream=True)

        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue

            for box in boxes:
                # Coordinates
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                # Class label
                cls = int(box.cls[0])
                if (model.names[cls] != 'cell phone'):
                    label = model.names[cls]

                    # Confidence
                    conf = float(box.conf[0])

                    # Track ID (may be None if tracking fails)
                    track_id = int(box.id[0]) if box.id is not None else -1

                    # Draw bounding box
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Draw label and tracking ID
                    cv2.putText(frame, f"{label} ID:{track_id} {conf:.2f}",
                                (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (0, 255, 0), 2)

        # Show the frame
        cv2.imshow("YOLOv8 Object Tracking", frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
