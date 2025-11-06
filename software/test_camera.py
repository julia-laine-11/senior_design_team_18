"""
Quick camera test script
Tests different camera indices to find available cameras
"""

import cv2

print("Testing cameras...")
print("=" * 50)

for i in range(5):
    print(f"\nTesting Camera {i}...")
    cap = cv2.VideoCapture(i)
    
    if cap.isOpened():
        # Try to read a frame
        ret, frame = cap.read()
        if ret:
            h, w = frame.shape[:2]
            # Try to get FPS
            fps = cap.get(cv2.CAP_PROP_FPS)
            print(f"  ✓ Camera {i} WORKS!")
            print(f"    Resolution: {w}x{h}")
            print(f"    FPS: {fps}")
            
            # Try to set 480p and 90fps
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_FPS, 90)
            
            # Check what we actually got
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = cap.get(cv2.CAP_PROP_FPS)
            
            print(f"    After setting 640x480@90fps:")
            print(f"      Got: {actual_w}x{actual_h}@{actual_fps}fps")
            
            # Show a frame
            cv2.imshow(f"Camera {i} Test", frame)
            print(f"    Press any key to continue...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print(f"  ✗ Camera {i} opened but can't read frames")
    else:
        print(f"  ✗ Camera {i} not available")
    
    cap.release()

print("\n" + "=" * 50)
print("Camera test complete!")
print("\nUpdate CAM_INDEX in puck_tracker_advanced.py to use")
print("the camera that worked for you.")
