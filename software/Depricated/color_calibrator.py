# HSV Color Calibration Tool for Puck & Mallet Detection
# Allows fine-tuning of HSV ranges with live preview
# Run this, adjust sliders until only the target object is detected, then copy the values

import cv2
import numpy as np

# -------------------- CONFIGURATION --------------------
CAM_INDEX = 0  # Same as tracker
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# Current values from tracker (starting point)
PUCK_HSV_LOW = [40, 50, 50]      # Green puck
PUCK_HSV_HIGH = [80, 255, 255]
MALLET_HSV_LOW = [5, 100, 100]   # Orange mallet  
MALLET_HSV_HIGH = [25, 255, 255]

# -------------------- GLOBALS FOR TRACKBARS --------------------
# Puck HSV ranges
puck_h_low, puck_h_high = PUCK_HSV_LOW[0], PUCK_HSV_HIGH[0]
puck_s_low, puck_s_high = PUCK_HSV_LOW[1], PUCK_HSV_HIGH[1]
puck_v_low, puck_v_high = PUCK_HSV_LOW[2], PUCK_HSV_HIGH[2]

# Mallet HSV ranges
mallet_h_low, mallet_h_high = MALLET_HSV_LOW[0], MALLET_HSV_HIGH[0]
mallet_s_low, mallet_s_high = MALLET_HSV_LOW[1], MALLET_HSV_HIGH[1]
mallet_v_low, mallet_v_high = MALLET_HSV_LOW[2], MALLET_HSV_HIGH[2]

# Current calibration target
calibrating = "puck"  # "puck" or "mallet"

def nothing(x):
    pass

def create_trackbars():
    """Create windows with trackbars for HSV adjustment"""
    
    # Puck calibration window
    cv2.namedWindow("Puck Calibration")
    cv2.createTrackbar("H Low", "Puck Calibration", puck_h_low, 179, nothing)
    cv2.createTrackbar("H High", "Puck Calibration", puck_h_high, 179, nothing)
    cv2.createTrackbar("S Low", "Puck Calibration", puck_s_low, 255, nothing)
    cv2.createTrackbar("S High", "Puck Calibration", puck_s_high, 255, nothing)
    cv2.createTrackbar("V Low", "Puck Calibration", puck_v_low, 255, nothing)
    cv2.createTrackbar("V High", "Puck Calibration", puck_v_high, 255, nothing)
    
    # Mallet calibration window
    cv2.namedWindow("Mallet Calibration")
    cv2.createTrackbar("H Low", "Mallet Calibration", mallet_h_low, 179, nothing)
    cv2.createTrackbar("H High", "Mallet Calibration", mallet_h_high, 179, nothing)
    cv2.createTrackbar("S Low", "Mallet Calibration", mallet_s_low, 255, nothing)
    cv2.createTrackbar("S High", "Mallet Calibration", mallet_s_high, 255, nothing)
    cv2.createTrackbar("V Low", "Mallet Calibration", mallet_v_low, 255, nothing)
    cv2.createTrackbar("V High", "Mallet Calibration", mallet_v_high, 255, nothing)

def get_trackbar_values():
    """Read current trackbar values"""
    puck_low = np.array([
        cv2.getTrackbarPos("H Low", "Puck Calibration"),
        cv2.getTrackbarPos("S Low", "Puck Calibration"),
        cv2.getTrackbarPos("V Low", "Puck Calibration")
    ], dtype=np.uint8)
    
    puck_high = np.array([
        cv2.getTrackbarPos("H High", "Puck Calibration"),
        cv2.getTrackbarPos("S High", "Puck Calibration"),
        cv2.getTrackbarPos("V High", "Puck Calibration")
    ], dtype=np.uint8)
    
    mallet_low = np.array([
        cv2.getTrackbarPos("H Low", "Mallet Calibration"),
        cv2.getTrackbarPos("S Low", "Mallet Calibration"),
        cv2.getTrackbarPos("V Low", "Mallet Calibration")
    ], dtype=np.uint8)
    
    mallet_high = np.array([
        cv2.getTrackbarPos("H High", "Mallet Calibration"),
        cv2.getTrackbarPos("S High", "Mallet Calibration"),
        cv2.getTrackbarPos("V High", "Mallet Calibration")
    ], dtype=np.uint8)
    
    return puck_low, puck_high, mallet_low, mallet_high

def print_values(puck_low, puck_high, mallet_low, mallet_high):
    """Print current values in copy-paste format"""
    print("\n" + "=" * 60)
    print("CALIBRATED HSV VALUES - Copy these to puck_mallet_tracker_fast.py")
    print("=" * 60)
    print(f"\n# Puck (Green) HSV Range")
    print(f"PUCK_HSV_LOW = np.array([{puck_low[0]}, {puck_low[1]}, {puck_low[2]}], dtype=np.uint8)")
    print(f"PUCK_HSV_HIGH = np.array([{puck_high[0]}, {puck_high[1]}, {puck_high[2]}], dtype=np.uint8)")
    print(f"\n# Mallet (Orange) HSV Range")
    print(f"MALLET_HSV_LOW = np.array([{mallet_low[0]}, {mallet_low[1]}, {mallet_low[2]}], dtype=np.uint8)")
    print(f"MALLET_HSV_HIGH = np.array([{mallet_high[0]}, {mallet_high[1]}, {mallet_high[2]}], dtype=np.uint8)")
    print("\n" + "=" * 60)

def main():
    print("=" * 60)
    print("HSV Color Calibration Tool")
    print("=" * 60)
    print("\nInstructions:")
    print("1. Adjust sliders until ONLY the target object is white in the mask")
    print("2. Start with H (Hue) to get the right color")
    print("3. Then adjust S (Saturation) to filter out gray/white objects")
    print("4. Finally adjust V (Value/Brightness) to filter lighting variations")
    print("\nTips:")
    print("- Narrower ranges = more specific detection (fewer false positives)")
    print("- Make sure the object is still detected when moved around")
    print("\nControls:")
    print("  P - Print current values to console")
    print("  S - Save screenshot")
    print("  Q - Quit")
    print("=" * 60)
    
    # Open camera
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 60)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    ret, frame = cap.read()
    if not ret:
        print("Error: Cannot read from camera")
        return
    
    h, w = frame.shape[:2]
    print(f"\nCamera: {w}x{h}")
    
    # Create trackbar windows
    create_trackbars()
    
    # Create display windows
    cv2.namedWindow("Original + Detection")
    cv2.namedWindow("Puck Mask")
    cv2.namedWindow("Mallet Mask")
    cv2.namedWindow("HSV View")
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Convert to HSV
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        
        # Get current trackbar values
        puck_low, puck_high, mallet_low, mallet_high = get_trackbar_values()
        
        # Create masks
        puck_mask = cv2.inRange(hsv, puck_low, puck_high)
        puck_mask = cv2.morphologyEx(puck_mask, cv2.MORPH_OPEN, kernel)
        puck_mask = cv2.morphologyEx(puck_mask, cv2.MORPH_CLOSE, kernel)
        
        mallet_mask = cv2.inRange(hsv, mallet_low, mallet_high)
        mallet_mask = cv2.morphologyEx(mallet_mask, cv2.MORPH_OPEN, kernel)
        mallet_mask = cv2.morphologyEx(mallet_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours and draw detections
        vis = frame.copy()
        
        # Puck detection (green circles)
        puck_contours, _ = cv2.findContours(puck_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        puck_count = 0
        for cnt in puck_contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                if 5 < radius < 100:
                    cv2.circle(vis, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                    cv2.putText(vis, f"PUCK", (int(x)+10, int(y)-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    puck_count += 1
        
        # Mallet detection (orange circles)
        mallet_contours, _ = cv2.findContours(mallet_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        mallet_count = 0
        for cnt in mallet_contours:
            area = cv2.contourArea(cnt)
            if area > 100:
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                if 5 < radius < 100:
                    cv2.circle(vis, (int(x), int(y)), int(radius), (0, 165, 255), 2)
                    cv2.putText(vis, f"MALLET", (int(x)+10, int(y)-10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)
                    mallet_count += 1
        
        # Draw info
        cv2.putText(vis, f"Puck detections: {puck_count} | Mallet detections: {mallet_count}", 
                   (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(vis, f"Puck: H[{puck_low[0]}-{puck_high[0]}] S[{puck_low[1]}-{puck_high[1]}] V[{puck_low[2]}-{puck_high[2]}]",
                   (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(vis, f"Mallet: H[{mallet_low[0]}-{mallet_high[0]}] S[{mallet_low[1]}-{mallet_high[1]}] V[{mallet_low[2]}-{mallet_high[2]}]",
                   (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1)
        cv2.putText(vis, "P=Print values | S=Screenshot | Q=Quit", (10, h-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Create HSV visualization (show what HSV looks like)
        hsv_vis = hsv.copy()
        # Show H, S, V channels side by side
        h_channel = cv2.cvtColor(hsv[:, :, 0], cv2.COLOR_GRAY2BGR)
        s_channel = cv2.cvtColor(hsv[:, :, 1], cv2.COLOR_GRAY2BGR)
        v_channel = cv2.cvtColor(hsv[:, :, 2], cv2.COLOR_GRAY2BGR)
        
        # Scale down for side-by-side view
        scale = 0.33
        h_small = cv2.resize(h_channel, (int(w*scale), int(h*scale)))
        s_small = cv2.resize(s_channel, (int(w*scale), int(h*scale)))
        v_small = cv2.resize(v_channel, (int(w*scale), int(h*scale)))
        
        hsv_combined = np.hstack([h_small, s_small, v_small])
        cv2.putText(hsv_combined, "H (Hue)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(hsv_combined, "S (Sat)", (int(w*scale)+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(hsv_combined, "V (Val)", (int(w*scale*2)+10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Color the mask windows
        puck_mask_color = cv2.cvtColor(puck_mask, cv2.COLOR_GRAY2BGR)
        puck_mask_color[:, :, 0] = 0  # Remove blue
        puck_mask_color[:, :, 2] = 0  # Remove red (keep green)
        
        mallet_mask_color = cv2.cvtColor(mallet_mask, cv2.COLOR_GRAY2BGR)
        mallet_mask_color[:, :, 0] = mallet_mask // 2  # Some blue
        mallet_mask_color[:, :, 1] = mallet_mask // 2  # Less green
        # Red stays full (orange tint)
        
        # Show windows
        cv2.imshow("Original + Detection", vis)
        cv2.imshow("Puck Mask", puck_mask_color)
        cv2.imshow("Mallet Mask", mallet_mask_color)
        cv2.imshow("HSV View", hsv_combined)
        
        # Handle keys
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == 27:
            print_values(puck_low, puck_high, mallet_low, mallet_high)
            break
        elif key == ord('p'):
            print_values(puck_low, puck_high, mallet_low, mallet_high)
        elif key == ord('s'):
            import time
            filename = f"calibration_{int(time.time())}.png"
            cv2.imwrite(filename, vis)
            print(f"Screenshot saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
