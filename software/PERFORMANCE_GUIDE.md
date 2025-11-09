# Puck Tracker Performance Optimization Guide

## Current Optimizations (40+ FPS achieved)

### 1. Resolution Change
- **Changed from**: 1280x720 (720p)
- **Changed to**: 640x480 (480p)
- **Impact**: 3.75x fewer pixels to process (~75% reduction)

### 2. Kernel Size Reduction
- **Gaussian Blur**: 5x5 → 3x3
- **Morphological Operations**: 5x5 → 3x3
- **Impact**: Faster convolution operations

### 3. Optional Line Suppression
- Set `SKIP_LINE_SUPPRESSION = True` to disable table line detection
- **Impact**: Skips HSV filtering and bitwise operations
- **Trade-off**: May pick up colored table markings as false positives

### 4. Frame Skip for Visualization
- `VISUAL_UPDATE_SKIP` parameter (default: 0)
- Set to 1 to render every other frame, 2 for every third frame, etc.
- **Note**: Tracking still runs at full speed, only visualization is skipped

### 5. Camera Settings
- Buffer size set to 1 (reduces lag)
- Manual exposure at -6 (reduces motion blur but darker image)
- Auto-gain disabled

## New Features Added

### Trajectory Prediction
- **Physics-based simulation** with bounce detection
- Configurable prediction time (default: 2 seconds)
- Adjustable damping factor (default: 0.95 = 5% energy loss per bounce)
- Real-time display of predicted path

### Table Boundaries
- Separate from ROI tracking boundaries
- Orange rectangle shows trajectory bounds
- Independent adjustment via GUI sliders
- Used for bounce physics in trajectory prediction

### Visual Elements
- **Blue box**: ROI (where tracking happens)
- **Orange box**: Table bounds (for trajectory physics)
- **Green circle**: Current puck position
- **Cyan arrow**: Velocity vector
- **Magenta line**: Predicted path with bounces
- **Magenta circle**: Predicted endpoint

## Further Optimization Options

### If you need MORE speed (60+ FPS):

1. **Increase VISUAL_UPDATE_SKIP**:
   ```python
   VISUAL_UPDATE_SKIP = 1  # Render every other frame
   ```

2. **Disable mask window**:
   - Uncheck "Show Mask Window" in GUI

3. **Reduce morphological operations**:
   ```python
   MORPH_KERNEL_SIZE = 2  # Even smaller
   ```

4. **Simplify trajectory prediction**:
   ```python
   TRAJECTORY_PREDICTION_TIME = 1.0  # Shorter prediction
   ```

5. **Disable trajectory visualization**:
   - Uncheck "Show Trajectory Prediction" in GUI

### If tracking quality is poor:

1. **Enable line suppression**:
   ```python
   SKIP_LINE_SUPPRESSION = False
   ```

2. **Increase kernel sizes**:
   ```python
   MORPH_KERNEL_SIZE = 5
   ```

3. **Use 3x3 Gaussian blur**:
   - Already optimized

4. **Adjust HSV color range**:
   - Use GUI sliders for fine-tuning
   - Try different color presets

## Camera-Specific Tips

### ELP USB Camera (H120)
- Supports up to 120 FPS at lower resolutions
- Good low-light performance
- May need exposure adjustment: `-3` to `-9`

### Adjusting Exposure
If image is too dark, edit line ~354:
```python
cap.set(cv2.CAP_PROP_EXPOSURE, -3)  # Brighter (more blur)
# or
cap.set(cv2.CAP_PROP_EXPOSURE, -9)  # Darker (less blur)
```

## Monitoring Performance

Watch the FPS counter in both:
1. Video window (top-left)
2. GUI "Current Tracking Results" panel

Target: 60-90 FPS for optimal tracking of fast-moving pucks.

## Troubleshooting

### Low FPS (<30)
- Check if another program is using the camera
- Reduce resolution further (320x240)
- Close mask window
- Increase VISUAL_UPDATE_SKIP

### Tracking loses puck
- Decrease VISUAL_UPDATE_SKIP to 0
- Enable line suppression
- Adjust color HSV range
- Increase ROI size

### Trajectory looks wrong
- Adjust table bounds to match actual table
- Tune damping factor (0.90-0.98 typical)
- Check that velocity is being measured correctly

## Configuration Quick Reference

```python
# In puck_tracker_advanced.py, top section:

CAM_INDEX = 1                    # 0=laptop, 1=USB camera
FRAME_WIDTH = 640                # 640x480 for speed
FRAME_HEIGHT = 480
TARGET_FPS = 90

SKIP_LINE_SUPPRESSION = True     # True=faster, False=better
MORPH_KERNEL_SIZE = 3            # 2-5 typical
VISUAL_UPDATE_SKIP = 0           # 0=every frame, 1=every other

TRAJECTORY_PREDICTION_TIME = 2.0 # Seconds ahead
TRAJECTORY_DAMPING = 0.95        # Bounce energy retention
```
