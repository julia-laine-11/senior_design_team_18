# Defensive Mode Software Report

This README documents `defensive_mode.py`, the main autonomous defense program for the air hockey table. The program combines camera vision, Kalman-filtered puck and mallet tracking, GUI calibration controls, persistent runtime settings, goal-protection logic, and CoreXY motor output.

## Purpose

`defensive_mode.py` runs the defensive side of the project. It watches the camera feed, finds the puck and mallet by color, estimates their filtered positions and puck velocity, predicts whether the puck will cross the defended goal, and commands the CoreXY gantry to block the goal while respecting the project's safety boundaries.

The program has two operating modes:

- Manual mode: the user drives the mallet with keyboard input.
- Game mode: the program autonomously guards the goal.

The program always keeps motor safety active. If the mallet is not detected, motors stop. If the mallet approaches the restricted red-zone boundary, `CoreXYController` blocks motion farther into that zone.

## Main Files

- `defensive_mode.py`: main application, camera loop, GUI, vision tracking, Kalman filtering, trajectory prediction, and defense decision logic.
- `corexy_controller.py`: converts desired camera-space movement into CoreXY motor commands and enforces red-zone/boundary safety.
- `defensive_mode_config.json`: runtime calibration file saved by the GUI and loaded on startup.
- `defense_controller.py`: older/simple dual-motor controller. It is not the controller used by the current defensive mode loop.

## Runtime Configuration

The defensive mode program stores adjustable settings in:

```text
software/defensive_mode_config.json
```

On startup, `GameState` loads this JSON file. If the file does not exist or cannot be parsed, the program falls back to constants in `defensive_mode.py`.

When a GUI slider or display checkbox changes, the new setting is saved back to the JSON file automatically. This means calibration values from the last run are restored the next time the program starts.

The JSON stores:

- table ROI margins
- table rounded-corner radius
- mallet box margins
- red-zone margins
- puck HSV low/high values
- mallet HSV low/high values
- max motor speed
- home position
- goal position and length
- display options for masks, bounds, and trajectory

`game_enabled` is intentionally not saved. The program starts with autonomous game mode off so the robot does not immediately move on launch.

## Coordinate System

The camera frame is treated as a 2D pixel plane:

- `x = 0` is the left edge of the frame.
- `y = 0` is the top edge of the frame.
- The defended goal is a vertical line on the left side of the table.
- The puck attacks from right to left.

Most calibration settings are stored as margins from the frame edge instead of direct rectangle corners. For a 640x480 camera frame:

```text
left   = margin from left edge
right  = frame_width - right_margin
top    = margin from top edge
bottom = frame_height - bottom_margin
```

For example, a table ROI of `(21,105)-(568,422)` is stored as:

```json
"table_roi": {
  "top": 105,
  "bottom": 58,
  "left": 21,
  "right": 72
}
```

because `480 - 422 = 58` and `640 - 568 = 72`.

## Startup Flow

When `main()` runs:

1. It prints the motor, camera, config path, and Kalman status.
2. It creates a `GameState`.
3. `GameState` loads `defensive_mode_config.json`.
4. A tracking thread starts.
5. The Tkinter GUI opens in the main thread.
6. The tracking thread opens the camera and initializes the motor controller.
7. The camera loop runs until the GUI closes or the user presses `Q`/`ESC`.

The GUI runs in the main thread because Tkinter requires that. The camera and motor loop run in a background thread.

## Camera And Vision Pipeline

Each camera frame goes through this process:

1. Read a frame from OpenCV.
2. Resize the frame by `PROCESSING_SCALE` for faster processing.
3. Apply a small Gaussian blur to reduce noise.
4. Convert BGR image data to HSV.
5. Threshold the HSV image separately for puck and mallet colors.
6. Mask both threshold images to the table ROI.
7. Clean each mask with morphological open/close operations.
8. Find contours in the puck mask and mallet mask.
9. Pick the best circular contour within the configured radius limits.

The puck is expected to be green by default. The mallet is expected to be orange by default. These colors can be adjusted from the Vision tab.

## Circle Detection

`find_circle()` receives contours and radius limits. It scores candidate contours based on circularity:

```text
circularity = 4*pi*area / perimeter^2
```

If a Kalman prediction is available, the function slightly favors contours near the predicted position. This helps reject noise and keeps tracking stable when multiple similarly colored blobs appear.

A contour is ignored if:

- its area is too small
- its enclosing radius is below the minimum
- its enclosing radius is above the maximum
- its perimeter is invalid

## Kalman Filtering

Kalman filtering is used for both the puck and the mallet.

The `KalmanTracker` state is:

```text
[x, y, vx, vy]
```

where:

- `x`, `y` are filtered position in pixels
- `vx`, `vy` are estimated velocity in pixels per second

The measurement is:

```text
[x, y]
```

The filter uses a constant-velocity model. Every frame:

1. `predict()` estimates the next position and velocity.
2. The predicted position is used as a search hint for circle detection.
3. If a valid circle is detected, `correct()` updates the filter with the measured position.
4. If the object has been lost too long and then reappears, the Kalman filter is reset at the new position.

The puck velocity from the Kalman filter drives trajectory prediction. This is important because raw frame-to-frame position changes are noisy and can cause unstable intercept decisions.

The program highlights Kalman filtering in three places:

- startup console output
- GUI Tracking status
- OpenCV camera overlay

## GUI Tabs

The Tkinter GUI provides calibration and status controls.

### Game Tab

Shows whether game mode is on or off and displays the current defense state. It also includes:

- max speed slider
- goal X/Y/length sliders
- home X/Y sliders

The max speed slider controls the speed percentage sent to `CoreXYController`, capped by `MAX_MOTOR_PCT`.

### Bounds Tab

Controls physical and vision boundaries:

- Table ROI: limits where puck and mallet detection can happen.
- Radius: rounded corner radius for the table ROI mask.
- Mallet Box: calibrated motor movement area.
- Red Zone: restricted margins inside the mallet box.

The red zone is a safety band. The mallet should not be driven farther into it.

### Vision Tab

Controls HSV thresholds for:

- puck color
- mallet color

It also includes display checkboxes:

- Show Masks
- Show ROI / Bounds
- Show Trajectory

These settings are saved to the JSON config.

### Status Tab

Displays runtime information:

- FPS
- puck position, speed, and detection state
- mallet position, radius, and detection state
- Kalman status
- motor command summary
- red-zone status
- computed ROI, box, red-zone, goal, and home readouts

The "Print All Values to Console" button prints calibration constants and computed rectangles.

## Keyboard Controls

The OpenCV camera window handles keyboard input:

- `G`: toggle game mode
- `Q` or `ESC`: quit
- `S`: save screenshot
- WASD: manual movement when game mode is off
- arrow keys: manual movement when game mode is off
- numpad keys: manual movement when game mode is off
- space or numpad 5: stop manual movement

Manual key movement times out after `KEY_RELEASE_TIMEOUT`, which prevents stale key input from continuing forever.

## Defense State Machine

Every frame, the program sets a `defense_state`. The main states are:

- `SAFETY STOP`: mallet is not detected, so motors stop.
- `MANUAL`: game mode is off and keyboard input controls movement.
- `HOMING`: game mode is on and puck is lost, so the mallet returns home.
- `HOME`: mallet is close enough to the configured home position.
- `INTERCEPT`: puck is predicted to cross the goal, so the mallet moves to block.
- `GUARD`: puck is visible but not predicted to score, so the mallet patrols safely.
- `WAITING`: target is already reached or no movement is needed.
- `IDLE`: default state before logic selects another state.

Safety has priority. If the mallet is not detected, no game or manual command is allowed to move the motors.

## Goal Protection Logic

The defended goal is a vertical segment:

```text
x = goal_x
y = goal_y - goal_length/2 through goal_y + goal_length/2
```

The puck is considered dangerous when `predict_intercept()` predicts that the puck path will cross that goal segment.

`predict_intercept()` simulates the puck path using:

- current filtered puck position
- current filtered puck velocity
- table bounds
- a time limit
- a small timestep
- limited wall bounces

It returns an intercept point if the puck crosses the requested vertical line within the goal opening.

### Intercept Behavior

If the puck will cross the goal:

1. The program records the actual goal crossing point for visualization.
2. It predicts where the puck path crosses the safe patrol line.
3. The mallet target becomes that safe patrol crossing point.
4. The target Y is clamped inside both the goal opening and the safe movement area.
5. The target X is the safe left patrol line, not the raw goal line.

This is important because the physical mallet has radius. Driving the mallet center to the red-zone edge would let the mallet edge enter the restricted area. The planner now offsets by mallet radius before choosing a target.

### Guard Behavior

If the puck is visible but not predicted to cross the goal:

1. The mallet patrols on the safe left guard line.
2. It tracks the puck's Y position.
3. That Y target is clamped within the safe area and goal opening.
4. If the mallet is already at the target, the state becomes `WAITING`.

This keeps the mallet in a useful defensive position without letting autonomous behavior override the safety boundaries.

## Safety And Boundary Enforcement

The program has two layers of safety.

### Planner-Level Safety

`defensive_mode.py` clamps autonomous targets to a safe rectangle:

```text
safe_left   = box_left + red_left + mallet_radius
safe_right  = box_right - red_right - mallet_radius
safe_top    = box_top + red_top + mallet_radius
safe_bottom = box_bottom - red_bottom - mallet_radius
```

Autonomous home, guard, and intercept targets are clamped to this safe rectangle before converting to motor commands.

### Motor-Level Safety

`CoreXYController._enforce_bounds()` is still the final authority. It checks the mallet's current detected position and radius. If motion would push the mallet farther into the red zone, that component of motion is zeroed.

Movement away from the red zone is allowed so the robot can escape a boundary.

This means autonomous defense cannot bypass the project safety limits. Even if the planner chooses a bad target, the motor controller blocks unsafe movement.

## Movement Conversion

The defense logic computes a desired target in camera coordinates. Robot command axes are aligned with the camera output this way:

```text
robot forward = pixel X+
robot back    = pixel X-
robot right   = pixel Y+
robot left    = pixel Y-
```

Because the current motor wiring rotates the command axes relative to camera pixels, command-to-camera mapping is:

```text
camera_dx = vy
camera_dy = -vx
```

Auto-drive uses the inverse of that mapping:

```text
diff_x = target_x - mallet_x
diff_y = target_y - mallet_y

vx = -diff_y
vy = diff_x
```

`CoreXYController.drive()` then converts the command vector into motor A/B commands:

```text
Motor A = Vx + Vy
Motor B = Vx - Vy
```

The controller normalizes the motor commands so the faster motor reaches the configured speed cap while preserving the intended direction ratio.

## CoreXY Motor Protocol

Motor commands are sent over serial as two bytes per motor:

```text
Byte 1: 0x80 | motor_select | reverse_flag
Byte 2: speed percentage
```

Motor B sets the motor-select bit. Reverse motion sets the reverse bit. Speed is clamped from 0 to 100, with an additional project cap of `MAX_MOTOR_PCT`.

The controller de-duplicates repeated motor packets, so it does not spam serial with identical commands every frame.

## Visualization

The OpenCV window overlays:

- table ROI
- mallet box
- red zone
- goal line
- home marker
- predicted puck trajectory
- intercept marker
- puck and mallet circles
- defense state
- motor status
- Kalman filtering status
- safety warnings

If "Show Masks" is enabled, a second mask window shows puck and mallet threshold results.

## Timing And Performance

The frame is processed at a reduced resolution controlled by:

```python
PROCESSING_SCALE = 0.5
```

Detection runs on the scaled image, then positions are converted back to full-resolution pixels. This improves speed while keeping GUI and motor logic in the full camera coordinate system.

FPS is smoothed with an exponential moving average so the displayed value does not jump wildly frame to frame.

## Common Calibration Workflow

1. Start `defensive_mode.py`.
2. Use the Bounds tab to align the table ROI with the visible table.
3. Adjust the mallet box to match the physical movement region.
4. Adjust red-zone margins to protect forbidden areas.
5. Use the Vision tab to tune puck and mallet HSV thresholds.
6. Confirm Kalman-filtered puck and mallet positions are stable.
7. Set home and goal values on the Game tab.
8. Press "Print All Values to Console" if you want a readable calibration report.
9. Close and restart the program to confirm values reload from JSON.

## Failure Behavior

If the camera cannot open, the program prints an error and exits the tracking loop.

If the motor serial port cannot open, the program prints a warning and continues in vision-only mode.

If the mallet is not detected during operation, the motors stop.

If the puck is lost in game mode, the mallet moves home.

If config loading fails, defaults from `defensive_mode.py` are used and a new config is saved.

## Known Assumptions

- Camera resolution is expected to be 640x480 by default.
- The defended goal is on the left side of the camera frame.
- The puck color and mallet color are separable in HSV.
- The mallet must remain detectable for autonomous movement.
- The motor controller's red-zone enforcement is the final safety layer.

## Running

Install dependencies:

```bash
pip install opencv-python numpy pyserial
```

Run from the `software` directory:

```bash
python defensive_mode.py
```

Or from the repository root:

```bash
python software/defensive_mode.py
```
