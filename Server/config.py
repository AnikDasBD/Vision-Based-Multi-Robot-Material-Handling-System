# config.py

import cv2
import numpy as np

# ── Fleet ─────────────────────────────────────────────────────────────────────
ROBOT_IDS  = [0, 1]
ROBOT_IPS  = {0: "192.168.0.151", 1: "192.168.0.152"}
ROBOT_PORT = 80

# Heading offsets measured with heading_test.py
YAW_OFFSETS = {0: 90, 1: 270}

# Marker-to-robot-center offset in pixels (~1000 px/m at 1.5m height)
# Robot 0: marker 2.325cm ahead of center → -23px
# Robot 1: marker 2.5cm behind center    → +25px
ROBOT_MARKER_OFFSETS = {0: -23, 1: 25}

# Robot geometry
ROBOT_GEOMETRIES = {
    0: {"length": 0.130, "width": 0.09},
    1: {"length": 0.136, "width": 0.09},
}

# ── Marker IDs ────────────────────────────────────────────────────────────────
OBJECT_IDS    = [10, 11, 12, 13, 14, 15]
DROP_ID       = 20
CAR_ZONE_IDS  = {0: 21, 1: 22}   # robot_id → home zone marker
ZONE_IDS      = [DROP_ID, 21, 22]
REFERENCE_MARKER_IDS = [DROP_ID, 21, 22]   # fixed ground markers for stabilisation

MARKER_NAMES = {
    0:"robot_0", 1:"robot_1",
    10:"obj_1",  11:"obj_2",  12:"obj_3",
    13:"obj_4",  14:"obj_5",  15:"obj_6",
    20:"drop",   21:"zone_0", 22:"zone_1",
}

# ── ArUco ─────────────────────────────────────────────────────────────────────
ARUCO_DICT   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()

# Physical marker sizes
MARKER_LEN_ROBOT  = round(2.5 * 0.0254, 4)   # 0.0635 m
MARKER_LEN_OBJECT = round(2.0 * 0.0254, 4)   # 0.0508 m
MARKER_LEN_ZONE   = round(3.5 * 0.0254, 4)   # 0.0889 m

# Object center offset in pixels
# (marker_size_cm + object_depth_cm) / 2 = (5.08 + 2.5)/2 = 3.79cm ≈ 38px
OBJ_CENTER_OFFSET_PX = 38

# ── Camera ────────────────────────────────────────────────────────────────────
CAM_INDEX    = 0
FRAME_W      = 1920
FRAME_H      = 1080
SMOOTH_ALPHA = 0.15

CAM_MTX = np.array([[1500, 0, 960],
                     [0, 1500, 540],
                     [0,    0,   1]], dtype=float)
DIST = np.zeros((5, 1))

# ── A* Grid ───────────────────────────────────────────────────────────────────
GRID_COLS    = 64
GRID_ROWS    = 36
ROBOT_CELL_R = 2

# ── Navigation thresholds (pixels) ───────────────────────────────────────────
ARRIVE_PX           = 40
AT_GOAL_OBJ_PX      = 15
AT_GOAL_DROP_PX     = 120
AT_GOAL_CAR_ZONE_PX = 180
DROP_ZONE_HOLD_DIST = 300   # wait here while drop zone is occupied

# ── Collision ─────────────────────────────────────────────────────────────────
# Robot diagonal 17.3cm → radius 8.65cm + 1.5cm safety = 10.15cm ≈ 102px
ROBOT_RADIUS_PX  = 87    # 17.3cm/2 at ~1000px/m
SAFETY_MARGIN_PX = 15    # 1.5cm
COLLISION_DIST_PX = 2 * (ROBOT_RADIUS_PX + SAFETY_MARGIN_PX)   # 204px

# ── Controller gains ──────────────────────────────────────────────────────────
KP_ANG               = 0.15
KD_ANG               = 0.6    # NEW: derivative gain for PD heading control
KP_LIN               = 0.006
MAX_V                = 0.15
MAX_W                = 0.6
HEADING_DEADZONE_DEG = 25

# ── Motor trim (PWM counts added to left/right wheel independently) ───────────
# Robot 0 drifts left → add +2 to left wheel to compensate mechanical bias.
# Robot 1 is well-balanced → both trims at 0.
# Tune LEFT_TRIM / RIGHT_TRIM per robot after physical testing.
MOTOR_TRIM = {
    0: (2, 0),   # (left_trim, right_trim) for Robot 0
    1: (0, 0),   # (left_trim, right_trim) for Robot 1
}

# ── Stop hysteresis (pixels) ──────────────────────────────────────────────────
# Prevents stop/start oscillations when the robot hovers near a goal.
# Robot 0 has wider hysteresis because its ArUco pose is noisier (drift issue).
# Format: (stop_px, resume_px)  — resume > stop to create a dead-band.
STOP_HYSTERESIS = {
    0: (55, 70),   # Robot 0: stop at 55 px, don't move again until 70 px away
    1: (40, 60),   # Robot 1: tighter, less drift
}

# ── Timing ────────────────────────────────────────────────────────────────────
CONTROL_HZ      = 20
PAUSE_AT_OBJECT = 3.5    # seconds — magnet engages
PAUSE_AT_DROP   = 2.0    # seconds — magnet off, object drops
OFFLINE_FRAMES  = 30     # frames before declaring robot offline (~1.5s)
