# heading_test.py
# ------------------------------------------------------------
# Universal ArUco Debug Tool
#
# Displays:
#   • Marker name
#   • Marker ID
#   • Image X,Y position
#   • Distance from camera (Z)
#   • Raw heading
#   • Corrected heading
#
# Press Q to quit.
# ------------------------------------------------------------

import cv2
import numpy as np
import math
import config as cfg

# ------------------------------------------------------------
# Camera
# ------------------------------------------------------------

cap = cv2.VideoCapture(cfg.CAM_INDEX)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg.FRAME_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg.FRAME_H)

cv2.namedWindow("Heading Test", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Heading Test", cfg.FRAME_W, cfg.FRAME_H)

aruco_dict = cfg.ARUCO_DICT
aruco_params = cfg.ARUCO_PARAMS

# ------------------------------------------------------------
# Main Loop
# ------------------------------------------------------------

while True:

    ret, frame = cap.read()

    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = cv2.aruco.detectMarkers(
        gray,
        aruco_dict,
        parameters=aruco_params
    )

    if ids is not None:

        for i, marker_id in enumerate(ids.flatten()):

            # ------------------------------------------------
            # Select correct physical marker size
            # ------------------------------------------------

            if marker_id in cfg.ROBOT_IDS:
                marker_len = cfg.MARKER_LEN_ROBOT

            elif marker_id in cfg.OBJECT_IDS:
                marker_len = cfg.MARKER_LEN_OBJECT

            elif marker_id in cfg.ZONE_IDS:
                marker_len = cfg.MARKER_LEN_ZONE

            else:
                continue

            # ------------------------------------------------
            # Pose estimation
            # ------------------------------------------------

            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]],
                marker_len,
                cfg.CAM_MTX,
                cfg.DIST
            )

            # ------------------------------------------------
            # Rotation matrix
            # ------------------------------------------------

            R, _ = cv2.Rodrigues(rvec[0])

            raw_yaw = (
                math.degrees(
                    math.atan2(R[1, 0], R[0, 0])
                ) + 360
            ) % 360

            corrected_yaw = (
                raw_yaw + cfg.YAW_OFFSET
            ) % 360

            # ------------------------------------------------
            # Marker centre
            # ------------------------------------------------

            cx = int(np.mean(corners[i][0][:, 0]))
            cy = int(np.mean(corners[i][0][:, 1]))

            # Distance from camera
            distance = float(tvec[0][0][2])

            # Name
            name = cfg.MARKER_NAMES.get(marker_id, f"ID {marker_id}")

            # ------------------------------------------------
            # Draw marker border
            # ------------------------------------------------

            cv2.polylines(
                frame,
                [corners[i].astype(int)],
                True,
                (0, 255, 0),
                2
            )

            # ------------------------------------------------
            # Draw coordinate axes
            # ------------------------------------------------

            cv2.drawFrameAxes(
                frame,
                cfg.CAM_MTX,
                cfg.DIST,
                rvec,
                tvec,
                marker_len * 0.5
            )

            # ------------------------------------------------
            # Text position
            # ------------------------------------------------

            x = max(10, cx - 120)
            y = max(30, cy - 90)

            cv2.putText(
                frame,
                f"{name} (ID {marker_id})",
                (x, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"X={cx}  Y={cy}",
                (x, y + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"Z={distance:.3f} m",
                (x, y + 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (255, 255, 255),
                2
            )

            cv2.putText(
                frame,
                f"Raw={raw_yaw:.1f}",
                (x, y + 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                f"Corrected={corrected_yaw:.1f}",
                (x, y + 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 255),
                2
            )

    # --------------------------------------------------------
    # Show frame
    # --------------------------------------------------------

    cv2.imshow("Heading Test", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

# ------------------------------------------------------------
# Cleanup
# ------------------------------------------------------------

cap.release()
cv2.destroyAllWindows()