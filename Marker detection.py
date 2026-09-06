import cv2
import numpy as np

# Load ArUco dictionary
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
parameters = cv2.aruco.DetectorParameters()

# Open camera
cap = cv2.VideoCapture(0)

# ⚠️ You should replace these with calibrated values later
# Temporary dummy camera matrix (works for demo, not accurate pose)
camera_matrix = np.array([[1000, 0, 320],
                          [0, 1000, 240],
                          [0, 0, 1]], dtype=float)

dist_coeffs = np.zeros((5, 1))  # assume no distortion

marker_length = 0.05  # meters (5 cm ~ 2 inch)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect markers
    corners, ids, rejected = cv2.aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )

    if ids is not None:
        # Draw markers
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Pose estimation
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_length, camera_matrix, dist_coeffs
        )

        for i in range(len(ids)):
            marker_id = ids[i][0]

            # Draw axis
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                              rvecs[i], tvecs[i], 0.03)

            # Position
            x, y, z = tvecs[i][0]

            # Display ID and position
            text = f"ID: {marker_id} | X:{x:.2f} Y:{y:.2f}"
            corner = corners[i][0][0]
            cv2.putText(frame, text,
                        (int(corner[0]), int(corner[1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

    cv2.imshow("ArUco Detection", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()