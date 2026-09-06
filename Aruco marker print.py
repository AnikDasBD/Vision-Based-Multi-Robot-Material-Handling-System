import cv2
import os

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)

# Default marker size (for robots & objects)
marker_size = 600  

# ID mapping
markers = {
    "car_1": 0,
    "car_2": 1,
    "obj_1": 10,
    "obj_2": 11,
    "obj_3": 12,    
    "obj_4": 13,
    "obj_5": 14,
    "obj_6": 15,
    "drop_zone": 20,

    # NEW car zones
    "car_zone_1": 21,
    "car_zone_2": 22
}

# output folder
output_dir = "aruco_markers"
os.makedirs(output_dir, exist_ok=True)

for name, marker_id in markers.items():
    
    # Make drop zone and car zones slightly bigger
    if name in ["drop_zone", "car_zone_1", "car_zone_2"]:
        size = 800   # bigger for stability
    else:
        size = marker_size

    img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, size)

    filename = os.path.join(output_dir, f"{name}_id_{marker_id}.png")
    cv2.imwrite(filename, img)

    print(f"Saved: {filename}")

print("All markers generated successfully.")