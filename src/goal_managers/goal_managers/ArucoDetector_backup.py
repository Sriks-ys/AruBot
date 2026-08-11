import cv2
import numpy as np
import time
from picamera2 import Picamera2

TARGET_ID = 10
MARKER_SIZE = 0.07  # 7 cm

# Load camera calibration
fs = cv2.FileStorage(
    "/home/arubot/pi_cam_calib.yaml",
    cv2.FILE_STORAGE_READ
)

camera_matrix = fs.getNode("K").mat()
dist_coeffs = fs.getNode("D").mat()
fs.release()

if camera_matrix is None:
    print("ERROR: Could not load camera matrix")
    raise SystemExit

print("Camera Matrix:")
print(camera_matrix)

print("Distortion Coefficients:")
print(dist_coeffs)

# Real coordinates of the four ArUco corners
half = MARKER_SIZE / 2.0

object_points = np.array([
    [-half,  half, 0],
    [ half,  half, 0],
    [ half, -half, 0],
    [-half, -half, 0]
], dtype=np.float32)

# Camera setup
picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={
        "size": (800, 600),
        "format": "RGB888"
    }
)

picam2.configure(config)
picam2.start()

time.sleep(2)

# ArUco setup
aruco = cv2.aruco

dictionary = aruco.getPredefinedDictionary(
    aruco.DICT_4X4_50
)

try:
    params = aruco.DetectorParameters_create()
except AttributeError:
    params = aruco.DetectorParameters()

print()
print("Real-time ArUco solvePnP started")
print("Target ID:", TARGET_ID)
print("Press Ctrl+C to stop")
print()

try:
    while True:

        frame = picam2.capture_array()

        corners, ids, rejected = aruco.detectMarkers(
            frame,
            dictionary,
            parameters=params
        )

        found = False

        if ids is not None:

            for marker_corners, marker_id in zip(
                corners,
                ids.flatten()
            ):

                if marker_id == TARGET_ID:

                    found = True

                    image_points = marker_corners[0].astype(
                        np.float32
                    )

                    success, rvec, tvec = cv2.solvePnP(
                        object_points,
                        image_points,
                        camera_matrix,
                        dist_coeffs,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE
                    )

                    if success:

                        x = float(tvec[0][0])
                        y = float(tvec[1][0])
                        z = float(tvec[2][0])

                        distance = float(
                            np.linalg.norm(tvec)
                        )

                        center_x = int(
                            np.mean(
                                marker_corners[0][:, 0]
                            )
                        )

                        image_center = frame.shape[1] // 2

                        if center_x < image_center - 50:
                            direction = "LEFT"
                        elif center_x > image_center + 50:
                            direction = "RIGHT"
                        else:
                            direction = "CENTER"

                        print(
                            f"Target {TARGET_ID} | "
                            f"{direction} | "
                            f"X={x:.3f} m | "
                            f"Y={y:.3f} m | "
                            f"Z={z:.3f} m | "
                            f"Distance={distance:.3f} m"
                        )

                    break

        if not found:
            print(
                f"Target {TARGET_ID}: NOT FOUND"
            )

        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopped")

finally:
    picam2.stop()
