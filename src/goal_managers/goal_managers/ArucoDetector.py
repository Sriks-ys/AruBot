import sys

sys.path.insert(0, "/home/arubot/kmsxx/build/py")
sys.path.insert(0, "/usr/local/lib/python3/dist-packages")

import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from picamera2 import Picamera2
from arubot_interface.srv import CameraPackagePose


MARKER_SIZE = 0.07  # 7 cm


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # --------------------------------------------------
        # ROS 2 SERVICE
        # --------------------------------------------------

        self.service = self.create_service(
            CameraPackagePose,
            '/camera_package_pose',
            self.get_package_pose_callback
        )

        # --------------------------------------------------
        # DEBUG IMAGE PUBLISHER
        # --------------------------------------------------

        self.image_publisher = self.create_publisher(
            Image,
            '/camera/package_pose/debug_image',
            10
        )

        # --------------------------------------------------
        # CAMERA CALIBRATION
        # --------------------------------------------------

        fs = cv2.FileStorage(
            "/home/arubot/pi_cam_calib.yaml",
            cv2.FILE_STORAGE_READ
        )

        self.camera_matrix = fs.getNode("K").mat()
        self.dist_coeffs = fs.getNode("D").mat()

        fs.release()

        if self.camera_matrix is None:
            raise RuntimeError(
                "Could not load camera matrix"
            )

        if self.dist_coeffs is None:
            raise RuntimeError(
                "Could not load distortion coefficients"
            )

        self.get_logger().info(
            "Camera calibration loaded successfully"
        )

        # --------------------------------------------------
        # ARUCO REAL-WORLD CORNER POINTS
        # --------------------------------------------------

        half = MARKER_SIZE / 2.0

        self.object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        # --------------------------------------------------
        # ARUCO SETUP
        # --------------------------------------------------

        self.aruco = cv2.aruco

        self.dictionary = self.aruco.getPredefinedDictionary(
            self.aruco.DICT_4X4_50
        )

        try:
            self.params = self.aruco.DetectorParameters_create()
        except AttributeError:
            self.params = self.aruco.DetectorParameters()

        # --------------------------------------------------
        # CAMERA SETUP
        # --------------------------------------------------

        self.picam2 = Picamera2()

        config = self.picam2.create_still_configuration(
            main={
                "size": (800, 600),
                "format": "RGB888"
            }
        )

        self.picam2.configure(config)

        self.get_logger().info(
            "Aruco service node ready"
        )

        self.get_logger().info(
            "Selection mode: front-orientation filter + largest area"
        )

        self.get_logger().info(
            "Waiting for requests on /camera_package_pose"
        )

        self.get_logger().info(
            "Debug image topic: /camera/package_pose/debug_image"
        )

    # ======================================================
    # SERVICE CALLBACK
    # ======================================================

    def get_package_pose_callback(self, request, response):

        self.get_logger().info(
            "Camera package pose requested"
        )

        try:

            # ----------------------------------------------
            # CAPTURE ONE FRAME
            # ----------------------------------------------

            self.picam2.start()

            time.sleep(0.5)

            frame = self.picam2.capture_array()

            self.picam2.stop()

            self.get_logger().info(
                "Frame captured"
            )

            # ----------------------------------------------
            # DETECT ALL ARUCO MARKERS
            # ----------------------------------------------

            corners, ids, _ = self.aruco.detectMarkers(
                frame,
                self.dictionary,
                parameters=self.params
            )

            if ids is None:

                self.get_logger().warn(
                    "No ArUco marker detected"
                )

                return response

            # ----------------------------------------------
            # FIND FRONT-FACING CANDIDATES
            # ----------------------------------------------

            candidates = []

            for marker_corners, marker_id in zip(
                corners,
                ids.flatten()
            ):

                image_points = marker_corners[0].astype(
                    np.float32
                )

                area = float(
                    cv2.contourArea(image_points)
                )

                success, rvec, tvec = cv2.solvePnP(
                    self.object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )

                if not success:

                    self.get_logger().warn(
                        f"solvePnP failed for ID {int(marker_id)}"
                    )

                    continue

                # ------------------------------------------
                # MARKER NORMAL IN CAMERA FRAME
                # ------------------------------------------

                rotation_matrix, _ = cv2.Rodrigues(
                    rvec
                )

                normal_marker = np.array(
                    [0.0, 0.0, 1.0]
                )

                normal_camera = (
                    rotation_matrix @ normal_marker
                )

                nx = float(normal_camera[0])
                ny = float(normal_camera[1])
                nz = float(normal_camera[2])

                self.get_logger().info(
                    f"Detected ID {int(marker_id)} | "
                    f"Area={area:.1f} px^2 | "
                    f"Normal=({nx:.2f}, {ny:.2f}, {nz:.2f})"
                )

                # ------------------------------------------
                # FRONT-FACE FILTER
                #
                # Front marker should have normal mainly
                # along camera Z rather than camera Y.
                # ------------------------------------------

                if abs(nz) > 0.40:

                    self.get_logger().info(
                        f"ID {int(marker_id)} accepted as front-like"
                    )

                    candidates.append({
                        "id": int(marker_id),
                        "area": area,
                        "corners": marker_corners,
                        "rvec": rvec,
                        "tvec": tvec
                    })

                else:

                    self.get_logger().info(
                        f"ID {int(marker_id)} rejected as top-like"
                    )

            # ----------------------------------------------
            # CHECK IF ANY FRONT CANDIDATE EXISTS
            # ----------------------------------------------

            if len(candidates) == 0:

                self.get_logger().warn(
                    "No front-facing ArUco candidate found"
                )

                return response

            # ----------------------------------------------
            # SELECT LARGEST FRONT-FACING MARKER
            # ----------------------------------------------

            selected = max(
                candidates,
                key=lambda item: item["area"]
            )

            selected_id = selected["id"]
            selected_area = selected["area"]
            selected_corners = selected["corners"]
            rvec = selected["rvec"]
            tvec = selected["tvec"]

            self.get_logger().info(
                f"Selected front marker ID {selected_id} | "
                f"Area={selected_area:.1f} px^2"
            )

            # ----------------------------------------------
            # DRAW ALL DETECTED MARKERS
            # ----------------------------------------------

            self.aruco.drawDetectedMarkers(
                frame,
                corners,
                ids
            )

            # ----------------------------------------------
            # DRAW AXES FOR SELECTED MARKER
            # ----------------------------------------------

            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                0.05
            )

            # ----------------------------------------------
            # DRAW SELECTION TEXT
            # ----------------------------------------------

            center = np.mean(
                selected_corners[0],
                axis=0
            ).astype(int)

            cv2.putText(
                frame,
                f"SELECTED ID {selected_id} AREA {selected_area:.0f}",
                (
                    max(0, int(center[0]) - 120),
                    max(20, int(center[1]) - 20)
                ),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2
            )

            # ----------------------------------------------
            # PUBLISH DEBUG IMAGE
            # ----------------------------------------------

            image_msg = Image()

            image_msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            image_msg.header.frame_id = "camera"

            image_msg.height = frame.shape[0]
            image_msg.width = frame.shape[1]

            image_msg.encoding = "rgb8"
            image_msg.is_bigendian = 0

            image_msg.step = frame.shape[1] * 3
            image_msg.data = frame.tobytes()

            self.image_publisher.publish(
                image_msg
            )

            self.get_logger().info(
                "Published solvePnP verification frame"
            )

            # ----------------------------------------------
            # POSITION IN CAMERA FRAME
            # ----------------------------------------------

            x = float(tvec[0][0])
            y = float(tvec[1][0])
            z = float(tvec[2][0])

            distance = float(
                np.linalg.norm(tvec)
            )

            # ----------------------------------------------
            # RETURN POSESTAMPED
            # ----------------------------------------------

            response.pose.header.stamp = (
                self.get_clock().now().to_msg()
            )

            response.pose.header.frame_id = "camera"

            response.pose.pose.position.x = x
            response.pose.pose.position.y = y
            response.pose.pose.position.z = z

            # Orientation still fixed for now
            response.pose.pose.orientation.x = 0.0
            response.pose.pose.orientation.y = 0.0
            response.pose.pose.orientation.z = 0.0
            response.pose.pose.orientation.w = 1.0

            self.get_logger().info(
                f"Package pose from selected marker ID {selected_id} | "
                f"X={x:.3f} m | "
                f"Y={y:.3f} m | "
                f"Z={z:.3f} m | "
                f"Distance={distance:.3f} m"
            )

            return response

        except Exception as e:

            try:
                self.picam2.stop()
            except Exception:
                pass

            self.get_logger().error(
                f"Camera/Aruco error: {str(e)}"
            )

            return response

    # ======================================================
    # SHUTDOWN
    # ======================================================

    def destroy_node(self):

        try:
            self.picam2.stop()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = ArucoDetector()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
