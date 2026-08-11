import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from picamera2 import Picamera2


TARGET_ID = 10
MARKER_SIZE = 0.07  # 7 cm


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        # ROS 2 publisher
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/aruco_pose',
            10
        )

        # Load camera calibration
        fs = cv2.FileStorage(
            "/home/arubot/pi_cam_calib.yaml",
            cv2.FILE_STORAGE_READ
        )

        self.camera_matrix = fs.getNode("K").mat()
        self.dist_coeffs = fs.getNode("D").mat()

        fs.release()

        if self.camera_matrix is None:
            raise RuntimeError(
                "Could not load camera matrix from "
                "/home/arubot/pi_cam_calib.yaml"
            )

        self.get_logger().info(
            "Camera calibration loaded successfully"
        )

        # Real coordinates of the four ArUco corners
        half = MARKER_SIZE / 2.0

        self.object_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        # ArUco setup
        self.aruco = cv2.aruco

        self.dictionary = self.aruco.getPredefinedDictionary(
            self.aruco.DICT_4X4_50
        )

        try:
            self.params = self.aruco.DetectorParameters_create()
        except AttributeError:
            self.params = self.aruco.DetectorParameters()

        # Camera setup
        self.picam2 = Picamera2()

        config = self.picam2.create_preview_configuration(
            main={
                "size": (800, 600),
                "format": "RGB888"
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

        time.sleep(2)

        self.get_logger().info(
            f"ArUco detector started. Target ID: {TARGET_ID}"
        )

        self.get_logger().info(
            "Publishing target pose on /aruco_pose"
        )

        # Run detection every 0.2 seconds
        self.timer = self.create_timer(
            0.2,
            self.detect_marker
        )

    def detect_marker(self):

        frame = self.picam2.capture_array()

        corners, ids, _ = self.aruco.detectMarkers(
            frame,
            self.dictionary,
            parameters=self.params
        )

        if ids is None:
            return

        for marker_corners, marker_id in zip(
            corners,
            ids.flatten()
        ):

            if int(marker_id) != TARGET_ID:
                continue

            image_points = marker_corners[0].astype(
                np.float32
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
                    "solvePnP failed"
                )
                return

            x = float(tvec[0][0])
            y = float(tvec[1][0])
            z = float(tvec[2][0])

            distance = float(
                np.linalg.norm(tvec)
            )

            # Determine left / center / right
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

            # Create PoseStamped message
            msg = PoseStamped()

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            msg.header.frame_id = "camera"

            # Position from solvePnP
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z

            # Orientation not used yet
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0

            # Publish ROS 2 message
            self.pose_publisher.publish(msg)

            self.get_logger().info(
                f"Target {TARGET_ID} | "
                f"{direction} | "
                f"X={x:.3f} m | "
                f"Y={y:.3f} m | "
                f"Z={z:.3f} m | "
                f"Distance={distance:.3f} m"
            )

            break

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
