import sys

sys.path.insert(0, "/home/arubot/kmsxx/build/py")
sys.path.insert(0, "/usr/local/lib/python3/dist-packages")

import cv2
import numpy as np
import time
import math
import rclpy
from rclpy.node import Node

import tf2_ros
from geometry_msgs.msg import Vector3Stamped, PoseStamped, PointStamped
from tf2_geometry_msgs import do_transform_vector3, do_transform_pose, do_transform_point
from sensor_msgs.msg import Image
from picamera2 import Picamera2
from arubot_interface.srv import CameraPackagePose
from std_msgs.msg import UInt8, String

MARKER_SIZE = 0.07

class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')

        self.declare_parameter("target_id", 10)

        self.camera_command_sub = self.create_subscription(String, "/camera_command", self.command_callback, 10)

        self.image_publisher = self.create_publisher(Image, '/camera/package_pose/debug_image', 10)
        self.servo_publisher = self.create_publisher(UInt8, "/servo_cmd", 10)
        self.pose_publisher = self.create_publisher(PoseStamped, '/camera/package_pose', 10)

        self.camera_timer = self.create_timer(0.5, self.camera_timer_callback)

        fs = cv2.FileStorage("/home/arubot/pi_cam_calib.yaml", cv2.FILE_STORAGE_READ)
        self.camera_matrix = fs.getNode("K").mat()
        self.dist_coeffs = fs.getNode("D").mat()
        fs.release()

        if self.camera_matrix is None:
            raise RuntimeError("Could not load camera matrix")

        if self.dist_coeffs is None:
            raise RuntimeError("Could not load distortion coefficients")

        self.get_logger().info("Camera calibration loaded successfully")

        half = MARKER_SIZE / 2.0

        self.object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)    

        self.aruco = cv2.aruco
        self.dictionary = self.aruco.getPredefinedDictionary(self.aruco.DICT_4X4_50)

        try:
            self.params = self.aruco.DetectorParameters_create()
        except AttributeError:
            self.params = self.aruco.DetectorParameters()

        self.picam2 = Picamera2()
        config = self.picam2.create_still_configuration(main={"size": (800, 600), "format": "RGB888"})
        self.picam2.configure(config)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.T_top_front = None

        self.Target_ID = self.get_parameter('target_id').value
        self.tracking = False
        self.MAX_TRIES = 3
        self.Tries = self.MAX_TRIES

    def command_callback(self, msg: String):
        if (msg.data == "track"):
            self.picam2.start()
            time.sleep(0.5)
            self.servo_publisher.publish(UInt8(data = 20))
            self.tracking = True

    def camera_timer_callback(self):
        if self.tracking:
            frame = self.picam2.capture_array()
            self.get_package_pose_callback(frame)

    def get_package_pose_callback(self, frame):
        self.get_logger().info("Camera package pose requested")

        try:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
            debug_frame = frame.copy()

            corners, ids, _ = self.aruco.detectMarkers(frame, self.dictionary, parameters=self.params)

            if ids is not None:
                self.get_logger().warn(f"No Target ArUco marker detected stopping tracking trying again:")
                # Keep only markers whose ID equals TARGET_ID
                mask = (ids.flatten() == self.Target_ID)
                corners = [corner for corner, keep in zip(corners, mask) if keep]
                ids = ids[mask]

            if ids is None:
                if self.Tries > 0:
                    self.get_logger().warn(f"No Target ArUco marker detected stopping tracking trying again  {ids}")
                    self.publish_debug_image(debug_frame)
                    self.Tries -= 1
                else:
                    self.get_logger().warn("Finished Tracking")
                    self.tracking = False
                return
            
            if (self.Tries!=self.MAX_TRIES):
                self.Tries = self.MAX_TRIES

            self.aruco.drawDetectedMarkers(debug_frame, corners, ids)

            top_candidates = []
            front_candidates = []

            for marker_corners, marker_id in zip(corners, ids.flatten()):
                image_points = marker_corners[0].astype(np.float32)
                area = float(cv2.contourArea(image_points))

                success, rvec, tvec = cv2.solvePnP(self.object_points, image_points, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

                if not success:
                    self.get_logger().warn(f"solvePnP failed for ID {int(marker_id)}")
                    continue

                cv2.drawFrameAxes(debug_frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, MARKER_SIZE * 0.75)

                rotation_matrix, _ = cv2.Rodrigues(rvec)
                normal_camera = rotation_matrix @ np.array([0.0, 0.0, 1.0], dtype=np.float64)

                normal_msg = Vector3Stamped()
                normal_msg.header.frame_id = "camera"
                normal_msg.header.stamp = self.get_clock().now().to_msg()
                normal_msg.vector.x = float(normal_camera[0])
                normal_msg.vector.y = float(normal_camera[1])
                normal_msg.vector.z = float(normal_camera[2])

                try:
                    transform = self.tf_buffer.lookup_transform("base_link", "camera", rclpy.time.Time())
                    normal_base_msg = do_transform_vector3(normal_msg, transform)
                except Exception as e:
                    self.get_logger().warn(f"Could not transform from camera to base_link: {e}")
                    continue

                nx = float(normal_base_msg.vector.x)
                ny = float(normal_base_msg.vector.y)
                nz = float(normal_base_msg.vector.z)

                self.get_logger().info(f"Detected ID {int(marker_id)} | Area={area:.1f} px^2 | Normal=({nx:.2f}, {ny:.2f}, {nz:.2f})")

                center = np.mean(image_points, axis=0).astype(int)

                if abs(nz) > 0.85:
                    label = "TOP"
                    color = (0, 255, 0)
                    top_candidates.append({"T": self.pnp_to_transform(rvec, tvec), "area": area, "tvec": tvec, "rvec": rvec})
                else:
                    label = "FRONT"
                    color = (255, 0, 0)
                    front_candidates.append({"T": self.pnp_to_transform(rvec, tvec), "area": area, "tvec": tvec, "rvec": rvec})

                cv2.putText(debug_frame, f"{label} ID {int(marker_id)}", (center[0] + 10, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)

            T_camera_front = None
            T_camera_top = None
            front_candidate = None

            if front_candidates:
                front_candidate = max(front_candidates, key=lambda marker: marker["area"])
                T_camera_front = front_candidate["T"]

            if top_candidates:
                T_camera_top = max(top_candidates, key=lambda marker: marker["area"])["T"]

            if T_camera_front is not None and T_camera_top is not None:
                self.T_top_front = self.invert_transform(T_camera_top) @ T_camera_front
                self.get_logger().info("Learned Top -> Front transform")

            T_output = None

            if front_candidate is not None:
                T_output = front_candidate["T"]
            elif T_camera_top is not None and self.T_top_front is not None:
                T_output = T_camera_top @ self.T_top_front
            elif T_camera_top is not None:
                T_output = T_camera_top
            
            self.publish_debug_image(debug_frame)

            if T_output is None:
                self.get_logger().warn("Could not determine package pose")
                return

            response = PoseStamped()

            self.servo_publisher.publish(UInt8(data = self.calculate_servo_angle(T_output[:3, 3])))

            q = self.rotation_matrix_to_quaternion(T_output[:3, :3])

            response.header.frame_id = "camera"
            response.pose.position.x = T_output[0, 3]
            response.pose.position.y = T_output[1, 3]
            response.pose.position.z = T_output[2, 3]
            response.pose.orientation.x = q[0]
            response.pose.orientation.y = q[1]
            response.pose.orientation.z = q[2]
            response.pose.orientation.w = q[3]
            response.header.stamp = self.get_clock().now().to_msg()
            self.pose_publisher.publish(response)
            return 

        except Exception as e:
            self.tracking = False
            self.servo_publisher.publish(UInt8(data = 0))
            try:
                self.picam2.stop()  
            except Exception:
                pass
            
            self.get_logger().error(f"Camera/Aruco error: {str(e)}")
            return 

    def publish_debug_image(self, frame):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        self.image_publisher.publish(msg)

    def pnp_to_transform(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = tvec.reshape(3)
        return T

    def invert_transform(self, T):
        R = T[:3, :3]
        t = T[:3, 3]
        T_inv = np.eye(4, dtype=np.float64)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        return T_inv

    def destroy_node(self):
        try:
            self.picam2.stop()
        except Exception:
            pass
        super().destroy_node()

    def rotation_matrix_to_quaternion(self, R):
        q = np.empty(4, dtype=np.float64)
        trace = np.trace(R)

        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s

        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            q[3] = (R[2, 1] - R[1, 2]) / s
            q[0] = 0.25 * s
            q[1] = (R[0, 1] + R[1, 0]) / s
            q[2] = (R[0, 2] + R[2, 0]) / s

        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            q[3] = (R[0, 2] - R[2, 0]) / s
            q[0] = (R[0, 1] + R[1, 0]) / s
            q[1] = 0.25 * s
            q[2] = (R[1, 2] + R[2, 1]) / s

        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            q[3] = (R[1, 0] - R[0, 1]) / s
            q[0] = (R[0, 2] + R[2, 0]) / s
            q[1] = (R[1, 2] + R[2, 1]) / s
            q[2] = 0.25 * s

        return q

    def calculate_servo_angle(self, target_camera_position):
        target_msg = PointStamped()

        target_msg.header.frame_id = "camera"
        target_msg.header.stamp = self.get_clock().now().to_msg()

        target_msg.point.x = float(target_camera_position[0])
        target_msg.point.y = float(target_camera_position[1])
        target_msg.point.z = float(target_camera_position[2])

        try:
            transform = self.tf_buffer.lookup_transform("base_link", "camera", rclpy.time.Time())

            target_base = do_transform_point(target_msg, transform)

        except Exception as e:
            self.get_logger().warn(
                f"Could not transform target to base_link: {e}"
            )
            return None
        
        # x, z relative to servo origin
        x = target_base.point.x - 0.098162
        z = target_base.point.z + 0.110597

        angle_rad = math.atan2(z, x)

        angle_deg = math.degrees(angle_rad)

        
        angle_deg = max(0.0, min(85.0, angle_deg))

        return int(round(angle_deg))




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
