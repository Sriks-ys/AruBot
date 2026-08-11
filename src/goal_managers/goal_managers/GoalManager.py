import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState


class GoalManager(Node):

    def __init__(self):
        super().__init__('goal_manager')

        self.STAGE = "INIT"

        # Receive package location from drone
        self.drone_package_sub = self.create_subscription(
            Point,
            '/package_from_drone',
            self.drone_location_callback,
            10
        )

        # Receive ArUco pose from camera node
        self.aruco_pose_sub = self.create_subscription(
            PoseStamped,
            '/aruco_pose',
            self.aruco_pose_callback,
            10
        )

        # Nav2 action client
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        # Nav2 lifecycle client
        self.nav2_state_client = self.create_client(
            GetState,
            '/bt_navigator/get_state'
        )

        self.drone_package_location = (0, 0)

        self.Target_ID = -1

        self.GOAL_FRAME = 'map'

        self.NAVIGATION_ACTIVE = False

        self.MAX_TRIES = 3
        self.TRIES_REMAINING = self.MAX_TRIES

        self.last_goal = None

        # Store latest ArUco position
        self.aruco_target_position = None

        self.get_logger().info(
            "Goal Manager started"
        )

        self.get_logger().info(
            "Listening to /aruco_pose"
        )

    # --------------------------------------------------
    # Drone package callback
    # --------------------------------------------------

    def drone_location_callback(self, msg: Point):

        self.drone_package_location = (
            msg.x,
            msg.y
        )

        self.Target_ID = int(msg.z)

        x_goal = msg.y - 1
        y_goal = msg.x

        self.get_logger().info(
            f"Package from drone received | "
            f"X={msg.x:.3f} | "
            f"Y={msg.y:.3f} | "
            f"Target ID={self.Target_ID}"
        )

        self.send_goal(
            x_goal,
            y_goal
        )

        self.last_goal = (
            x_goal,
            y_goal,
            0.0
        )

    # --------------------------------------------------
    # ArUco camera callback
    # --------------------------------------------------

    def aruco_pose_callback(self, msg: PoseStamped):

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        self.aruco_target_position = (
            x,
            y,
            z
        )

        self.get_logger().info(
            f"ArUco target received | "
            f"X={x:.3f} m | "
            f"Y={y:.3f} m | "
            f"Z={z:.3f} m"
        )

    # --------------------------------------------------
    # Send navigation goal
    # --------------------------------------------------

    def send_goal(
        self,
        x,
        y,
        yaw=0.0
    ):

        if self.NAVIGATION_ACTIVE:

            self.get_logger().warn(
                "Active Navigation, ignoring goal"
            )

            return

        if not self.nav2_state_client.wait_for_service(
            timeout_sec=10.0
        ):

            self.get_logger().warn(
                "Nav2 Lifecycle is not active, shutting down"
            )

            rclpy.shutdown()

            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = (
            self.GOAL_FRAME
        )

        goal_msg.pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f"Setting Goal to ({x}, {y})"
        )

        future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        future.add_done_callback(
            self.goal_response_callback
        )

    # --------------------------------------------------
    # Nav2 goal response
    # --------------------------------------------------

    def goal_response_callback(
        self,
        future
    ):

        goal_handle = future.result()

        if not goal_handle.accepted:

            self.get_logger().warn(
                "Nav2 rejected the goal"
            )

            return

        self.NAVIGATION_ACTIVE = True

        self.get_logger().info(
            "Nav2 Accepted the Goal"
        )

        result_future = (
            goal_handle.get_result_async()
        )

        result_future.add_done_callback(
            self.goal_result_callback
        )

    # --------------------------------------------------
    # Nav2 result
    # --------------------------------------------------

    def goal_result_callback(
        self,
        future
    ):

        result = (
            future.result().result.error_code
        )

        self.NAVIGATION_ACTIVE = False

        self.get_logger().info(
            f"Navigation result: {result}"
        )

    # --------------------------------------------------
    # Nav2 feedback
    # --------------------------------------------------

    def feedback_callback(
        self,
        feedback_msg
    ):

        pass


def main(args=None):

    rclpy.init(args=args)

    node = GoalManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
