import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose, DockRobot
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String, UInt8

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        
        self.stage_id = 0
        self.STAGES = ["INIT", "NEXT_PACKAGE", "COLLECT_PACKAGE", "SOUND RETREAT", "READY PAST OBSTACLE", "BEFORE DOOR", "CORRIDOR", "BEFORE DOOR 2", "RECIEPIENT"]
        self.STAGE = self.STAGES[self.stage_id]

        self.drone_package_sub = self.create_subscription(Point, '/package_from_drone', self.drone_location_callback, 10)
        self.camera_sub = self.create_subscription(UInt8, "/cam_feedback", self.camera_callback, 10)
        self.controller_sub = self.create_subscription(UInt8, "/controller_feedback", self.controller_feedback, 10)

        self.camera_command_pub = self.create_publisher(String, "/camera_command", 10)
        self.controller_cmd_pub = self.create_publisher(String, "/controller_cmd", 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.dock_client = ActionClient(self, DockRobot, '/dock_robot')

        self.nav2_state_client = self.create_client(GetState, '/bt_navigator/get_state')

        self.Main_timer = self.create_timer(10, self.main_thread)

        self.drone_package_location = (0, 0)
        self.Target_ID = -1

        self.NAVIGATION_ACTIVE = False

        self.MAX_TRIES = 5
        self.TRIES_REMAINING = self.MAX_TRIES

        self.last_goal = None

        self.state_command_initiated = False
        self.cam_ack = False
        
        self.package_staging_pose = None

        self.WARE_HOUSE_CENTER = (0.37, 3.01, 0.0)
        self.PAST_OBSTACLES = (3.56, 3.64, 0.0)

    def main_thread(self):
        if self.STAGE == "INIT":
            return 

        elif self.STAGE == "NEXT_PACKAGE":
            self.get_logger().info("Next to Package")
            self.get_logger().info("Starting camera for tracking")
            if not self.state_command_initiated:
                self.camera_command_pub.publish(String(data = "track"))
                self.state_command_initiated = True
                
            if self.cam_ack:
                self.send_dock_goal()
                self.cam_ack = False

        elif self.STAGE == "COLLECT_PACKAGE":
            self.get_logger().info("Collecting Package")
            # Add later
            self.stage_id += 1
            self.STAGE = self.STAGES[self.stage_id]

        elif self.STAGE == "SOUND RETREAT":
            self.get_logger().info("Going Back")
            if not self.state_command_initiated:
                self.controller_cmd_pub.publish(String(data = "retreat"))
                self.state_command_initiated = True

        elif self.STAGE == "READY PAST OBSTACLE":
            self.get_logger().info("Getting Ready to get past obstacles")
            if not self.state_command_initiated:
                self.send_nav2_goal(*self.WARE_HOUSE_CENTER) #can I send goal like this ?
                self.last_goal = self.WARE_HOUSE_CENTER
                self.state_command_initiated = True

        elif self.STAGE == "BEFORE DOOR":
            self.get_logger().info("Moving past obstacles and stopping before door")
            if not self.state_command_initiated:
                self.send_nav2_goal(*self.PAST_OBSTACLES)
                self.last_goal = self.PAST_OBSTACLES
                self.state_command_initiated = True

    def camera_callback(self, msg: UInt8):
        if (msg.data == 0):
            self.get_logger().info("Camera Acknowledged")
            self.cam_ack = True
    
    def controller_feedback(self, msg: UInt8):
        if (msg.data == 0):
            self.get_logger().info("Controller Finished Retreating")
            self.stage_id += 1
            self.STAGE = self.STAGES[self.stage_id]
            self.state_command_initiated = False
            self.get_logger().info(f"Reached Stage {self.STAGE}")
    
    def drone_location_callback(self, msg: Point):
        self.drone_package_location = (msg.x, msg.y)
        self.Target_ID = msg.z
        
        x_goal = msg.y - 1.2
        y_goal = msg.x

        self.send_nav2_goal(x_goal, y_goal)

        self.last_goal = (x_goal, y_goal, 0.0)
        self.package_staging_pose = (x_goal, y_goal, 0.0)

    def send_nav2_goal(self, x, y, yaw = 0.0):

        if self.NAVIGATION_ACTIVE:
            self.get_logger().warn("Active Navigation, Ignoring goal")
            return 

        if not self.nav2_state_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Nav2 Lifecycle is not active, Shutting Down")
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        self.get_logger().info(f"Setting Goal to ({x}, {y})")
        
        future = self.nav_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        future.add_done_callback(self.nav2_goal_response_callback)
    
    def nav2_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(f"Nav2 rejected the goal: {goal_handle}")
            return 
        
        self.NAVIGATION_ACTIVE = True
        self.get_logger().info("Nav2 Accepted the Goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_goal_result_callback)
    
    def nav2_goal_result_callback(self, future):
        result_wrapper = future.result()

        status = result_wrapper.status
        result = result_wrapper.result

        self.NAVIGATION_ACTIVE = False
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Navigation result: {result}')
            self.stage_id += 1
            self.STAGE = self.STAGES[self.stage_id]
            self.get_logger().info(f"Reached Stage: {self.STAGE}")
            self.state_command_initiated = False
            if self.TRIES_REMAINING != self.MAX_TRIES:
                self.get_logger().info(f"Goal Succseeded after {self.MAX_TRIES - self.TRIES_REMAINING}")
                self.TRIES_REMAINING = self.MAX_TRIES
        else:
            if (self.TRIES_REMAINING > 0):
                self.get_logger().warn(f"Navigation to {self.STAGE} failed, retrying to {self.last_goal}")
                self.TRIES_REMAINING -= 1
                self.get_logger().warn(f"Tries remaining: {self.TRIES_REMAINING}")
                self.send_nav2_goal(*self.last_goal)
            else:
                self.get_logger().error("Goal FAILED")
                rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        pass

    def send_dock_goal(self):
        if self.NAVIGATION_ACTIVE:
            self.get_logger().warn("Active Docking, Ignoring dock")
            return 

        goal_msg = DockRobot.Goal()

        goal_msg.use_dock_id = False
        goal_msg.dock_id = ""

        goal_msg.dock_pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.dock_pose.header.frame_id = "map"

        goal_msg.dock_pose.pose.position.x = self.package_staging_pose[0]
        goal_msg.dock_pose.pose.position.y = self.package_staging_pose[1]
        goal_msg.dock_pose.pose.position.z = 0.0

        goal_msg.dock_type = "simple_noncharging_dock"
        goal_msg.max_staging_time = 1000.0
        goal_msg.navigate_to_staging_pose = False

        future = self.dock_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        future.add_done_callback(self.dock_goal_response_callback)
    
    def dock_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info(f"Docking server rejected the goal: {goal_handle}")
            return 
        
        self.NAVIGATION_ACTIVE = True
        self.get_logger().info("Docking server accepted the goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.docking_server_result_callback)

    def docking_server_result_callback(self, future):
        wrapper = future.result()
        status = wrapper.status
        result = wrapper.result

        self.NAVIGATION_ACTIVE = False
        if status == GoalStatus.STATUS_SUCCEEDED and result.success:
            self.get_logger().info(f'Navigation result: {result}')
            self.stage_id += 1
            self.STAGE = self.STAGES[self.stage_id]
            self.get_logger().info(f"Reached Stage: {self.STAGE}")
            self.get_logger().info("Stopping Camera")
            self.camera_command_pub.publish(String(data = "stop_tracking"))
            self.state_command_initiated = False
        else:
            self.get_logger().error("Mission FAILED")
            rclpy.shutdown()

def main():
    rclpy.init()

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