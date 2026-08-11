import rclpy 
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from lifecycle_msgs.srv import GetState


class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        
        self.stage_id = 0
        self.STAGES = ["INIT", "NEXT_PACKAGE", "COLLECT_PACKAGE", "READY PAST OBSTACLE", "BEFORE DOOR", "CORRIDOR", "BEFORE DOOR 2", "RECIEPIENT"]
        self.STAGE = self.STAGES[self.stage_id]

        self.drone_package_sub = self.create_subscription(Point, '/package_from_drone', self.drone_location_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        
        self.nav2_state_client = self.create_client(GetState, '/bt_navigator/get_state')

        self.drone_package_location = (0, 0)
        self.Target_ID = -1

        self.GOAL_FRAME = 'map'

        self.NAVIGATION_ACTIVE = False

        self.MAX_TRIES = 3
        self.TRIES_REMAINING = self.MAX_TRIES

        self.last_goal = None


    def drone_location_callback(self, msg: Point):
        self.drone_package_location = (msg.x, msg.y)
        self.Target_ID = msg.z
        
        x_goal = msg.y - 1
        y_goal = msg.x

        
        self.send_goal(x_goal, y_goal)

        self.last_goal = (x_goal, y_goal, 0.0)

    def send_goal(self, x, y, yaw = 0.0):

        if self.NAVIGATION_ACTIVE:
            self.get_logger().warn("Active Navigation, Ignoring goal")
            return 

        if not self.nav2_state_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().warn("Nav2 Lifecycle is not active, Shutting Down")
            rclpy.shutdown()
            return

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = self.GOAL_FRAME
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        self.get_logger().info(f"Setting Goal to ({x}, {y})")
        
        future = self.nav_client.send_goal_async(
        goal_msg,
        feedback_callback=self.feedback_callback
        )

        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn(f"Nav2 rejected the goal: {goal_handle}")
            return 
        
        self.NAVIGATION_ACTIVE = True
        self.get_logger().info("Nav2 Accepted the Goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        result_wrapper = future.result()

        status = result_wrapper.status
        result = result_wrapper.result

        self.NAVIGATION_ACTIVE = False

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Navigation result: {result}')
            
            self.stage_id += 1
            self.STAGE = self.STAGES[self.stage_id]

            if self.TRIES_REMAINING != self.MAX_TRIES:
                self.get_logger().info(f"Goal Succseeded after {self.MAX_TRIES - self.TRIES_REMAINING}")
                self.TRIES_REMAINING = self.MAX_TRIES
        
        else:
            if (self.TRIES_REMAINING > 0):
                self.get_logger().warn(f"Navigation to {self.STAGE} failed, retrying to {self.last_goal}")
                self.TRIES_REMAINING -= 1
                self.get_logger().warn(f"Tries remaining: {self.TRIES_REMAINING}")
                self.send_goal(*self.last_goal)
            else:
                self.get_logger().error("Goal FAILED")
                rclpy.shutdown()

    def feedback_callback(self, feedback_msg):

        pass

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