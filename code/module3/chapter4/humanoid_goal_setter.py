import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator # Simplified Navigator

class HumanoidGoalSetter(Node):
    def __init__(self):
        super().__init__('humanoid_goal_setter')
        self.navigator = BasicNavigator() # Simplified Nav2 interface

    def set_goal(self, x, y, yaw):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = np.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = np.cos(yaw / 2.0)

        self.get_logger().info(f"Setting goal to x={x}, y={y}, yaw={yaw}")
        self.navigator.goToPose(goal_pose)

        # Wait for navigation to complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance remaining: {feedback.distance_remaining} meters')
                if feedback.navigation_time > 600: # 10 minutes timeout
                    self.navigator.cancelTask()
                    self.get_logger().warn("Navigation timed out!")
                    break
        
        result = self.navigator.getResult()
        if result == BasicNavigator.TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == BasicNavigator.TaskResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == BasicNavigator.TaskResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().info('Goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)
    goal_setter = HumanoidGoalSetter()
    
    # Example: Set a goal for the humanoid
    # Assume the robot is localized in a 'map' frame
    import numpy as np
    goal_setter.set_goal(x=1.0, y=0.0, yaw=np.pi/2.0) # Move to (1,0) facing +Y

    rclpy.spin(goal_setter)
    goal_setter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()