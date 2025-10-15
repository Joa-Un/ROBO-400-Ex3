#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from math import atan2, sqrt, pi
from tf_transformations import euler_from_quaternion
import time


class MotionController(Node):
    def __init__(self):
        super().__init__('motion_controller')
        # Publishers
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.distance_error_pub = self.create_publisher(Float32, '/distance_error', 10)
        self.heading_error_pub = self.create_publisher(Float32, '/heading_error', 10)

        # Subscriber
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Timer
        self.timer = self.create_timer(0.1, self.update)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Goal state
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None

        # Path following
        self.path_points = []
        self.path_index = 0

        # Mode: 'point', 'pose', 'path'
        self.mode = None

        # Internal error log
        self.error_log = []  # store errors [timestamp, distance_error, heading_error]
        self.start_time = self.get_clock().now().to_msg().sec

        self.get_logger().info("Choose mode: 1) Go to Point 2) Go to Pose 3) Follow Path")
        self.mode_input()

    # -------------------- User input --------------------
    def mode_input(self):
        choice = input("Enter mode number: ").strip()
        if choice == '1':
            self.mode = 'point'
            x, y = self.get_goal_input()
            self.goal_x, self.goal_y = x, y
            self.get_logger().info(f"New goal: x={self.goal_x}, y={self.goal_y}")
        elif choice == '2':
            self.mode = 'pose'
            x, y, theta = self.get_goal_pose_input()
            self.goal_x, self.goal_y, self.goal_theta = x, y, theta
            self.get_logger().info(f"New goal pose: x={self.goal_x}, y={self.goal_y}, theta={self.goal_theta}")
        elif choice == '3':
            self.mode = 'path'
            self.define_path()
        else:
            self.get_logger().info("Invalid choice. Exiting...")
            rclpy.shutdown()
            exit(0)

    def get_goal_input(self):
        x_input = input("Enter goal x,y (comma-separated): ").strip()
        x_str, y_str = x_input.split(',')
        return float(x_str), float(y_str)

    def get_goal_pose_input(self):
        inp = input("Enter goal x,y,theta (comma-separated, theta in radians): ").strip()
        x_str, y_str, theta_str = inp.split(',')
        return float(x_str), float(y_str), float(theta_str)

    def define_path(self):
        # Example square path
        self.path_points = [(-1, -1), (-1, -2), (-2, -2), (-2, -1)]
        self.path_index = 0
        self.goal_x, self.goal_y = self.path_points[self.path_index]

    # -------------------- Odometry callback --------------------
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([q.x, q.y, q.z, q.w])

    # -------------------- Control loop --------------------
    def update(self):
        if self.goal_x is None:
            return

        msg = Twist()

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance = sqrt(dx**2 + dy**2)
        angle_to_goal = atan2(dy, dx)
        heading_error = angle_to_goal - self.theta

        # Normalize heading error
        while heading_error > pi:
            heading_error -= 2*pi
        while heading_error < -pi:
            heading_error += 2*pi

        # Log errors with timestamp
        timestamp = self.get_clock().now().to_msg().sec - self.start_time
        self.error_log.append([timestamp, distance, heading_error])

        # Publish errors for rqt_plot
        dist_msg = Float32()
        dist_msg.data = distance
        self.distance_error_pub.publish(dist_msg)

        head_msg = Float32()
        head_msg.data = heading_error
        self.heading_error_pub.publish(head_msg)

        # Control logic
        if distance > 0.05:
            msg.linear.x = 0.3
            msg.angular.z = 1.5 * heading_error
            self.get_logger().info(f"Moving: distance={distance:.2f}, heading_error={heading_error:.2f}")
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if self.mode == 'point':
                self.stop_robot()
                self.get_logger().info("Point reached! Enter new goal:")
                self.goal_x, self.goal_y = self.get_goal_input()
            elif self.mode == 'pose':
                self.get_logger().info("Pose reached! Rotating to goal theta...")
                angle_error = self.goal_theta - self.theta
                while angle_error > pi:
                    angle_error -= 2*pi
                while angle_error < -pi:
                    angle_error += 2*pi
                if abs(angle_error) > 0.05:
                    msg.angular.z = 1.5 * angle_error
                else:
                    self.get_logger().info("Goal pose reached! Enter new pose:")
                    self.goal_x, self.goal_y, self.goal_theta = self.get_goal_pose_input()
            elif self.mode == 'path':
                self.path_index = (self.path_index + 1) % len(self.path_points)
                self.goal_x, self.goal_y = self.path_points[self.path_index]
                self.get_logger().info(f"Moving to next path point {self.goal_x, self.goal_y}")

        self.publisher_.publish(msg)

    # -------------------- Stop robot --------------------
    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        for _ in range(5):
            self.publisher_.publish(stop_msg)
            time.sleep(0.1)
        self.get_logger().info("Robot stopped.")

# -------------------- Main --------------------
def main(args=None):
    rclpy.init(args=args)
    node = MotionController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C detected, stopping robot...")
        node.stop_robot()
    finally:
        # Destroy node and shutdown cleanly
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
