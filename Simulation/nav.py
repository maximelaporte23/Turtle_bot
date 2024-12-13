import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped


class LidarReaderAndMover(Node):
    def __init__(self):
        super().__init__('lidar_reader_and_mover')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Souscriptions
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        self.amcl_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        # Publication
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer pour l'envoi des commandes
        self.timer = self.create_timer(0.05, self.timer_callback)

        # Variables
        self.lidar_data = None
        self.current_position = None
        self.current_orientation = None
        self.previous_barycenter = None
        self.mode = "follow_me"
        self.start_position = None
        self.start_orientation = None
        self.aligned_to_path = False
        self.reached_position = False
        self.navigator = BasicNavigator()
        self.result = self.navigator.getResult()
        self.last_movement_time = time.time()

        # Paramètres follow_me
        self.K0 = 1
        self.K1 = 2
        self.target_distance = 0.6
        self.tolerance = 0.02
        self.stability_time = 10.0

        # Tolérances go_home et dock
        self.position_tolerance = 0.08
        self.angle_tolerance = math.radians(5)

        # Dernière commande valide
        self.last_valid_cmd = Twist()

        # Coordonnées pour le mode dock
        self.dock_coordinates = (6.4, -1.3)
        self.dock_orientation = math.radians(5)

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def amcl_pose_callback(self, msg):
        pose = msg.pose.pose
        self.current_position = (pose.position.x, pose.position.y)

        orientation_q = pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

        if self.start_position is None:
            self.start_position = self.current_position
            self.start_orientation = self.current_orientation
            self.get_logger().info(f"Position de départ : {self.start_position}, Orientation : {math.degrees(self.start_orientation):.2f}°")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_barycenter(self):
        if not self.lidar_data:
            return None, None

        ranges = self.lidar_data.ranges
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment

        points = []
        for i in range(len(ranges)):
            r = ranges[i]
            angle = angle_min + i * angle_increment
            angle_deg = math.degrees(angle)

            if 0.2 <= r <= 1.0 and (angle_deg >= 348 or angle_deg <= 12):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])

        if not points:
            return None, None

        x_avg = sum(p[0] for p in points) / len(points)
        y_avg = sum(p[1] for p in points) / len(points)

        self.get_logger().info(f"Barycentre : x = {x_avg:.2f}, y = {y_avg:.2f}")
        return x_avg, y_avg

    def follow_me(self, cmd):
        x_avg, y_avg = self.calculate_barycenter()
        if x_avg is None or y_avg is None:
            cmd.linear.x = self.last_valid_cmd.linear.x
            cmd.angular.z = self.last_valid_cmd.angular.z
            return

        delta_x = x_avg - self.target_distance
        delta_y = math.atan2(y_avg, x_avg)

        cmd.linear.x = delta_x * self.K0
        cmd.angular.z = delta_y * self.K1

        cmd.linear.x = max(-1, min(1, cmd.linear.x))
        cmd.angular.z = max(-3, min(3, cmd.angular.z))

        self.last_valid_cmd = cmd

        if self.previous_barycenter:
            prev_x, prev_y = self.previous_barycenter
            if abs(x_avg - prev_x) < self.tolerance and abs(y_avg - prev_y) < self.tolerance:
                if time.time() - self.last_movement_time > self.stability_time:
                    self.get_logger().info("Object stable, switching to go_home mode.")
                    self.mode = "go_home"
                    return
            else:
                self.last_movement_time = time.time()

        self.previous_barycenter = (x_avg, y_avg)

    def go_home(self, cmd):
        # Use the BasicNavigator to go to the home position
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        target_x, target_y = self.start_position
        goal_pose.pose.position.x = target_x  # Set home position
        goal_pose.pose.position.y = target_y  # Set home position
        goal_pose.pose.orientation.z = self.start_orientation 

        # Move to the home position using BasicNavigator
        self.navigator.goToPose(goal_pose)

        # Once task is complete, switch to docking mode
        if self.result == TaskResult.SUCCEEDED or self.result == TaskResult.CANCELED or self.result == TaskResult.FAILED:  # TaskResult.SUCCEEDED
            self.mode = "dock"

    def dock(self, cmd):
        # Use BasicNavigator to go to docking position
        dock_pose = PoseStamped()
        dock_pose.header.frame_id = 'map'
        dock_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        dock_pose.pose.position.x = 7.63  # Dock position
        dock_pose.pose.position.y = -6.87  # Dock position
        dock_pose.pose.orientation.z = 1.0

        # Move to the dock position using BasicNavigator
        self.navigator.goToPose(dock_pose)

        # Once task is complete, return to follow_me mode
        if self.result == TaskResult.SUCCEEDED:  # TaskResult.SUCCEEDED
            self.mode = "follow_me"

    def timer_callback(self):
        cmd = Twist()
        if self.mode == "follow_me":
            self.follow_me(cmd)
        elif self.mode == "go_home":
            self.go_home(cmd)
        elif self.mode == "dock":
            self.dock(cmd)

        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarReaderAndMover()
    rclpy.spin(node)
    rclpy.shutdown()
    rclpy.init()


if __name__ == '__main__':
    main()

