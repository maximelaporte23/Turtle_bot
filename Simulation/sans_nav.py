import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy


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
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("En attente des données d'odométrie...")
            return

        target_x, target_y = self.start_position

        # Calculer la distance et l'orientation cibles
        target_distance = math.sqrt(
            (target_x - self.current_position[0]) ** 2 +
            (target_y - self.current_position[1]) ** 2
        )
        target_angle = math.atan2(
            target_y - self.current_position[1],
            target_x - self.current_position[0]
        )
        orientation_error = self.normalize_angle(target_angle - self.current_orientation)

        # Étape 1 : Tourner pour s'aligner avec le chemin optimal
        if not self.aligned_to_path:
            if abs(orientation_error) > math.radians(0.2):
                cmd.linear.x = 0.0
                cmd.angular.z = 0.1 
                self.get_logger().info(f"Alignement au chemin : erreur = {math.degrees(orientation_error):.2f}°")
                return
            else:
                self.aligned_to_path = True
                self.get_logger().info("Alignement avec le chemin validé.")

        # Étape 2 : Avancer vers la position cible
        if not self.reached_position:
            if target_distance > self.position_tolerance:
                cmd.linear.x = min(0.2, target_distance)
                cmd.angular.z = 0.0
                self.get_logger().info(f"Déplacement vers la cible : distance restante = {target_distance:.2f} m")
                return
            else:
                self.reached_position = True
                self.get_logger().info("Position cible atteinte.")

        # Étape 3 : Tourner pour ajuster l'orientation de départ
        orientation_error = self.normalize_angle(self.start_orientation - self.current_orientation)

        if abs(orientation_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
            self.stable_orientation_time = None  # Réinitialiser le temps stable
            self.get_logger().info(f"Alignement à l'orientation de départ : erreur = {math.degrees(orientation_error):.2f}°")
            return
        else:
            # Orientation validée
            self.get_logger().info(f"Retour validé ! Position de départ : {self.start_position}, Position actuelle : {self.current_position}")
            self.get_logger().info(f"Orientation de départ : {math.degrees(self.start_orientation):.2f}°, Orientation actuelle : {math.degrees(self.current_orientation):.2f}°")
            
            # Forcer plusieurs publications pour s'assurer que le robot s'arrête
            for _ in range(5):  # Publier 5 fois
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_publisher.publish(cmd)
                time.sleep(0.1)  # Petite pause entre les publications
            
            # Réinitialiser les états pour le mode suivant
            self.mode = "dock"  # Repasser en mode dock après le retour
            self.aligned_to_path = False  # Réinitialiser pour les prochains retours
            self.reached_position = False  # Réinitialiser pour les prochains retours


    def dock(self, cmd):
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("En attente des données d'odométrie...")
            return

        target_x, target_y = self.dock_coordinates

        # Calculer la distance et l'orientation cibles
        target_distance = math.sqrt(
            (target_x - self.current_position[0]) ** 2 +
            (target_y - self.current_position[1]) ** 2
        )
        target_angle = math.atan2(
            target_y - self.current_position[1],
            target_x - self.current_position[0]
        )
        orientation_error = self.normalize_angle(target_angle - self.current_orientation)

        # Étape 1 : Tourner pour s'aligner avec le chemin optimal
        if not self.aligned_to_path:
            if abs(orientation_error) > math.radians(0.2):
                cmd.linear.x = 0.0
                cmd.angular.z = 0.1
                self.get_logger().info(f"Alignement au chemin : erreur = {math.degrees(orientation_error):.2f}°")
                return
            else:
                self.aligned_to_path = True
                self.get_logger().info("Alignement avec le chemin validé.")

        # Étape 2 : Avancer vers la position cible
        if not self.reached_position:
            if target_distance > self.position_tolerance:
                cmd.linear.x = min(0.2, target_distance)
                cmd.angular.z = 0.0
                self.get_logger().info(f"Déplacement vers la cible : distance restante = {target_distance:.2f} m")
                return
            else:
                self.reached_position = True
                self.get_logger().info("Position cible atteinte.")

        # Étape 3 : Tourner pour ajuster l'orientation de départ
        orientation_error = self.normalize_angle(self.dock_orientation - self.current_orientation)
        if abs(orientation_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
            self.get_logger().info(f"Alignement à l'orientation de départ : erreur = {math.degrees(orientation_error):.2f}°")
            return
        else:
            # Étape 4 : Retour validé
            self.get_logger().info(f"Retour validé !")
            self.get_logger().info(f"Orientation dock")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.mode = "follow_me"  # Repasser en mode follow_me après le retour
            self.aligned_to_path = False  # Réinitialiser pour les prochains retours
            self.reached_position = False  # Réinitialiser pour les prochains retours

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


if __name__ == '__main__':
    main()
