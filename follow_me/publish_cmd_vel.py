import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time


class LidarReaderAndMover(Node):
    def __init__(self):
        super().__init__('lidar_reader_and_mover')

        # Souscription au topic du LiDAR
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Souscription au topic d'odométrie
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publication des commandes de mouvement
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer pour envoyer régulièrement les commandes
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialisation des variables
        self.lidar_data = None
        self.current_position = None
        self.current_orientation = None
        self.previous_barycenter = None
        self.mode = "follow_me"  # Mode par défaut
        self.start_position = None  # Position de départ
        self.start_orientation = None  # Orientation de départ
        self.aligned_to_path = False  # Indique si le robot est aligné avec le chemin vers la position cible
        self.reached_position = False  # Indique si le robot a atteint la position cible
        self.last_movement_time = time.time()  # Suivi du temps pour le mode follow_me

        # Paramètres pour le mode follow_me
        self.K0 = 0.5  # Constante pour la vitesse linéaire
        self.K1 = 0.5  # Constante pour la vitesse angulaire
        self.target_distance = 1.0  # Distance cible (1.0 m)
        self.tolerance = 0.02  # Tolérance sur la position de l'objet
        self.stability_time = 15.0  # Temps en secondes avant de passer en go_home

        # Tolérances pour considérer le retour réussi (modifiables)
        self.position_tolerance = 0.2  # Tolérance de position en mètres
        self.angle_tolerance = math.radians(5)  # Tolérance d'angle en radians

    def lidar_callback(self, msg):
        self.lidar_data = msg

    def odom_callback(self, msg):
        # Récupérer la position actuelle
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        # Calculer l'orientation actuelle
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

        # Enregistrer la position et l'orientation de départ au démarrage
        if self.start_position is None:
            self.start_position = self.current_position
            self.start_orientation = self.current_orientation
            self.get_logger().info(f"Position de départ enregistrée : {self.start_position}, Orientation : {math.degrees(self.start_orientation):.2f}°")

    def normalize_angle(self, angle):
        """
        Normaliser un angle entre -pi et pi.
        """
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

            if r >= 0.5 and r <= 2:
                if angle_deg >= 345 or angle_deg <= 15:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append([x, y])

        if not points:
            return None, None

        x_sum = sum(p[0] for p in points)
        y_sum = sum(p[1] for p in points)

        x_avg = x_sum / len(points)
        y_avg = y_sum / len(points)

        self.get_logger().info(f"Barycentre visé : x = {x_avg:.2f}, y = {y_avg:.2f}")
        return x_avg, y_avg

    def follow_me(self, cmd):
        x_avg, y_avg = self.calculate_barycenter()
        if x_avg is None or y_avg is None:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return

        delta_x = x_avg - self.target_distance
        delta_y = math.atan2(y_avg, x_avg)

        if self.previous_barycenter:
            prev_x, prev_y = self.previous_barycenter
            if abs(x_avg - prev_x) < self.tolerance and abs(y_avg - prev_y) < self.tolerance:
                if time.time() - self.last_movement_time > self.stability_time:
                    self.get_logger().info("Object stable, switching to go_home mode.")
                    self.mode = "go_home"
            else:
                self.last_movement_time = time.time()

        self.previous_barycenter = (x_avg, y_avg)

        cmd.linear.x = delta_x * self.K0
        cmd.angular.z = delta_y * self.K1
        cmd.linear.x = max(-0.3, min(0.3, cmd.linear.x))
        cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))

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
            if abs(orientation_error) > self.angle_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
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
            self.get_logger().info(f"Alignement à l'orientation de départ : erreur = {math.degrees(orientation_error):.2f}°")
            return

        # Étape 4 : Retour validé
        self.get_logger().info(f"Retour validé ! Position de départ : {self.start_position}, Position actuelle : {self.current_position}")
        self.get_logger().info(f"Orientation de départ : {math.degrees(self.start_orientation):.2f}°, Orientation actuelle : {math.degrees(self.current_orientation):.2f}°")
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

        self.cmd_vel_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidarReaderAndMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
