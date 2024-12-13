import rclpy  # Bibliothèque pour ROS 2
from rclpy.node import Node  # Permet de créer un nœud ROS 2
from sensor_msgs.msg import LaserScan  # Messages pour les données du LiDAR
from nav_msgs.msg import Odometry  # Messages pour la position via odométrie
from geometry_msgs.msg import Twist  # Messages pour contrôler le mouvement du robot
import math  # Bibliothèque mathématique
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class DockingWithCoordinates(Node):
    def __init__(self):
        # Initialisation du nœud avec un nom
        super().__init__('docking_with_coordinates')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Souscriptions
        # Souscription au LiDAR pour détecter les obstacles
        self.lidar_subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, qos_profile)
        
        # Souscription aux données d'odométrie pour connaître la position
        self.odom_subscription = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Publication pour commander le mouvement du robot
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer pour exécuter la boucle principale toutes les 0.1 secondes
        self.timer = self.create_timer(0.1, self.control_loop)

        # Variables pour stocker les données
        self.lidar_data = None  # Contiendra les données du LiDAR
        self.robot_position = [0.0, 0.0]  # Position actuelle du robot (x, y)
        self.robot_orientation = 0.0  # Orientation actuelle du robot (en radians)

        # Coordonnées de la base cible
        self.base_coordinates = (6.92146, -7.83828)

        # Paramètres de sécurité et de tolérance
        self.safety_distance = 0.8  # Distance minimale avant un obstacle
        self.tolerance_to_base = 0.02  # Tolérance pour considérer que le robot est arrivé (2cm)
        self.detection_angle_range = 30  # Angle de détection d'obstacle (±15°)

    def lidar_callback(self, msg):
        """
        Stocke les données du LiDAR à chaque mise à jour.
        """
        self.lidar_data = msg

    def odom_callback(self, msg):
        """
        Met à jour la position et l'orientation actuelle du robot.
        """
        # Mettre à jour la position x, y
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y

        # Calculer l'orientation (yaw) à partir du quaternion
        orientation_q = msg.pose.pose.orientation
        yaw = self.calculate_yaw_from_quaternion(
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.robot_orientation = yaw

    def calculate_yaw_from_quaternion(self, x, y, z, w):
        """
        Convertit un quaternion en angle yaw (orientation sur le plan 2D). Les données d'orientation provenant de l'odométrie sont sous forme de quaternions. Le code les convertit en angle yaw pour faciliter les calculs directionnels.
        """
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)
        return yaw

    def calculate_distance_and_angle_to_base(self):
        """
        Calcule la distance et l'angle vers la base cible.
        """
        # Coordonnées actuelles du robot
        robot_x, robot_y = self.robot_position

        # Coordonnées de la base
        base_x, base_y = self.base_coordinates

        # Calculer la différence en x et y
        dx = base_x - robot_x
        dy = base_y - robot_y

        # Distance entre le robot et la base
        distance = math.sqrt(dx**2 + dy**2)

        # Angle vers la base
        angle_to_base = math.atan2(dy, dx)

        return distance, angle_to_base

    def control_loop(self):
        """
        Fonction principale exécutée périodiquement.
        """
        # Vérifie si le LiDAR est prêt
        if self.lidar_data is None:
            self.get_logger().info("En attente des données du LiDAR...")
            return

        # Calcule la distance et l'angle vers la base
        distance_to_base, angle_to_base = self.calculate_distance_and_angle_to_base()
        print(f"[INFO] Distance à la base : {distance_to_base:.3f} m, Angle à la base : {math.degrees(angle_to_base):.2f}°")

        # Vérifie si le robot est arrivé à la base
        if distance_to_base < self.tolerance_to_base:
            self.get_logger().info("Robot arrivé à la base.")
            self.stop_robot()
            print(f"[DEBUG] Position actuelle : {self.robot_position}, Orientation : {math.degrees(self.robot_orientation):.2f}°")
            return

        # Vérifie si un obstacle est détecté
        obstacle_detected, obstacle_angle = self.detect_obstacle()
        

        if obstacle_detected:
            # Si un obstacle est détecté, éviter l'obstacle
            self.avoid_obstacle(obstacle_angle)
        else:
            # Sinon, avancer vers la base
            self.move_towards_base(distance_to_base, angle_to_base)

    def detect_obstacle(self):
        """
        Vérifie si un obstacle se trouve dans la plage de détection.
        """
        # Plage d'angle pour détecter les obstacles
        angle_min = -math.radians(self.detection_angle_range / 2)
        angle_max = math.radians(self.detection_angle_range / 2)

        # Vérifie chaque distance dans les données du LiDAR
        for i, distance in enumerate(self.lidar_data.ranges):
            angle = self.lidar_data.angle_min + i * self.lidar_data.angle_increment
            if angle_min <= angle <= angle_max and distance < self.safety_distance:
                return True, angle  # Obstacle détecté

        return False, None  # Aucun obstacle détecté

    def move_towards_base(self, distance_to_base, angle_to_base):
        """
        Dirige le robot vers la base cible.
        """
        cmd = Twist()  # Crée une commande de mouvement

        # Vitesse linéaire pour avancer vers la base
        #cmd.linear.x = 0.3
        if distance_to_base > 0.5:
            cmd.linear.x = 0.3  # Vitesse normale
        elif 0.01 <= distance_to_base <= 0.5:
            cmd.linear.x = 0.05
        else:
            cmd.linear.x = 0.05


        # Différence d'angle entre l'orientation du robot et l'angle vers la base
        angle_difference = angle_to_base - self.robot_orientation

        # Ajuste l'orientation
        cmd.angular.z = 0.5 * math.atan2(math.sin(angle_difference), math.cos(angle_difference))

        # Publie la commande de mouvement
        self.cmd_vel_publisher.publish(cmd)

    def avoid_obstacle(self, obstacle_angle):
        """
        Évite un obstacle détecté en ajustant la direction du robot.
        """
        cmd = Twist()  # Crée une commande de mouvement

        # Tourne dans la direction opposée à l'obstacle
        if obstacle_angle < 0:
            cmd.angular.z = 0.5  # Tourne à gauche
        else:
            cmd.angular.z = -0.5  # Tourne à droite

        # Publie la commande pour éviter l'obstacle
        self.cmd_vel_publisher.publish(cmd)

    def stop_robot(self):
        """
        Arrête le robot.
        """
        cmd = Twist()  # Crée une commande vide (arrêt)
        self.cmd_vel_publisher.publish(cmd)  # Publie la commande d'arrêt

def main(args=None):
    """
    Fonction principale pour démarrer le nœud.
    """
    rclpy.init(args=args)  # Initialise ROS 2
    node = DockingWithCoordinates()  # Crée une instance du nœud
    try:
        rclpy.spin(node)  # Lance le nœud (écoute et exécute)
    except KeyboardInterrupt:
        pass  # Permet d'arrêter proprement avec Ctrl+C
    finally:
        node.destroy_node()  # Détruit le nœud
        rclpy.shutdown()  # Arrête ROS 2

# Point d'entrée du programme
if __name__ == '__main__':
    main()
