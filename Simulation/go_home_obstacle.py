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

        # Souscriptions aux topics
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer pour gérer les actions en boucle
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Initialisation des variables d'état
        self.lidar_data = None
        self.current_position = None
        self.current_orientation = None
        self.start_position = None
        self.start_orientation = None
        self.last_state_change_time = time.time()  # Temps du dernier changement d'état

        # Phases du robot
        self.mode = "follow_me"  # "follow_me", "go_home", ou "avoid_obstacle"
        self.human_stopped_time = None
        self.aligned_to_path = False
        self.avoiding_obstacle = False  # État d'évitement actif

        # Paramètres pour follow_me
        self.target_distance = 0.5  # Distance cible pour suivre l'humain (50 cm)
        self.min_distance = 0.2  # Distance minimale (20 cm)
        self.max_distance = 1.0  # Distance maximale (1 m)
        self.linear_speed = 0.3
        self.angular_speed = 1.0
        self.stability_duration = 3.0  # Temps d'arrêt avant de passer à la phase 2

        # Paramètres pour go_home
        self.obstacle_distance = 0.7  # Distance critique pour éviter les obstacles
        self.position_tolerance = 0.2  # Tolérance de position en mètres
        self.angle_tolerance = math.radians(10)  # Tolérance angulaire en radians

        # Validation des paramètres
        self.validate_parameters()
        self.get_logger().info("Initialisation terminée avec succès.")
        self.last_mode = None
        self.avoid_obstacle_start_time = None
        self.movement_threshold = 0.05  # Tolérance pour considérer que le robot est immobile
        self.timeout_threshold = 5.0  # Temps en secondes avant de passer en mode "go_home"

    def validate_parameters(self):
        """
        Valide les paramètres pour s'assurer qu'ils sont cohérents.
        """
        if not (0.1 <= self.target_distance <= 2.0):
            raise ValueError("La distance cible (target_distance) doit être comprise entre 0.1 et 2.0 m.")
        if not (0.1 <= self.obstacle_distance <= 1.0):
            raise ValueError("La distance critique pour obstacle (obstacle_distance) doit être comprise entre 0.1 et 1.0 m.")
        if not (0.0 <= self.linear_speed <= 1.0):
            raise ValueError("La vitesse linéaire (linear_speed) doit être comprise entre 0.0 et 1.0 m/s.")
        if not (0.0 <= self.angular_speed <= 2.0):
            raise ValueError("La vitesse angulaire (angular_speed) doit être comprise entre 0.0 et 2.0 rad/s.")
        self.get_logger().info("Paramètres validés.")

    def apply_speed_limits(self, cmd):
        """
        Applique des limites strictes sur les commandes linéaires et angulaires.
        """
        max_linear_speed = 0.3  # Limite supérieure pour la vitesse linéaire
        max_angular_speed = 1.0  # Limite supérieure pour la vitesse angulaire

        cmd.linear.x = max(-max_linear_speed, min(max_linear_speed, cmd.linear.x))
        cmd.angular.z = max(-max_angular_speed, min(max_angular_speed, cmd.angular.z))

        self.get_logger().debug(
            f"Vitesse limitée : linéaire={cmd.linear.x:.2f}, angulaire={cmd.angular.z:.2f}"
        )
        return cmd

    def lidar_callback(self, msg):
        self.lidar_data = msg
        self.get_logger().debug("LiDAR data mise à jour.")

    def odom_callback(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y ** 2 + orientation_q.z ** 2)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

        # Enregistrer la position de départ si nécessaire
        if self.start_position is None:
            self.start_position = self.current_position
            self.start_orientation = self.current_orientation
            self.get_logger().info(
                f"Position de départ enregistrée : {self.start_position}, "
                f"Orientation : {math.degrees(self.start_orientation):.2f}°"
            )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def calculate_barycenter(self):
        """
        Méthode pour suivre un humain.
        """
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

            if self.min_distance <= r <= self.max_distance and (angle_deg >= 330 or angle_deg <= 30):
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])

        if not points:
            return None, None

        x_avg = sum(p[0] for p in points) / len(points)
        y_avg = sum(p[1] for p in points) / len(points)

        return x_avg, y_avg

    def follow_me(self, cmd):
        """
        Suivre un humain en calculant un barycentre.
        """
        x_avg, y_avg = self.calculate_barycenter()

        if x_avg is None or y_avg is None:
            self.get_logger().info("Aucun humain détecté. Arrêt du robot.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.human_stopped_time = None
            return

        # Calcul de l'erreur de distance et d'angle
        distance_error = x_avg - self.target_distance
        angle_error = math.atan2(y_avg, x_avg)

        # Commandes proportionnelles
        cmd.linear.x = max(-self.linear_speed, min(self.linear_speed, distance_error * 0.5))
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, angle_error * 0.8))

        # Détecter si l'humain est immobile
        if abs(distance_error) < 0.05 and abs(angle_error) < math.radians(5):
            if self.human_stopped_time is None:
                self.human_stopped_time = time.time()
            elif time.time() - self.human_stopped_time >= self.stability_duration and self.can_change_state():
                self.get_logger().info("Humain immobile. Passage à la phase 2 : go_home.")
                self.mode = "go_home"
                self.last_state_change_time = time.time()
        else:
            self.human_stopped_time = None

    def detect_obstacle(self, min_angle, max_angle):
        if not self.lidar_data or not self.lidar_data.ranges:
            self.get_logger().warn("Aucune donnée LiDAR disponible.")
            return False

        ranges = self.lidar_data.ranges
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment

        # Conversion des angles en indices
        min_idx = int((math.radians(min_angle) - angle_min) / angle_increment)
        max_idx = int((math.radians(max_angle) - angle_min) / angle_increment)

        # Assurez-vous que les indices restent dans la plage valide
        min_idx = max(0, min_idx)
        max_idx = min(len(ranges) - 1, max_idx)

        # Parcourir les distances dans la plage spécifiée
        for i in range(min_idx, max_idx + 1):
            angle = math.degrees(angle_min + i * angle_increment)
            distance = ranges[i]

            # Ignorer les angles latéraux où les seuils sont détectés
            if 15 < abs(angle) < 30 and distance < self.obstacle_distance:
                continue  # Ignore ces lectures

            if distance < self.obstacle_distance:
                return True

        return False


    def recalculate_path(self, cmd):
        """
        Recalcule la trajectoire optimale vers le point de départ après un contournement.
        Limite les corrections angulaires à ±20°.
        """
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("En attente des données d'odométrie...")
            return

        target_x, target_y = self.start_position
        target_distance = math.sqrt(
            (target_x - self.current_position[0]) ** 2 +
            (target_y - self.current_position[1]) ** 2
        )
        target_angle = math.atan2(
            target_y - self.current_position[1],
            target_x - self.current_position[0]
        )
        orientation_error = self.normalize_angle(target_angle - self.current_orientation)

        self.get_logger().info(f"Recalcul de trajectoire : distance={target_distance:.2f} m, "
                                f"orientation={math.degrees(orientation_error):.2f}°")

        # Si la distance est petite, fin du déplacement
        if target_distance < self.position_tolerance:
            self.get_logger().info("Arrivé au point de départ. Retour en mode 'follow_me'.")
            self.mode = "follow_me"
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return

        # Détection d'obstacles dans la direction de la cible
        if self.detect_obstacle(min_angle=-15, max_angle=15):
            self.get_logger().warn("Obstacle détecté sur le chemin direct. Passage en mode 'avoid_obstacle'.")
            self.mode = "avoid_obstacle"
            return

        # Limiter la correction angulaire à ±20°
        if abs(orientation_error) > math.radians(20):
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
            return

        # Avancer si orienté correctement
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0

    def avoid_obstacle(self, cmd):
        if self.detect_doorway():
            self.get_logger().info("Seuil de porte détecté. Passage au mode 'go_home'.")
            self.mode = "go_home"
            return
        
        # Logique existante pour l'évitement des obstacles
        clear_path_angle = self.find_clear_path(min_angle=-20, max_angle=20)
        if clear_path_angle is not None:
            orientation_error = self.normalize_angle(clear_path_angle - self.current_orientation)

            if abs(orientation_error) > math.radians(5):  # Ajuster l'orientation
                cmd.linear.x = 0.0
                cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
            else:  # Avancer si orienté correctement
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0
        else:
            self.perform_small_rotation(cmd)
            
    def perform_small_rotation(self, cmd):
        """
        Effectue une rotation incrémentale de 20° pour explorer les directions.
        """
        if not hasattr(self, 'rotation_step'):
            self.rotation_step = 0  # Initialiser à la première rotation

        # Calculer l'angle cible pour la rotation en incréments de 20°
        angle_increment = math.radians(20)
        target_angle = self.normalize_angle(self.rotation_step * angle_increment + self.current_orientation)

        orientation_error = self.normalize_angle(target_angle - self.current_orientation)

        if abs(orientation_error) > math.radians(5):  # Continuer la rotation vers la cible
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
        else:  # Passer à l'incrément suivant une fois aligné
            self.rotation_step += 1
            self.get_logger().info(f"Rotation incrémentale terminée : {math.degrees(target_angle):.2f}°")

            # Réinitialiser après un tour complet (360°)
            if self.rotation_step >= 18:  # 18 étapes de 20° pour faire un tour complet
                self.rotation_step = 0
                self.get_logger().warn("Tour complet effectué, aucune direction dégagée trouvée.")


    def find_clear_path(self, min_angle=-120, max_angle=120):
        if not self.lidar_data:
            self.get_logger().info("Aucune donnée LiDAR disponible.")
            return None

        ranges = self.lidar_data.ranges
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment

        clear_zones = []
        current_zone = []

        for i, r in enumerate(ranges):
            angle = math.degrees(angle_min + i * angle_increment)
            if min_angle <= angle <= max_angle and r >= self.obstacle_distance:
                current_zone.append((i, angle))
            elif current_zone:
                clear_zones.append(current_zone)
                current_zone = []

        if current_zone:
            clear_zones.append(current_zone)

        clear_zones = [zone for zone in clear_zones if zone[-1][1] - zone[0][1] > 10]

        if clear_zones:
            # Priorité aux zones centrées autour de 0° (devant)
            clear_zones.sort(key=lambda z: abs((z[0][1] + z[-1][1]) / 2))
            largest_zone = clear_zones[0]

            start_angle = largest_zone[0][1]
            end_angle = largest_zone[-1][1]
            best_angle = (start_angle + end_angle) / 2
            self.get_logger().info(f"Zone dégagée trouvée : {start_angle:.2f}° à {end_angle:.2f}°.")
            return math.radians(best_angle)

        self.get_logger().info("Aucune zone dégagée trouvée.")
        return None
    
    def can_change_state(self):
        """
        Vérifie si suffisamment de temps s'est écoulé depuis le dernier changement d'état.
        """
        if time.time() - self.last_state_change_time > 2.0:
            self.last_state_change_time = time.time()
            return True
        return False
    
    def moving_to_clear_path(self, cmd):
        """
        Déplace le robot vers une direction dégagée.
        """
        if self.detect_obstacle(min_angle=-15, max_angle=15):  # Vérifier les obstacles en cours de mouvement
            self.get_logger().info("Obstacle détecté pendant le mouvement. Retour à 'go_home'.")
            self.mode = "go_home"
            return

        # Vérifier si la direction est toujours correcte
        orientation_error = self.normalize_angle(self.target_angle - self.current_orientation)
        if abs(orientation_error) > math.radians(5):  # Réorienter si nécessaire
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
        else:
            cmd.linear.x = 0.2
            cmd.angular.z = 0.0  # Avancer dans la direction dégagée

        # Vérifier la progression
        if time.time() - self.last_progress_time > 5.0:  # Pas de progrès après 5 secondes
            self.get_logger().warn("Aucun progrès détecté. Retour à 'go_home'.")
            self.mode = "go_home"
        else:
            self.last_progress_time = time.time()


    def go_home(self, cmd):
        """
        Logique pour retourner à la maison tout en évitant les obstacles.
        """
        if self.current_position is None or self.current_orientation is None:
            self.get_logger().info("En attente des données d'odométrie...")
            return

        # Vérification des obstacles directement devant
        if self.detect_obstacle(min_angle=-15, max_angle=15):
            self.get_logger().info("Obstacle détecté en face. Passage en mode 'avoid_obstacle'.")
            self.mode = "avoid_obstacle"
            return

        # Calcul de la distance et de l'orientation vers la position de départ
        target_x, target_y = self.start_position
        target_distance = math.sqrt(
            (target_x - self.current_position[0]) ** 2 +
            (target_y - self.current_position[1]) ** 2
        )
        target_angle = math.atan2(
            target_y - self.current_position[1],
            target_x - self.current_position[0]
        )
        orientation_error = self.normalize_angle(target_angle - self.current_orientation)

        self.get_logger().info(f"Distance à la cible : {target_distance:.2f} m, "
                            f"Erreur d'orientation : {math.degrees(orientation_error):.2f}°")

        # Si la distance est en dessous de la tolérance, retour terminé
        if target_distance < self.position_tolerance:
            self.get_logger().info("Retour au point de départ terminé. Passage en mode 'follow_me'.")
            self.mode = "follow_me"
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return

        # Ajustement de l'orientation vers le point de départ
        if abs(orientation_error) > self.angle_tolerance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3 if orientation_error > 0 else -0.3
            return

        # Avancer vers la cible
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0



    def moving_to_home(self, cmd):
        """
        Avancer vers le point de départ une fois orienté correctement.
        """
        if self.current_position is None or self.start_position is None:
            self.get_logger().info("En attente des données d'odométrie pour avancer vers la maison...")
            return

        # Vérification des obstacles en cours de mouvement
        if self.detect_obstacle(min_angle=-15, max_angle=15):
            self.get_logger().info("Obstacle détecté pendant le déplacement. Activation du recul.")
            cmd.linear.x = -0.2  # Reculer pour dégager l'obstacle
            cmd.angular.z = 0.0
            time.sleep(1.0)  # Attendre que le robot recule
            self.mode = "go_home"
            return

        # Calcul de la distance au point de départ
        target_x, target_y = self.start_position
        distance_to_target = math.sqrt(
            (target_x - self.current_position[0]) ** 2 +
            (target_y - self.current_position[1]) ** 2
        )
        
        # Vérifier si nous sommes proches de la cible
        if distance_to_target < self.position_tolerance:
            self.get_logger().info("Retour au point de départ terminé. Passage en mode 'follow_me'.")
            self.mode = "follow_me"
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            return

        # Avancer vers le point de départ
        cmd.linear.x = 0.2
        cmd.angular.z = 0.0
        self.get_logger().info(f"Avancement vers la maison : distance restante = {distance_to_target:.2f} m")
        
    

    def timer_callback(self):
        cmd = Twist()
        self.get_logger().info(f"Mode actuel : {self.mode}")

        if self.mode == "follow_me":
            self.follow_me(cmd)
        elif self.mode == "go_home":
            self.go_home(cmd)
        elif self.mode == "avoid_obstacle":
            if self.detect_doorway():
                self.get_logger().info("Seuil détecté. Forçage au mode 'go_home'.")
                self.mode = "go_home"
                return
            self.avoid_obstacle(cmd)
        elif self.mode == "recalculate_path":
            self.recalculate_path(cmd)

        # Vérification si le robot dépasse la porte sans changer de mode
        if self.current_position and self.start_position:
            distance_from_home = math.sqrt(
                (self.current_position[0] - self.start_position[0]) ** 2 +
                (self.current_position[1] - self.start_position[1]) ** 2
            )
            if distance_from_home > 3.0:  # Seuil de dépassement (par exemple, 3 m)
                self.get_logger().warn("Le robot semble avoir dépassé la maison. Recalcul de la trajectoire.")
                self.mode = "recalculate_path"

        cmd = self.apply_speed_limits(cmd)
        self.cmd_vel_publisher.publish(cmd)

    def detect_doorway(self):
        """
        Détecte un seuil de porte basé sur les données LiDAR.
        Cherche des zones dégagées étroites centrées devant le robot.
        """
        if not self.lidar_data:
            self.get_logger().info("Aucune donnée LiDAR disponible.")
            return False

        ranges = self.lidar_data.ranges
        angle_min = self.lidar_data.angle_min
        angle_increment = self.lidar_data.angle_increment

        # Recherche des zones dégagées devant le robot (angles proches de 0°)
        min_idx = int((math.radians(-15) - angle_min) / angle_increment)
        max_idx = int((math.radians(15) - angle_min) / angle_increment)

        # Limiter la plage d'indices pour éviter les erreurs
        min_idx = max(0, min_idx)
        max_idx = min(len(ranges) - 1, max_idx)

        # Vérifier la largeur de la zone dégagée
        door_width = sum(1 for i in range(min_idx, max_idx) if self.min_distance <= ranges[i] <= self.obstacle_distance)

        # Considérer une zone comme un seuil si elle est suffisamment étroite
        is_doorway = 0.5 <= door_width * self.lidar_data.angle_increment <= 1.0  # Largeur de la porte en mètres
        if is_doorway:
            self.get_logger().info(f"Seuil détecté avec une largeur de {door_width * self.lidar_data.angle_increment:.2f} m.")
        return is_doorway



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
