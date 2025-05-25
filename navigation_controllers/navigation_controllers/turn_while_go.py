#!/usr/bin/env python3
import rclpy, math, tf_transformations, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class TurnWhileGoController(Node):
    def __init__(self):
        super().__init__('turn_while_go_controller')
        
        # Publicadores
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.point_reached_pub = self.create_publisher(Bool, 'point_reached', 10)
        
        # Suscripciones 
        self.create_subscription(Pose, 'next_point', self.next_point_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.real_pose_callback, 10)
        
        # Timers
        self.timer = self.create_timer(0.1, self.control_loop)  
        self.comparison_timer = self.create_timer(0.1, self.check_midpoint)  # Chequear punto medio
        
        # Parámetros de control 
        self.k_angular  = 0.2           # Ganancia para velocidad angular
        self.k_linear   = 0.1502        # Ganancia para velocidad lineal
        self.distance_threshold = 0.01  # Distancia (metros) para considerar punto alcanzado
        self.slowdown_distance = 0.05   # Distancia para comenzar a reducir velocidad 
        self.max_linear_speed = 0.2     # Velocidad lineal máxima
        self.max_angular_speed = 0.5    # Velocidad angular máxima
        
        # Variables de estado
        self.odom_x = 0.0  # Odometría
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.real_x = 5.544  # Pose real
        self.real_y = 5.544
        self.real_theta = 0.0

        self.target_x = None  # Objetivo
        self.target_y = None
        self.initial_x = 5.544  # Punto de partida
        self.initial_y = 5.544
        self.state = 'IDLE'  # IDLE, MOVING
        
        self.point_count = 0
        self.total_points = None
        self.midpoint_reported = False
        
        # Variable para registrar el tiempo
        self.start_time = None
        
        self.declare_parameter('invert_coordinates', False)
        self.invert_coordinates = self.get_parameter('invert_coordinates').value

    def log_pose_comparison(self, trigger_event):
        if self.target_x is None or self.target_y is None:
            return
            
        # Calcular diferencias REAL vs ODOM
        x_diff_real_odom = self.real_x - self.odom_x
        y_diff_real_odom = self.real_y - self.odom_y
        ang_diff_real_odom = self.real_theta - self.odom_theta
        
        # Calcular diferencias ODOM vs TARGET
        x_diff_odom_target = self.odom_x - self.target_x
        y_diff_odom_target = self.odom_y - self.target_y
        
        # Calcular tiempo transcurrido si es el final del movimiento
        tiempo_info = ""
        if trigger_event == "end" and self.start_time is not None:
            elapsed_time = time.time() - self.start_time
            tiempo_info = f"\n\n=== TIEMPO DE LLEGADA ===\nTiempo total: {elapsed_time:.2f} segundos"
        
        # Mostrar comparación
        self.get_logger().info(
            f"\n=== COMPARACIÓN ({trigger_event.upper()}) PUNTO {self.point_count}/{self.total_points if self.total_points else '?'} ==="
            "\n[REAL] ".ljust(15) + f"X={self.real_x:.3f}, Y={self.real_y:.3f}, θ={self.real_theta:.3f}"
            "\n[ODOM] ".ljust(15) + f"X={self.odom_x:.3f}, Y={self.odom_y:.3f}, θ={self.odom_theta:.3f}"
            "\n[TARGET] ".ljust(15) + f"X={self.target_x:.3f}, Y={self.target_y:.3f}"
            
            "\n\n=== DIFERENCIAS ==="
            "\nREAL vs ODOM: ".ljust(20) + 
            f"X_diff={x_diff_real_odom:.3f}, Y_diff={y_diff_real_odom:.3f}, θ_diff={ang_diff_real_odom:.3f}"
            
            "\nODOM vs TARGET: ".ljust(20) + 
            f"X_diff={x_diff_odom_target:.3f}, Y_diff={y_diff_odom_target:.3f}"
            + tiempo_info +
            "\n" + "="*40
        )

    def check_midpoint(self):
        if self.state != 'MOVING' or self.target_x is None:
            return
            
        total_distance = math.sqrt((self.target_x - self.initial_x)**2 + 
                                 (self.target_y - self.initial_y)**2)
        current_distance = math.sqrt((self.odom_x - self.initial_x)**2 + 
                                  (self.odom_y - self.initial_y)**2)
        progress = current_distance / total_distance if total_distance > 0 else 0
        
        if 0.4 < progress < 0.6 and not self.midpoint_reported:
            self.log_pose_comparison("mid")
            self.midpoint_reported = True

    def real_pose_callback(self, msg):
        self.real_x = msg.x
        self.real_y = msg.y
        self.real_theta = msg.theta
        
    def next_point_callback(self, msg):
        # Guardar posición inicial
        self.initial_x = self.odom_x
        self.initial_y = self.odom_y
        self.midpoint_reported = False
        
        # Iniciar el temporizador
        self.start_time = time.time()
        
        if self.invert_coordinates:
            self.target_x = msg.y  
            self.target_y = msg.x  
        else:
            self.target_x = msg.x
            self.target_y = msg.y
            
        self.state = 'MOVING'  
        self.point_count += 1
        
        if self.total_points is None:
            self.total_points = float('inf')  # Número desconocido de puntos
            
        self.log_pose_comparison("start")
        self.get_logger().info(f'Iniciando movimiento a punto {self.point_count}: ({self.target_x}, {self.target_y})')
        
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.odom_theta = tf_transformations.euler_from_quaternion(orientation_list)
        
    def control_loop(self):
        if self.target_x is None or self.target_y is None or self.state != 'MOVING':
            return
            
        # Calcular distancia al objetivo
        dx = self.target_x - self.odom_x
        dy = self.target_y - self.odom_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calcular ángulo hacia el objetivo
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.odom_theta
        
        # Normalizar el error de ángulo a [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Preparar comando de velocidad
        cmd = Twist()
        
        # ¿Llegamos al objetivo?
        if distance < self.distance_threshold:
            # Detenerlo
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            
            # Publicar que se alcanzó el punto
            point_reached_msg = Bool()
            point_reached_msg.data = True
            self.point_reached_pub.publish(point_reached_msg)
            
            # Mostrar comparación final
            self.log_pose_comparison("end")
            self.state = 'IDLE'
            
            # Si es el último punto, cancelar el timer de comparaciones
            if self.point_count == self.total_points:
                self.comparison_timer.cancel()
                self.get_logger().info('¡Último punto alcanzado! Finalizando comparaciones.')
            return
        
        # Cálculo de velocidad lineal con factor de reducción basado en el error angular
        # Reducir velocidad lineal cuando el error angular es grande
        angular_factor = max(0.0, 1.0 - abs(angle_error) / math.pi)
        
        # También reducir velocidad cuando se esta cerca del objetivo
        distance_factor = min(1.0, distance / self.slowdown_distance)
        
        # Combinar ambos factores para la velocidad lineal
        linear_speed = self.k_linear * distance * angular_factor * distance_factor
        
        # Limitar la velocidad lineal máxima
        linear_speed = min(linear_speed, self.max_linear_speed)
        
        # Calcular velocidad angular
        angular_speed = self.k_angular * angle_error
        
        # Limitar la velocidad angular máxima
        if angular_speed > self.max_angular_speed:
            angular_speed = self.max_angular_speed
        elif angular_speed < -self.max_angular_speed:
            angular_speed = -self.max_angular_speed
        
        # Asignar velocidades al mensaje Twist
        cmd.linear.x = linear_speed
        cmd.angular.z = angular_speed
        
        # Publicar comando de velocidad
        self.cmd_vel_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TurnWhileGoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    except Exception as error:
        print(error)
    finally:
        if rclpy.ok():
            stop_cmd = Twist()
            node.cmd_vel_pub.publish(stop_cmd)
            node.destroy_node()
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()