# -- Experimentación Base --
#!/usr/bin/env python3
import rclpy, math, tf_transformations, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')

        # Log initialization
        self.get_logger().info('Fuzzy controller initialized!')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.point_reached_pub = self.create_publisher(Bool, 'point_reached', 10)
        
        # Subscriptions
        self.create_subscription(Pose, 'next_point', self.next_point_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.real_pose_callback, 10)
        
        # Timers
        self.timer = self.create_timer(0.1, self.control_loop)
        self.comparison_timer = self.create_timer(0.1, self.check_midpoint)  # Timer para chequeo de punto medio
        
        # Control parameters
        self.distance_threshold = 0.01  # Distance threshold (meters)
        
        # State variables
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
        
        # Fuzzy controller
        self.setup_fuzzy_controller()

    def setup_fuzzy_controller(self):
        # Fuzzy variables 
        position_error = ctrl.Antecedent(np.arange(0, 1.5, 0.01), 'position_error')
        orientation_error = ctrl.Antecedent(np.arange(-1.5708, 1.5708, 0.01), 'orientation_error')
        linear_vel = ctrl.Consequent(np.arange(0, 0.20, 0.01), 'linear_vel')
        angular_vel = ctrl.Consequent(np.arange(-0.5, 0.50, 0.01), 'angular_vel')
        
        # Funciones de membresía para position error 
        position_error['NO_HAY'] = fuzz.zmf(position_error.universe, 0.01, 0.05)
        position_error['POCO'] = fuzz.trimf(position_error.universe, [0.03, 0.1, 0.3])
        position_error['MUCHO'] = fuzz.smf(position_error.universe, 0.2, 0.5)
        
        # Funciones de membresía para orientation error 
        orientation_error['MUY_NEGATIVO'] = fuzz.zmf(orientation_error.universe, -1.2, -0.6)
        orientation_error['POCO_NEGATIVO'] = fuzz.trimf(orientation_error.universe, [-0.8, -0.4, -0.05])
        orientation_error['NULO'] = fuzz.trimf(orientation_error.universe, [-0.08, 0, 0.08])
        orientation_error['POCO_POSITIVO'] = fuzz.trimf(orientation_error.universe, [0.05, 0.4, 0.8])
        orientation_error['MUY_POSITIVO'] = fuzz.smf(orientation_error.universe, 0.6, 1.2)
        
        # Funciones de membresía para velocidad lineal
        linear_vel['ALTO'] = fuzz.trimf(linear_vel.universe, [0, 0, 0.04])
        linear_vel['AVANZA'] = fuzz.trimf(linear_vel.universe, [0.03, 0.1, 0.17])
        linear_vel['MAXIMA'] = fuzz.trimf(linear_vel.universe, [0.15, 0.2, 0.2])
        
        # Funciones de membresía para velocidad angular
        angular_vel['MAXIMO_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.5, -0.5, -0.35])
        angular_vel['MIN_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.35, -0.25, -0.15])
        angular_vel['STOP'] = fuzz.trimf(angular_vel.universe, [-0.10, 0, 0.10])
        angular_vel['MIN_DER'] = fuzz.trimf(angular_vel.universe, [0.15, 0.25, 0.35])
        angular_vel['MAX_DER'] = fuzz.trimf(angular_vel.universe, [0.35, 0.5, 0.5])
        
        # Reglas para el control de velocidad angular (giro)
        rule1 = ctrl.Rule(orientation_error['MUY_POSITIVO'], angular_vel['MAX_DER'])
        rule2 = ctrl.Rule(orientation_error['MUY_NEGATIVO'], angular_vel['MAXIMO_IZQ'])
        rule3 = ctrl.Rule(orientation_error['POCO_NEGATIVO'], angular_vel['MIN_IZQ'])
        rule4 = ctrl.Rule(orientation_error['POCO_POSITIVO'], angular_vel['MIN_DER'])
        rule5 = ctrl.Rule(orientation_error['NULO'], angular_vel['STOP'])
        
        # Reglas para el control de velocidad lineal (avance)
        rule6 = ctrl.Rule(position_error['NO_HAY'] & orientation_error['NULO'], linear_vel['ALTO'])
        rule7 = ctrl.Rule(position_error['POCO'] & orientation_error['NULO'], linear_vel['AVANZA'])
        rule8 = ctrl.Rule(position_error['MUCHO'] & orientation_error['NULO'], linear_vel['MAXIMA'])
        
        # Reglas para avance con errores de orientación
        rule9 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule10 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        rule11 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule12 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        
        # Reglas para mejorar el comportamiento
        rule13 = ctrl.Rule(position_error['NO_HAY'] & (orientation_error['POCO_POSITIVO'] | orientation_error['POCO_NEGATIVO']), linear_vel['ALTO'])
        rule14 = ctrl.Rule(position_error['MUCHO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])
        rule15 = ctrl.Rule(position_error['POCO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])
        
        # Control system
        control_system = ctrl.ControlSystem([
            rule1, rule2, rule3, rule4, rule5, 
            rule6, rule7, rule8, rule9, rule10, 
            rule11, rule12, rule13, rule14, rule15
        ])
        
        # Control system simulator
        self.fuzzy_system = ctrl.ControlSystemSimulation(control_system)

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
        position_error = math.sqrt(dx**2 + dy**2)
        
        # Calcular error de orientación
        target_angle = math.atan2(dy, dx)
        orientation_error = target_angle - self.odom_theta
        orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))
        
        try:
            self.fuzzy_system.input['position_error'] = min(position_error, 1.49)  
            self.fuzzy_system.input['orientation_error'] = max(min(orientation_error, 1.57), -1.57)  
            
            self.fuzzy_system.compute()
            
            linear_vel = self.fuzzy_system.output['linear_vel']
            angular_vel = self.fuzzy_system.output['angular_vel']
            
            # Adaptación de velocidad dependiendo de la distancia al objetivo
            if position_error < 0.15:
                # Reducir gradualmente la velocidad al acercarse al objetivo
                linear_vel *= (position_error / 0.15)
                       
            # Crear comando de velocidad
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            
            # Verificar si se ha alcanzado el objetivo
            if position_error < self.distance_threshold:
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
                
        except Exception as e:
            self.get_logger().error(f'Error en el controlador difuso: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    fuzzy_controller = FuzzyController()
    
    try:
        rclpy.spin(fuzzy_controller)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    except Exception as error:
        print(error)
    finally:
        if rclpy.ok():
            stop_cmd = Twist()
            fuzzy_controller.cmd_vel_pub.publish(stop_cmd)
            fuzzy_controller.destroy_node()
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()

'''
# -- Experimentación 4 --
#!/usr/bin/env python3
import rclpy, math, tf_transformations, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.point_reached_pub = self.create_publisher(Bool, 'point_reached', 10)
        
        # Subscriptions
        self.create_subscription(Pose, 'next_point', self.next_point_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.real_pose_callback, 10)
        
        # Timers
        self.timer = self.create_timer(0.1, self.control_loop)
        self.comparison_timer = self.create_timer(0.1, self.check_midpoint)  # Timer para chequeo de punto medio
        
        # Control parameters
        self.distance_threshold = 0.01  # Distance threshold (meters)
        
        # State variables
        self.odom_x = 0.0  # Odometría
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.real_x = 5.544  # Pose real
        self.real_y = 5.544
        self.real_theta = 0.0

        self.target_x = None  # Objetivo
        self.target_y = None
        self.initial_x = 5.5  # Punto de partida
        self.initial_y = 5.5
        self.state = 'IDLE'  # IDLE, MOVING
        
        self.point_count = 0
        self.total_points = None
        self.midpoint_reported = False
        
        # Variable para registrar el tiempo
        self.start_time = None
        
        # Declarar parámetro para controlar la interpretación de coordenadas
        self.declare_parameter('invert_coordinates', False)
        self.invert_coordinates = self.get_parameter('invert_coordinates').value
        
        # Fuzzy controller
        self.setup_fuzzy_controller()
        
        # Log initialization
        self.get_logger().info('Fuzzy controller initialized!')

    def setup_fuzzy_controller(self):
        # Fuzzy variables - Mantenemos los rangos que proporcionaste
        position_error = ctrl.Antecedent(np.arange(0, 1.5, 0.01), 'position_error')
        orientation_error = ctrl.Antecedent(np.arange(-1.5708, 1.5708, 0.01), 'orientation_error')
        linear_vel = ctrl.Consequent(np.arange(0, 0.20, 0.01), 'linear_vel')
        angular_vel = ctrl.Consequent(np.arange(-0.5, 0.50, 0.01), 'angular_vel')
        
        # Membership functions for position error 
        position_error['NO_HAY'] = fuzz.zmf(position_error.universe, 0.01, 0.05)
        position_error['POCO'] = fuzz.trimf(position_error.universe, [0.03, 0.1, 0.3])
        position_error['MUCHO'] = fuzz.smf(position_error.universe, 0.2, 0.5)
        
        # Membership functions for orientation error - Mejores transiciones
        orientation_error['MUY_NEGATIVO'] = fuzz.zmf(orientation_error.universe, -1.2, -0.6)
        orientation_error['POCO_NEGATIVO'] = fuzz.trimf(orientation_error.universe, [-0.8, -0.4, -0.05])
        orientation_error['NULO'] = fuzz.trimf(orientation_error.universe, [-0.08, 0, 0.08])
        orientation_error['POCO_POSITIVO'] = fuzz.trimf(orientation_error.universe, [0.05, 0.4, 0.8])
        orientation_error['MUY_POSITIVO'] = fuzz.smf(orientation_error.universe, 0.6, 1.2)
        
        # Funciones de membresía mejoradas para velocidad lineal
        # Manteniendo los puntos centrales en 0, 0.1 y 0.2 pero con mejores rangos
        linear_vel['ALTO'] = fuzz.trimf(linear_vel.universe, [0, 0, 0.04])
        linear_vel['AVANZA'] = fuzz.trimf(linear_vel.universe, [0.03, 0.1, 0.17])
        linear_vel['MAXIMA'] = fuzz.trimf(linear_vel.universe, [0.15, 0.2, 0.2])
        
        # Funciones de membresía mejoradas para velocidad angular
        # Manteniendo los puntos centrales en -0.5, -0.25, 0, 0.25, 0.5 pero con mejores rangos
        angular_vel['MAXIMO_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.5, -0.5, -0.35])
        angular_vel['MIN_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.35, -0.25, -0.15])
        angular_vel['STOP'] = fuzz.trimf(angular_vel.universe, [-0.10, 0, 0.10])
        angular_vel['MIN_DER'] = fuzz.trimf(angular_vel.universe, [0.15, 0.25, 0.35])
        angular_vel['MAX_DER'] = fuzz.trimf(angular_vel.universe, [0.35, 0.5, 0.5])
        
        # Reglas mejoradas para permitir avance y giro simultáneos con transiciones más suaves
        
        # Reglas para el control de velocidad angular (giro)
        rule1 = ctrl.Rule(orientation_error['MUY_POSITIVO'], angular_vel['MAX_DER'])
        rule2 = ctrl.Rule(orientation_error['MUY_NEGATIVO'], angular_vel['MAXIMO_IZQ'])
        rule3 = ctrl.Rule(orientation_error['POCO_NEGATIVO'], angular_vel['MIN_IZQ'])
        rule4 = ctrl.Rule(orientation_error['POCO_POSITIVO'], angular_vel['MIN_DER'])
        rule5 = ctrl.Rule(orientation_error['NULO'], angular_vel['STOP'])
        
        # Reglas para el control de velocidad lineal (avance)
        # Relación más progresiva entre error de posición y velocidad
        #rule6 = ctrl.Rule(position_error['NO_HAY'] & orientation_error['NULO'], linear_vel['ALTO'])
        rule6 = ctrl.Rule(position_error['NO_HAY'], (linear_vel['ALTO'], angular_vel['STOP']))
        rule7 = ctrl.Rule(position_error['POCO'] & orientation_error['NULO'], linear_vel['AVANZA'])
        rule8 = ctrl.Rule(position_error['MUCHO'] & orientation_error['NULO'], linear_vel['MAXIMA'])
        
        # Reglas para avance con errores de orientación
        rule9 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule10 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        rule11 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule12 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        
        # Reglas para mejorar el comportamiento
        #rule13 = ctrl.Rule(position_error['NO_HAY'] & (orientation_error['POCO_POSITIVO'] | orientation_error['POCO_NEGATIVO']), linear_vel['ALTO'])
        rule13 = ctrl.Rule(position_error['NO_HAY'] & orientation_error['NULO'], (linear_vel['ALTO'], angular_vel['STOP']))
        rule14 = ctrl.Rule(position_error['MUCHO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])
        rule15 = ctrl.Rule(position_error['POCO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])
        
        # Control system
        control_system = ctrl.ControlSystem([
            rule1, rule2, rule3, rule4, rule5, 
            rule6, rule7, rule8, rule9, rule10, 
            rule11, rule12, rule13, rule14, rule15
        ])
        
        # Control system simulator
        self.fuzzy_system = ctrl.ControlSystemSimulation(control_system)

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
        
        # Calculate position error (distance to target)
        dx = self.target_x - self.odom_x
        dy = self.target_y - self.odom_y
        position_error = math.sqrt(dx**2 + dy**2)
        
        # Calculate orientation error
        target_angle = math.atan2(dy, dx)
        orientation_error = target_angle - self.odom_theta
        orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))
        
        try:
            # Input values to fuzzy system
            self.fuzzy_system.input['position_error'] = min(position_error, 1.49)  # Clip to range
            self.fuzzy_system.input['orientation_error'] = max(min(orientation_error, 1.57), -1.57)  # Clip to range
            
            # Compute fuzzy output
            self.fuzzy_system.compute()
            
            # Get control outputs
            linear_vel = self.fuzzy_system.output['linear_vel']
            angular_vel = self.fuzzy_system.output['angular_vel']
            
            # Adaptación dinámica de velocidad dependiendo de la distancia al objetivo
            if position_error < 0.15:
                # Reducir gradualmente la velocidad al acercarse al objetivo
                linear_vel *= (position_error / 0.15)
                       
            # Create and publish Twist message
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            
            # Check if target is reached
            if position_error < self.distance_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                
                # Publish point reached message
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
                
        except Exception as e:
            self.get_logger().error(f'Error en el controlador difuso: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    fuzzy_controller = FuzzyController()
    
    try:
        rclpy.spin(fuzzy_controller)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    except Exception as error:
        print(error)
    finally:
        if rclpy.ok():
            stop_cmd = Twist()
            fuzzy_controller.cmd_vel_pub.publish(stop_cmd)
            fuzzy_controller.destroy_node()
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
'''

'''
# -- Experimentación 3 --
#!/usr/bin/env python3
import rclpy, math, tf_transformations, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.point_reached_pub = self.create_publisher(Bool, 'point_reached', 10)
        
        # Subscriptions
        self.create_subscription(Pose, 'next_point', self.next_point_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.real_pose_callback, 10)
        
        # Timers
        self.timer = self.create_timer(0.1, self.control_loop)
        self.comparison_timer = self.create_timer(0.1, self.check_midpoint)  # Timer para chequeo de punto medio
        
        # Control parameters
        self.distance_threshold = 0.01  # Distance threshold (meters)
        
        # State variables
        self.odom_x = 0.0  # Odometría
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.real_x = 5.544  # Pose real
        self.real_y = 5.544
        self.real_theta = 0.0

        self.target_x = None  # Objetivo
        self.target_y = None
        self.initial_x = 5.5  # Punto de partida
        self.initial_y = 5.5
        self.state = 'IDLE'  # IDLE, MOVING
        
        self.point_count = 0
        self.total_points = None
        self.midpoint_reported = False
        
        # Variable para registrar el tiempo
        self.start_time = None
        
        # Declarar parámetro para controlar la interpretación de coordenadas
        self.declare_parameter('invert_coordinates', False)
        self.invert_coordinates = self.get_parameter('invert_coordinates').value
        
        # Fuzzy controller
        self.setup_fuzzy_controller()
        
        # Log initialization
        self.get_logger().info('Fuzzy controller initialized!')

    def setup_fuzzy_controller(self):
        # Fuzzy variables - Mantenemos los rangos que proporcionaste
        position_error = ctrl.Antecedent(np.arange(0, 1.5, 0.01), 'position_error')
        orientation_error = ctrl.Antecedent(np.arange(-1.5708, 1.5708, 0.01), 'orientation_error')
        linear_vel = ctrl.Consequent(np.arange(0, 0.20, 0.01), 'linear_vel')
        angular_vel = ctrl.Consequent(np.arange(-0.5, 0.50, 0.01), 'angular_vel')
        
        # Membership functions for position error 
        position_error['NO_HAY'] = fuzz.zmf(position_error.universe, 0.01, 0.05)
        position_error['POCO'] = fuzz.trimf(position_error.universe, [0.03, 0.1, 0.3])
        position_error['MUCHO'] = fuzz.smf(position_error.universe, 0.2, 0.5)
        
        # Membership functions for orientation error - Mejores transiciones
        orientation_error['MUY_NEGATIVO'] = fuzz.zmf(orientation_error.universe, -1.2, -0.6)
        orientation_error['POCO_NEGATIVO'] = fuzz.trimf(orientation_error.universe, [-0.8, -0.4, -0.05])
        orientation_error['NULO'] = fuzz.trimf(orientation_error.universe, [-0.08, 0, 0.08])
        orientation_error['POCO_POSITIVO'] = fuzz.trimf(orientation_error.universe, [0.05, 0.4, 0.8])
        orientation_error['MUY_POSITIVO'] = fuzz.smf(orientation_error.universe, 0.6, 1.2)
        
        # Funciones de membresía mejoradas para velocidad lineal
        # Manteniendo los puntos centrales en 0, 0.1 y 0.2 pero con mejores rangos
        linear_vel['ALTO'] = fuzz.trimf(linear_vel.universe, [0, 0, 0.04])
        linear_vel['AVANZA'] = fuzz.trimf(linear_vel.universe, [0.03, 0.1, 0.17])
        linear_vel['MAXIMA'] = fuzz.trimf(linear_vel.universe, [0.15, 0.2, 0.2])
        
        # Funciones de membresía mejoradas para velocidad angular
        # Manteniendo los puntos centrales en -0.5, -0.25, 0, 0.25, 0.5 pero con mejores rangos
        angular_vel['MAXIMO_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.5, -0.5, -0.35]) 
        angular_vel['MIN_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.35, -0.25, -0.15])
        angular_vel['MUY_POCO_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.2, -0.1, -0.05]) 
        angular_vel['STOP'] = fuzz.trimf(angular_vel.universe, [-0.10, 0, 0.10])
        angular_vel['MUY_POCO_DER'] = fuzz.trimf(angular_vel.universe, [0.05, 0.1, 0.2]) 
        angular_vel['MIN_DER'] = fuzz.trimf(angular_vel.universe, [0.15, 0.25, 0.35])
        angular_vel['MAX_DER'] = fuzz.trimf(angular_vel.universe, [0.35, 0.5, 0.5])
        
        # Reglas mejoradas para permitir avance y giro simultáneos con transiciones más suaves
        
        # Reglas para el control de velocidad angular (giro)
        rule1 = ctrl.Rule(orientation_error['MUY_POSITIVO'], angular_vel['MAX_DER'])
        rule2 = ctrl.Rule(orientation_error['MUY_NEGATIVO'], angular_vel['MAXIMO_IZQ'])
        rule3 = ctrl.Rule(orientation_error['POCO_NEGATIVO'], angular_vel['MIN_IZQ'])
        rule4 = ctrl.Rule(orientation_error['POCO_POSITIVO'], angular_vel['MIN_DER'])
        rule5 = ctrl.Rule(orientation_error['NULO'], angular_vel['STOP'])
        
        # Reglas para el control de velocidad lineal (avance)
        # Relación más progresiva entre error de posición y velocidad
        rule6 = ctrl.Rule(position_error['NO_HAY'] & orientation_error['NULO'], linear_vel['ALTO'])
        rule7 = ctrl.Rule(position_error['POCO'] & orientation_error['NULO'], linear_vel['AVANZA'])
        rule8 = ctrl.Rule(position_error['MUCHO'] & orientation_error['NULO'], linear_vel['MAXIMA'])
        
        # Reglas para avance con errores de orientación
        rule9 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule10 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        rule11 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule12 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        
        # Reglas para mejorar el comportamiento
        rule13 = ctrl.Rule(position_error['NO_HAY'] & (orientation_error['POCO_POSITIVO'] | orientation_error['POCO_NEGATIVO']), linear_vel['ALTO'])
        rule14 = ctrl.Rule(position_error['MUCHO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])
        rule15 = ctrl.Rule(position_error['POCO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])

        # Reglas nuevas para los nuevos conjuntos de velocidad angular
        rule16 = ctrl.Rule(orientation_error['POCO_NEGATIVO'] & position_error['POCO'], angular_vel['MUY_POCO_IZQ'])
        rule17 = ctrl.Rule(orientation_error['POCO_POSITIVO'] & position_error['POCO'], angular_vel['MUY_POCO_DER'])
        
        # Control system
        control_system = ctrl.ControlSystem([
            rule1, rule2, rule3, rule4, rule5, 
            rule6, rule7, rule8, rule9, rule10, 
            rule11, rule12, rule13, rule14, rule15
        ])
        
        # Control system simulator
        self.fuzzy_system = ctrl.ControlSystemSimulation(control_system)

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
        
        # Calculate position error (distance to target)
        dx = self.target_x - self.odom_x
        dy = self.target_y - self.odom_y
        position_error = math.sqrt(dx**2 + dy**2)
        
        # Calculate orientation error
        target_angle = math.atan2(dy, dx)
        orientation_error = target_angle - self.odom_theta
        orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))
        
        try:
            # Input values to fuzzy system
            self.fuzzy_system.input['position_error'] = min(position_error, 1.49)  # Clip to range
            self.fuzzy_system.input['orientation_error'] = max(min(orientation_error, 1.57), -1.57)  # Clip to range
            
            # Compute fuzzy output
            self.fuzzy_system.compute()
            
            # Get control outputs
            linear_vel = self.fuzzy_system.output['linear_vel']
            angular_vel = self.fuzzy_system.output['angular_vel']
            
            # Adaptación dinámica de velocidad dependiendo de la distancia al objetivo
            if position_error < 0.15:
                # Reducir gradualmente la velocidad al acercarse al objetivo
                linear_vel *= (position_error / 0.15)
                       
            # Create and publish Twist message
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            
            # Check if target is reached
            if position_error < self.distance_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                
                # Publish point reached message
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
                
        except Exception as e:
            self.get_logger().error(f'Error en el controlador difuso: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    fuzzy_controller = FuzzyController()
    
    try:
        rclpy.spin(fuzzy_controller)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    except Exception as error:
        print(error)
    finally:
        if rclpy.ok():
            stop_cmd = Twist()
            fuzzy_controller.cmd_vel_pub.publish(stop_cmd)
            fuzzy_controller.destroy_node()
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
'''

'''
# -- Experimentación 2 --
#!/usr/bin/env python3
import rclpy, math, tf_transformations, time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from std_msgs.msg import Bool
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyController(Node):
    def __init__(self):
        super().__init__('fuzzy_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.point_reached_pub = self.create_publisher(Bool, 'point_reached', 10)
        
        # Subscriptions
        self.create_subscription(Pose, 'next_point', self.next_point_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.real_pose_callback, 10)
        
        # Timers
        self.timer = self.create_timer(0.1, self.control_loop)
        self.comparison_timer = self.create_timer(0.1, self.check_midpoint)  # Timer para chequeo de punto medio
        
        # Control parameters
        self.distance_threshold = 0.01  # Distance threshold (meters)
        
        # State variables
        self.odom_x = 0.0  # Odometría
        self.odom_y = 0.0
        self.odom_theta = 0.0

        self.real_x = 5.544  # Pose real
        self.real_y = 5.544
        self.real_theta = 0.0

        self.target_x = None  # Objetivo
        self.target_y = None
        self.initial_x = 5.5  # Punto de partida
        self.initial_y = 5.5
        self.state = 'IDLE'  # IDLE, MOVING
        
        self.point_count = 0
        self.total_points = None
        self.midpoint_reported = False
        
        # Variable para registrar el tiempo
        self.start_time = None
        
        # Declarar parámetro para controlar la interpretación de coordenadas
        self.declare_parameter('invert_coordinates', False)
        self.invert_coordinates = self.get_parameter('invert_coordinates').value
        
        # Fuzzy controller
        self.setup_fuzzy_controller()
        
        # Log initialization
        self.get_logger().info('Fuzzy controller initialized!')

    def setup_fuzzy_controller(self):
        # Fuzzy variables - Mantenemos los rangos que proporcionaste
        position_error = ctrl.Antecedent(np.arange(0, 3.0, 0.01), 'position_error')  # Duplicar el rango máximo
        orientation_error = ctrl.Antecedent(np.arange(-1.5708, 1.5708, 0.01), 'orientation_error')
        linear_vel = ctrl.Consequent(np.arange(0, 0.20, 0.01), 'linear_vel')
        angular_vel = ctrl.Consequent(np.arange(-0.5, 0.50, 0.01), 'angular_vel')
        
        # Membership functions for position error 
        position_error['NO_HAY'] = fuzz.zmf(position_error.universe, 0.01, 0.1)  # Más amplio
        position_error['POCO'] = fuzz.trimf(position_error.universe, [0.05, 0.5, 1.0])  # Rango extendido
        position_error['MUCHO'] = fuzz.smf(position_error.universe, 0.8, 2.0)          # Más gradual
        
        # Membership functions for orientation error - Mejores transiciones
        orientation_error['MUY_NEGATIVO'] = fuzz.zmf(orientation_error.universe, -1.2, -0.6)
        orientation_error['POCO_NEGATIVO'] = fuzz.trimf(orientation_error.universe, [-0.8, -0.4, -0.05])
        orientation_error['NULO'] = fuzz.trimf(orientation_error.universe, [-0.08, 0, 0.08])
        orientation_error['POCO_POSITIVO'] = fuzz.trimf(orientation_error.universe, [0.05, 0.4, 0.8])
        orientation_error['MUY_POSITIVO'] = fuzz.smf(orientation_error.universe, 0.6, 1.2)
        
        # Funciones de membresía mejoradas para velocidad lineal
        # Manteniendo los puntos centrales en 0, 0.1 y 0.2 pero con mejores rangos
        linear_vel['ALTO'] = fuzz.trimf(linear_vel.universe, [0, 0, 0.04])
        linear_vel['AVANZA'] = fuzz.trimf(linear_vel.universe, [0.03, 0.1, 0.17])
        linear_vel['MAXIMA'] = fuzz.trimf(linear_vel.universe, [0.15, 0.2, 0.2])
        
        # Funciones de membresía mejoradas para velocidad angular
        # Manteniendo los puntos centrales en -0.5, -0.25, 0, 0.25, 0.5 pero con mejores rangos
        angular_vel['MAXIMO_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.5, -0.5, -0.35])
        angular_vel['MIN_IZQ'] = fuzz.trimf(angular_vel.universe, [-0.35, -0.25, -0.15])
        angular_vel['STOP'] = fuzz.trimf(angular_vel.universe, [-0.10, 0, 0.10])
        angular_vel['MIN_DER'] = fuzz.trimf(angular_vel.universe, [0.15, 0.25, 0.35])
        angular_vel['MAX_DER'] = fuzz.trimf(angular_vel.universe, [0.35, 0.5, 0.5])
        
        # Reglas mejoradas para permitir avance y giro simultáneos con transiciones más suaves
        
        # Reglas para el control de velocidad angular (giro)
        rule1 = ctrl.Rule(orientation_error['MUY_POSITIVO'], angular_vel['MAX_DER'])
        rule2 = ctrl.Rule(orientation_error['MUY_NEGATIVO'], angular_vel['MAXIMO_IZQ'])
        rule3 = ctrl.Rule(orientation_error['POCO_NEGATIVO'], angular_vel['MIN_IZQ'])
        rule4 = ctrl.Rule(orientation_error['POCO_POSITIVO'], angular_vel['MIN_DER'])
        rule5 = ctrl.Rule(orientation_error['NULO'], angular_vel['STOP'])
        
        # Reglas para el control de velocidad lineal (avance)
        # Relación más progresiva entre error de posición y velocidad
        rule6 = ctrl.Rule(position_error['NO_HAY'] & orientation_error['NULO'], linear_vel['ALTO'])
        rule7 = ctrl.Rule(position_error['POCO'] & orientation_error['NULO'], linear_vel['AVANZA'])
        rule8 = ctrl.Rule(position_error['MUCHO'] & orientation_error['NULO'], linear_vel['MAXIMA'])
        
        # Reglas para avance con errores de orientación
        rule9 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule10 = ctrl.Rule(position_error['POCO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        rule11 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_POSITIVO'], linear_vel['AVANZA'])
        rule12 = ctrl.Rule(position_error['MUCHO'] & orientation_error['POCO_NEGATIVO'], linear_vel['AVANZA'])
        
        # Reglas para mejorar el comportamiento
        rule13 = ctrl.Rule(position_error['NO_HAY'] & (orientation_error['POCO_POSITIVO'] | orientation_error['POCO_NEGATIVO']), linear_vel['ALTO'])
        rule14 = ctrl.Rule(position_error['MUCHO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])
        rule15 = ctrl.Rule(position_error['POCO'] & (orientation_error['MUY_POSITIVO'] | orientation_error['MUY_NEGATIVO']), linear_vel['ALTO'])
        
        # Control system
        control_system = ctrl.ControlSystem([
            rule1, rule2, rule3, rule4, rule5, 
            rule6, rule7, rule8, rule9, rule10, 
            rule11, rule12, rule13, rule14, rule15
        ])
        
        # Control system simulator
        self.fuzzy_system = ctrl.ControlSystemSimulation(control_system)

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
        
        # Calculate position error (distance to target)
        dx = self.target_x - self.odom_x
        dy = self.target_y - self.odom_y
        position_error = math.sqrt(dx**2 + dy**2)
        
        # Calculate orientation error
        target_angle = math.atan2(dy, dx)
        orientation_error = target_angle - self.odom_theta
        orientation_error = math.atan2(math.sin(orientation_error), math.cos(orientation_error))
        
        try:
            # Input values to fuzzy system
            self.fuzzy_system.input['position_error'] = min(position_error, 1.49)  # Clip to range
            self.fuzzy_system.input['orientation_error'] = max(min(orientation_error, 1.57), -1.57)  # Clip to range
            
            # Compute fuzzy output
            self.fuzzy_system.compute()
            
            # Get control outputs
            linear_vel = self.fuzzy_system.output['linear_vel']
            angular_vel = self.fuzzy_system.output['angular_vel']
            
            # Adaptación dinámica de velocidad dependiendo de la distancia al objetivo
            if position_error < 0.15:
                # Reducir gradualmente la velocidad al acercarse al objetivo
                linear_vel *= (position_error / 0.15)
                       
            # Create and publish Twist message
            cmd = Twist()
            cmd.linear.x = linear_vel
            cmd.angular.z = angular_vel
            self.cmd_vel_pub.publish(cmd)
            
            # Check if target is reached
            if position_error < self.distance_threshold:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                
                # Publish point reached message
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
                
        except Exception as e:
            self.get_logger().error(f'Error en el controlador difuso: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    fuzzy_controller = FuzzyController()
    
    try:
        rclpy.spin(fuzzy_controller)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    except Exception as error:
        print(error)
    finally:
        if rclpy.ok():
            stop_cmd = Twist()
            fuzzy_controller.cmd_vel_pub.publish(stop_cmd)
            fuzzy_controller.destroy_node()
            rclpy.shutdown()
        
if __name__ == '__main__':
    main()
'''
 