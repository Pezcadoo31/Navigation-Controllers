#!/usr/bin/env python3
import rclpy, time, math, tf_transformations 
from rclpy import qos  
from rclpy.node import Node  
from nav_msgs.msg import Odometry  
from std_msgs.msg import Float32  
from geometry_msgs.msg import Twist 

# Clase principal del nodo de odometría
class ClassOdometry(Node):  
    def __init__(self):
        super().__init__("odometry")  
        
        # 1. Selección de modo (Turtlesim o Puzzlebot)
        self.declare_parameter('robot_mode', 'turtlesim')  # Declarar un parámetro para seleccionar el modo
        self.robot_mode = self.get_parameter('robot_mode').value  
        
        if self.robot_mode == 'turtlesim':
            # Configuración para el modo Turtlesim (simulación)
            self.get_logger().info("Modo: Turtlesim (simulación)")
            self.create_subscription(Twist, "/turtle1/cmd_vel", self.cmd_vel_cb, qos.qos_profile_sensor_data)
            self.r = 0.1  # Radio ficticio de las ruedas
            self.L = 0.5  # Distancia ficticia entre las ruedas
        else:
            # Configuración para el modo Puzzlebot (fisico)
            self.get_logger().info("Modo: Puzzlebot (robot real)")
            self.create_subscription(Float32, "/VelocityEncR", self.wR_cb, qos.qos_profile_sensor_data)
            self.create_subscription(Float32, "/VelocityEncL", self.wL_cb, qos.qos_profile_sensor_data)
            self.r = 0.05  # Radio real de las ruedas (en metros)
            self.L = 0.19  # Distancia real entre las ruedas (en metros)

        # Configuración común para ambos modos
        self.create_timer(0.1, self.odometry_callback)  
        self.pub = self.create_publisher(Odometry, "/odom", 1)  # Publicador para enviar los datos de odometría
        
        # Variables de estado
        self.wR = 0.0   # Velocidad angular de la rueda derecha
        self.wL = 0.0   # Velocidad angular de la rueda izquierda
        self.x = 0.0    # Posición en el eje X
        self.y = 0.0    # Posición en el eje Y
        self.q = 0.0    # Orientación (ángulo en radianes)
        self.t0 = self.get_clock().now()    # Tiempo inicial para medir intervalos
        self.last_valid_msg = None          # Último mensaje válido de odometría
        self.epsilon = 1e-5                 # Umbral para considerar velocidades como cero

    # Callback para recibir comandos de velocidad (modo Turtlesim)
    def cmd_vel_cb(self, msg):
        v = msg.linear.x  # Velocidad lineal
        w = msg.angular.z  # Velocidad angular

        # Calcular las velocidades angulares de las ruedas
        self.wR = (2 * v + w * self.L) / (2 * self.r)
        self.wL = (2 * v - w * self.L) / (2 * self.r)

    # Callback para recibir la velocidad angular de la rueda derecha (modo Puzzlebot)
    def wR_cb(self, msg):
        self.wR = msg.data
        self.get_logger().info(f"Encoder derecho: {self.wR:.2f} rad/s")  


    # Callback para recibir la velocidad angular de la rueda izquierda (modo Puzzlebot)
    def wL_cb(self, msg):
        self.wL = msg.data
        self.get_logger().info(f"Encoder izquierdo: {self.wL:.2f} rad/s")  

    # Callback para calcular y publicar la odometría
    def odometry_callback(self):
        # Calcular el tiempo transcurrido desde la última actualización
        elapsed_time = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.t0 = self.get_clock().now()  # Reiniciar el tiempo

        # Calcular velocidades lineal y angular
        v = (self.wR + self.wL) * self.r / 2.0      # Velocidad lineal
        w = (self.wR - self.wL) * self.r / self.L   # Velocidad angular

        # Actualizar la posición y orientación 
        self.x += v * math.cos(self.q) * elapsed_time
        self.y += v * math.sin(self.q) * elapsed_time
        self.q += w * elapsed_time

        # Normalizar el ángulo entre -pi y pi
        self.q = math.atan2(math.sin(self.q), math.cos(self.q))

        # Verificar si las velocidades son cercanas a cero
        if abs(v) < self.epsilon and abs(w) < self.epsilon:
            if self.last_valid_msg:
                self.pub.publish(self.last_valid_msg)  # Publicar el último mensaje válido
            return

        # Crear el mensaje de odometría
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()  # Tiempo actual
        msg.header.frame_id = "odom"  
        msg.child_frame_id = "base_footprint"  

        # Posición 
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Orientación en forma de cuaternión
        q = tf_transformations.quaternion_from_euler(0.0, 0.0, self.q)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # Velocidades lineal y angular
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        # Guardar el mensaje como el último válido y publicarlo
        self.last_valid_msg = msg
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)  
    node = ClassOdometry() 
    try:
        rclpy.spin(node)  
    except Exception as error:
        print(error)
    except KeyboardInterrupt:
        print("Node terminated by user!")
    finally:
        if rclpy.ok():
            node.destroy_node()  
            rclpy.shutdown() 

if __name__ == "__main__":
    main()