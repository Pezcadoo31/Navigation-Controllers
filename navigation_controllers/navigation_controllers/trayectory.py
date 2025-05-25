#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from std_msgs.msg import Bool 

class PathGeneratorClass(Node):
    def __init__(self):
        super().__init__("path_generator")
        self.get_logger().info("Path Generator node has been started ...")
        
        # Publicador para los puntos objetivos
        self.pub = self.create_publisher(Pose, "next_point", 1)
        # Suscriptor para señal de punto alcanzado
        self.sub = self.create_subscription(Bool, "point_reached", self.point_reached_callback, 10)
        
        self.msg = Pose()
        self.point_index = 0 # Índice del punto actual
        self.waiting_for_completion = False
        
        # Lista de puntos (x, y) 
        self.point_list = [
            # Puntos dentro de los cuadrantes cartesianos
            # Primer Cuadrante 
            #[1.0, 3.0],
            #[3.0, 1.0], 
            #[4.0, 4.0],

            # Segundo Cuadrante
            #[-1.0, 3.0],
            #[-3.0, 1.0], 
            #[-4.0, 4.0],

            # Tercer Cuadrante
            #[-1.0, -3.0],
            #[-3.0, -1.0],
            #[-4.0, -4.0],

            # Cuarto Cuadrante
            #[1.0, -3.0],
            #[3.0, -1.0],
            #[4.0, -4.0],    

            # Figuras geométricas
            # Cuadrado
            #[1.0, 0.0],
            #[1.0, 1.0],
            #[0.0, 1.0],
            #[0.0, 0.0],
            
            # Rombo
            #[0.5, -0.5],
            #[0.0, -1.0],
            #[-0.5, -0.5],
            #[0.0, 0.0],
            
            # Romboide
            #[1.0, 0.0],
            #[1.5, 1.0],
            #[0.5, 1.0],
            #[0.0, 0.0],
            
            # Triángulo
            #[1.0, 0.0],
            #[0.0, 0.5],
            #[-1.0, 0.0],
            #[0.0, 0.0],

            # Trapecio
            #[1.0, 0.0],
            #[1.5, -1.0],
            #[-0.5, -1.0],
            #[0.0, 0.0],

            # Diamante
            [1.0, 0.0],
            [1.5, -0.5],
            [0.5, -2.0],
            [-0.5, -0.5],
            [0.0, 0.0],
        ]
        
        # Enviar el primer punto automáticamente después de iniciar
        self.init_timer = self.create_timer(1.0, self.send_first_point)
    
    def send_first_point(self):
        # Cancelar el timer después de la primera ejecución
        self.init_timer.cancel()
        if len(self.point_list) > 0:
            self.send_next_point()
        else:
            self.get_logger().warn("La lista de puntos está vacía")
    
    def point_reached_callback(self, msg):
        if msg.data and self.waiting_for_completion:
            self.get_logger().info(f"Punto {self.point_index} alcanzado!")
            self.waiting_for_completion = False
            self.point_index += 1 # Avanzar al siguiente punto
            
            # Enviar el siguiente punto o terminar
            if self.point_index < len(self.point_list):
                self.send_next_point()
            else:
                self.get_logger().info("Todos los puntos han sido alcanzados. Terminando...")
                self.destroy_node()
                rclpy.shutdown()
    
    def send_next_point(self):
        if self.point_index < len(self.point_list):
            x, y = self.point_list[self.point_index]
            self.msg.x = x
            self.msg.y = y
            self.pub.publish(self.msg)
            self.get_logger().info(f"Enviando punto {self.point_index}: ({x}, {y})")
            self.waiting_for_completion = True
        else:
            self.get_logger().warn("No hay más puntos para enviar")

def main(args=None):
    rclpy.init(args=args)
    node = PathGeneratorClass()
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
