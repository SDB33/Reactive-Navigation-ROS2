#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan

import numpy as np

class MyNode(Node):
    def __init__(self):
        super().__init__("conmovedor")

        self.postura = None
        self.objectiu = None
        self.raig = None
        self.forsaRE = None
        self.forsaAT = None

        self.emMoc = self.create_publisher(Twist, "/cmd_vel", 10)  # Publicacion para mover el robot
        self.vull_posicio = self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.onEstic, 10)  # Suscripcion a la posicion del robot
        self.vull_posicio_objetctiu = self.create_subscription(PoseStamped, "/goal_pose", self.onEstare, 10)  # Suscripcion a la posicion objetivo del robot
        self.basto_de_cec = self.create_subscription(LaserScan, "/laser1", self.noXoquis, 10)  # Suscripcion al laser frontal del robot

        # Crear el bucle
        self.create_timer(0.05, self.Anella)

    def onEstic(self, encarrec: PoseWithCovarianceStamped): self.postura = encarrec

    def onEstare(self, encarrec: PoseStamped): self.objectiu = encarrec

    def noXoquis(self, encarrec: LaserScan): self.raig = encarrec

    def donamGiransa(self, orientacio):
        x = orientacio.x
        y = orientacio.y
        z = orientacio.z
        w = orientacio.w
        return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

    def Anella(self):
        # Al principio no hay una posicion objetivo, hay que elegirla manualmente
        if self.objectiu is None:
            self.get_logger().info("Quin error de cinc palets, no tinc objectiu!")
            return

        # Verificación de llegada al destino y cálculo de la fuerza atractiva
        self.forsaAT = np.array([
            self.objectiu.pose.position.x - self.postura.pose.pose.position.x,
            self.objectiu.pose.position.y - self.postura.pose.pose.position.y
        ])
        modulo = np.linalg.norm(self.forsaAT)
        if modulo < 3:
            return
        else:
            self.forsaAT = self.forsaAT / modulo

        # Crear un mensaje y siempre avanzar hacia el objetivo
        mensaje = Twist()
        mensaje.linear.x = 1.0

        # Calculo de la fuerza repulsiva
        self.forsaRE = np.array([0.0, 0.0])  # Inicializar fuerza repulsiva en cero

        direccion_objetivo = np.arctan2(self.forsaAT[1], self.forsaAT[0])

        for i, distancia_obstaculo in enumerate(self.raig.ranges):
            if distancia_obstaculo < 3:

                angulo = self.raig.angle_min + i * self.raig.angle_increment

                # Calcular el peso basado principalmente en el ángulo hacia el objetivo
                peso = np.exp(10 * np.cos(angulo - direccion_objetivo))  # Obstáculos alineados tienen peso exponencialmente mayor
                peso = max(0, peso)  # Ignorar obstáculos fuera de la dirección del objetivo

                self.forsaRE += -np.array([
                    np.cos(angulo),
                    np.sin(angulo)
                ]) * peso

        # Sumar fuerzas atractiva y repulsiva
        forsa_total = 0.3 * self.forsaAT + self.forsaRE  # Dar mucho más peso a la repulsión

        angulo_fuerza_total = np.arctan2(forsa_total[1], forsa_total[0])

        theta_robot = self.donamGiransa(self.postura.pose.pose.orientation)
        delta_theta = (angulo_fuerza_total - theta_robot + np.pi) % (2 * np.pi) - np.pi

        # Ajustar velocidad angular en función de la desviación angular
        if abs(delta_theta) < 0.2:
            mensaje.angular.z = 0.0
        else:
            mensaje.angular.z = 0.9 * np.sign(delta_theta)

        self.get_logger().info("Calculando fuerzas")
        self.get_logger().info(f"Fuerza atractiva: {self.forsaAT}")
        self.get_logger().info(f"Fuerza repulsiva: {self.forsaRE}")

        self.emMoc.publish(mensaje)


def main(args=None):
    rclpy.init(args=args)
    nodo = MyNode()
    rclpy.spin(nodo)
    rclpy.shutdown()


if __name__ == "__main__":
    main()