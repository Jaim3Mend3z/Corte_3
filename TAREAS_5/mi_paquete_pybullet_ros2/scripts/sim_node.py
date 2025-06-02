#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
from std_msgs.msg import Float64MultiArray
import time
# --- NUEVAS IMPORTACIONES DE QOS ---
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# --- PERFIL DE QOS DEFINIDO (DEBE SER COMPATIBLE CON EL PUBLICADOR) ---
qos_profile_comandos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

class PyBulletKukaNode(Node):
    def __init__(self):
        super().__init__('pybullet_kuka_simulator')

        self.get_logger().info('Iniciando simulación en PyBullet...')
        try:
            self.physics_client = p.connect(p.GUI)
        except p.error as e:
            self.get_logger().error(f"No se pudo conectar al servidor GUI de PyBullet: {e}")
            self.get_logger().info("Intentando conectar en modo DIRECT (sin GUI)...")
            try:
                self.physics_client = p.connect(p.DIRECT)
                self.get_logger().info("Conectado a PyBullet en modo DIRECT.")
            except p.error as e_direct:
                self.get_logger().fatal(f"No se pudo conectar a PyBullet en modo DIRECT tampoco: {e_direct}")
                rclpy.shutdown()
                return

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        kuka_start_pos = [0, 0, 0]
        kuka_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.kuka_id = p.loadURDF("kuka_iiwa/model.urdf", kuka_start_pos, kuka_start_orientation, useFixedBase=1)
        self.get_logger().info('Brazo KUKA cargado en PyBullet.')

        # --- USAR EL PERFIL DE QOS ---
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/kuka_joint_commands',
            self.command_callback,
            qos_profile_comandos # <--- CAMBIO AQUÍ
        )

        self.get_logger().info("Nodo de simulación listo. Esperando comandos en /kuka_joint_commands...")
        self.timer = self.create_timer(1./240., self.step_simulation_callback)

    def command_callback(self, msg):
        self.get_logger().info(f"SimNode: Comando de articulación recibido: {list(msg.data)}")
        target_positions = msg.data
        if len(target_positions) == 7:
            for i in range(7):
                p.setJointMotorControl2(
                    bodyIndex=self.kuka_id,
                    jointIndex=i, 
                    controlMode=p.POSITION_CONTROL,
                    targetPosition=target_positions[i],
                    force=200.0, 
                    positionGain=0.03,
                    velocityGain=0.5  
                )
        else:
            self.get_logger().warn(f"SimNode: Comando recibido con longitud incorrecta ({len(target_positions)}), se esperaban 7.")

    def step_simulation_callback(self):
        if p.isConnected(self.physics_client):
            p.stepSimulation()
        else:
            self.get_logger().warn("PyBullet no está conectado, saltando stepSimulation.")

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletKukaNode()
    if p.isConnected(node.physics_client):
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Cerrando por KeyboardInterrupt')
        finally:
            if p.isConnected(node.physics_client): p.disconnect(node.physics_client)
            node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    else:
        node.get_logger().error("No se pudo inicializar PyBullet. Terminando nodo.")
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
