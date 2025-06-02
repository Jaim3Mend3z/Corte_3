#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
from std_msgs.msg import Float64MultiArray
import time

class PyBulletKukaNode(Node):
    def __init__(self):
        super().__init__('pybullet_kuka_simulator')
        
        # --- Configuración de PyBullet ---
        self.get_logger().info('Iniciando simulación en PyBullet...')
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        
        start_pos = [0, 0, 0]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.kuka_id = p.loadURDF("kuka_iiwa/model.urdf", start_pos, start_orientation, useFixedBase=1)
        self.get_logger().info('Brazo KUKA cargado en PyBullet.')

        # --- Configuración de ROS 2 ---
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/kuka_joint_commands',
            self.command_callback,
            10)
        
        self.get_logger().info("Nodo de simulación listo. Esperando comandos...")
        
        # --- Bucle de Simulación ---
        self.timer = self.create_timer(1./240., self.step_simulation)

    def command_callback(self, msg):
        target_positions = msg.data
        for i in range(7):
            p.setJointMotorControl2(
                bodyIndex=self.kuka_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target_positions[i]
            )

    def step_simulation(self):
        p.stepSimulation()

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletKukaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
