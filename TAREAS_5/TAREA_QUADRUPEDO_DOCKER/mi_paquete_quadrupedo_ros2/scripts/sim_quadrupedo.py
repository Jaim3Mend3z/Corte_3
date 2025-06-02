#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# Nombres de los 8 motores del Minitaur que queremos controlar
ACTUAL_MINITAUR_MOTOR_JOINT_NAMES = [
    "motor_front_rightR_joint", "motor_front_rightL_joint",
    "motor_back_rightR_joint",  "motor_back_rightL_joint",
    "motor_front_leftL_joint",  "motor_front_leftR_joint",
    "motor_back_leftL_joint",   "motor_back_leftR_joint"
]

# --- PERFIL DE QOS CAMBIADO A BEST_EFFORT ---
qos_profile_communications = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, # <--- CAMBIO AQUÍ
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

class PyBulletMinitaurNode(Node):
    def __init__(self):
        super().__init__('pybullet_minitaur_simulator')
        
        self.get_logger().info('Iniciando simulación de Minitaur en PyBullet...')
        try:
            self.physics_client = p.connect(p.GUI)
        except p.error as e:
            self.get_logger().error(f"No se pudo conectar al servidor GUI: {e}, intentando modo DIRECT.")
            try:
                self.physics_client = p.connect(p.DIRECT)
                self.get_logger().info("Conectado a PyBullet en modo DIRECT (sin GUI).")
            except p.error as e_direct:
                self.get_logger().fatal(f"Fallo al conectar a PyBullet en modo DIRECT tampoco: {e_direct}")
                rclpy.shutdown(); return

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        
        minitaur_start_pos = [0, 0, 0.3] 
        minitaur_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        self.robot_id = p.loadURDF("quadruped/minitaur.urdf", 
                                   minitaur_start_pos, 
                                   minitaur_start_orientation)
        self.get_logger().info('Minitaur cargado en PyBullet.')

        self.num_total_joints = p.getNumJoints(self.robot_id)
        self.all_joint_names_map = {} 
        self.motor_joint_indices = [] 
        self.motor_joint_names_ordered = []

        self.get_logger().info("------ Mapeando Articulaciones del Minitaur ------")
        for i in range(self.num_total_joints):
            info = p.getJointInfo(self.robot_id, i)
            joint_name = info[1].decode('utf-8')
            self.all_joint_names_map[joint_name] = i
        
        for name in ACTUAL_MINITAUR_MOTOR_JOINT_NAMES:
            if name in self.all_joint_names_map:
                idx = self.all_joint_names_map[name]
                if p.getJointInfo(self.robot_id, idx)[2] == p.JOINT_REVOLUTE:
                    self.motor_joint_indices.append(idx)
                    self.motor_joint_names_ordered.append(name)
                    self.get_logger().info(f"Motor Principal Identificado: '{name}' (Índice: {idx})")
                else:
                    self.get_logger().error(f"'{name}' está en la lista pero NO es JOINT_REVOLUTE. Tipo: {p.getJointInfo(self.robot_id, idx)[2]}")
            else:
                self.get_logger().error(f"Motor esperado '{name}' NO encontrado en el URDF con ese nombre exacto.")
        
        if len(self.motor_joint_indices) == 8:
            self.get_logger().info(f"ÉXITO: Se identificaron y ordenaron correctamente los 8 motores principales.")
        else:
            self.get_logger().warn(f"ALERTA: Se identificaron {len(self.motor_joint_indices)} de los 8 motores. El control será incorrecto.")
        
        self.current_joint_targets = [0.0] * 8 

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/minitaur_joint_commands',
            self.command_callback,
            qos_profile_communications # Usando el perfil de QoS
        )
        
        self.joint_state_publisher = self.create_publisher(
            JointState, 
            '/minitaur_joint_states', 
            qos_profile_communications # Usando el perfil de QoS
        )
        
        self.get_logger().info(f"Nodo de simulación listo. Controlando {len(self.motor_joint_indices)} motores.")
        self.timer = self.create_timer(1./240., self.simulation_step_callback)

    def command_callback(self, msg):
        self.get_logger().info(f"SimNode: Comando de articulación recibido: {list(msg.data)}") # DEBUG
        target_positions = msg.data
        if len(target_positions) == 8:
            self.current_joint_targets = list(target_positions) 
        else:
            self.get_logger().warn(f"Comando recibido con {len(target_positions)} valores, se esperaban 8.")

    def simulation_step_callback(self):
        if p.isConnected(self.physics_client):
            if len(self.motor_joint_indices) == 8:
                for i, motor_idx in enumerate(self.motor_joint_indices):
                    target_pos_for_this_motor = self.current_joint_targets[i]
                    p.setJointMotorControl2(
                        bodyIndex=self.robot_id,
                        jointIndex=motor_idx,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=target_pos_for_this_motor,
                        force=100.0,  # <--- VALOR DE FUERZA AUMENTADO
                        positionGain=0.05, # <--- VALOR DE GANANCIA AJUSTADO
                        velocityGain=0.5   # <--- VALOR DE GANANCIA AJUSTADO
                    )
            p.stepSimulation()
            
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            positions = []
            velocities = []
            
            if len(self.motor_joint_names_ordered) == 8:
                joint_state_msg.name = self.motor_joint_names_ordered
                for motor_idx in self.motor_joint_indices:
                    state = p.getJointState(self.robot_id, motor_idx)
                    positions.append(state[0])
                    velocities.append(state[1])
            
            joint_state_msg.position = positions
            joint_state_msg.velocity = velocities
            self.joint_state_publisher.publish(joint_state_msg)
        else:
            self.get_logger().warn("PyBullet no está conectado.")

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletMinitaurNode()
    if p.isConnected(node.physics_client):
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Cerrando por KeyboardInterrupt')
        finally:
            if p.isConnected(node.physics_client): p.disconnect(node.physics_client)
            if hasattr(node, 'is_valid') and node.is_valid(): node.destroy_node()
            if rclpy.ok(): rclpy.shutdown()
    else:
        node.get_logger().error("No se pudo inicializar PyBullet. Terminando nodo.")
        if rclpy.ok(): 
            if hasattr(node, 'is_valid') and node.is_valid(): node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
