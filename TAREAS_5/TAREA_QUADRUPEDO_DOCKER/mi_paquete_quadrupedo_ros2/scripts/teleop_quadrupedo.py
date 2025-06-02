#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
from pynput import keyboard 
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy # IMPORTACIÓN DE QOS

MOTOR_NAMES = [
    "motor_front_rightR_joint", "motor_front_rightL_joint",
    "motor_back_rightR_joint",  "motor_back_rightL_joint",
    "motor_front_leftL_joint",  "motor_front_leftR_joint",
    "motor_back_leftL_joint",   "motor_back_leftR_joint"
]
instructions_header = """
Control del Minitaur (ROS 2 con Pynput) - 8 Motores
----------------------------------------------------
Selecciona un motor (1-8) y luego usa Flechas Arriba/Abajo.
Presiona 'ESC' para salir.
Motores (1-8):
"""
motor_instructions = "\n".join([f"  {i+1}: {name}" for i, name in enumerate(MOTOR_NAMES)])
instructions = instructions_header + motor_instructions

# --- PERFIL DE QOS CAMBIADO A BEST_EFFORT ---
qos_profile_communications = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT, # <--- CAMBIO AQUÍ
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    durability=DurabilityPolicy.VOLATILE
)

class MinitaurTeleopNode(Node):
    def __init__(self):
        super().__init__('minitaur_teleop_keyboard')
        self.publisher = self.create_publisher(
            Float64MultiArray, 
            '/minitaur_joint_commands', 
            qos_profile_communications # Usando el perfil de QoS
        )
        self.timer = self.create_timer(0.1, self.publish_commands_periodically) 
        
        self.num_motors = 8 
        self.joint_positions = [0.0] * self.num_motors
        self.selected_motor_index = 0 
        self.running = True
        
        self.key_listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener_thread = threading.Thread(target=self.key_listener.start)
        self.listener_thread.daemon = True
        self.listener_thread.start()
        
        print(instructions)
        sys.stdout.write(f"\rPosiciones: [{', '.join(['{:.2f}'.format(p) for p in self.joint_positions])}]")
        sys.stdout.flush()

    def on_key_press(self, key):
        # print(f"DEBUG PYNPUtils: Tecla presionada: {key}, Tipo: {type(key)}") 
        if not self.running: return False

        if hasattr(key, 'char') and key.char is not None:
            if '1' <= key.char <= str(self.num_motors):
                self.selected_motor_index = int(key.char) - 1
                print(f"\nControlando Motor {self.selected_motor_index + 1}: {MOTOR_NAMES[self.selected_motor_index]}")
        elif key == keyboard.Key.up:
            self.joint_positions[self.selected_motor_index] += 0.1
        elif key == keyboard.Key.down:
            self.joint_positions[self.selected_motor_index] -= 0.1
        elif key == keyboard.Key.esc:
            print("\nSaliendo...")
            self.running = False
            return False 
        
        sys.stdout.write(f"\rPosiciones: [{', '.join(['{:.2f}'.format(p) for p in self.joint_positions])}]")
        sys.stdout.flush()
        return True

    def publish_commands_periodically(self):
        if not self.running:
            if rclpy.ok(): pass 
            return
        command_msg = Float64MultiArray()
        command_msg.data = [round(p, 4) for p in self.joint_positions]
        self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = MinitaurTeleopNode()
    try:
        while rclpy.ok() and teleop_node.running:
            rclpy.spin_once(teleop_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        teleop_node.get_logger().info('Cerrando por KeyboardInterrupt')
    finally:
        teleop_node.running = False
        if teleop_node.key_listener.is_alive(): teleop_node.key_listener.stop()
        if hasattr(teleop_node, 'listener_thread') and teleop_node.listener_thread.is_alive():
             teleop_node.listener_thread.join(timeout=0.2)
        if rclpy.ok():
            if hasattr(teleop_node, 'is_valid') and teleop_node.is_valid():
                try: teleop_node.destroy_node()
                except Exception: pass
            rclpy.shutdown()

if __name__ == '__main__':
    main()
