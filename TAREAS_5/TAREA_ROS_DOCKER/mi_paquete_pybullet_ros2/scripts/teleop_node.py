#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
from pynput import keyboard # Importar pynput
import threading

instructions = """
Control del Brazo KUKA (ROS 2 con Pynput)
---------------------------
Selecciona una articulación (1-7)
Usa las flechas Arriba/Abajo para mover la articulación seleccionada.
Presiona 'ESC' para salir.
"""

class KukaTeleopNode(Node):
    def __init__(self):
        super().__init__('kuka_teleop_keyboard')
        self.publisher = self.create_publisher(Float64MultiArray, '/kuka_joint_commands', 10)
        self.timer = self.create_timer(0.1, self.publish_commands_periodically) 

        self.joint_positions = [0.0] * 7
        self.selected_joint = 0
        self.running = True

        self.key_listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener_thread = threading.Thread(target=self.key_listener.start)
        self.listener_thread.daemon = True
        self.listener_thread.start()

        print(instructions)

    def on_key_press(self, key):
        # --- LÍNEA DE DEPURACIÓN AÑADIDA ---
        print(f"DEBUG PYNPUtils: Tecla presionada: {key}, Tipo: {type(key)}")

        if not self.running:
            return False

        if hasattr(key, 'char') and key.char is not None:
            if '1' <= key.char <= '7':
                self.selected_joint = int(key.char) - 1
                print(f"\nArticulación seleccionada: {self.selected_joint + 1}")
                sys.stdout.write(f"\rPosiciones: [{', '.join(['{:.2f}'.format(p) for p in self.joint_positions])}]")
                sys.stdout.flush()
                return True 

        elif key == keyboard.Key.up:
            self.joint_positions[self.selected_joint] += 0.1
            sys.stdout.write(f"\rPosiciones: [{', '.join(['{:.2f}'.format(p) for p in self.joint_positions])}]")
            sys.stdout.flush()
        elif key == keyboard.Key.down:
            self.joint_positions[self.selected_joint] -= 0.1
            sys.stdout.write(f"\rPosiciones: [{', '.join(['{:.2f}'.format(p) for p in self.joint_positions])}]")
            sys.stdout.flush()
        elif key == keyboard.Key.esc:
            print("\nSaliendo...")
            self.running = False
            return False 

        return True

    def publish_commands_periodically(self):
        if not self.running:
            if rclpy.ok():
                pass 
            return

        command_msg = Float64MultiArray()
        command_msg.data = [round(p, 4) for p in self.joint_positions]
        self.publisher.publish(command_msg)

def main(args=None):
    rclpy.init(args=args)
    teleop_node = KukaTeleopNode()

    try:
        while rclpy.ok() and teleop_node.running:
            rclpy.spin_once(teleop_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        teleop_node.get_logger().info('Cerrando por KeyboardInterrupt')
    finally:
        teleop_node.running = False
        if teleop_node.key_listener.is_alive():
             teleop_node.key_listener.stop()
        if hasattr(teleop_node, 'listener_thread') and teleop_node.listener_thread.is_alive():
             teleop_node.listener_thread.join(timeout=0.2)

        if rclpy.ok():
            if hasattr(teleop_node, '_labels') and teleop_node._labels: # Comprobación más segura
                try:
                    teleop_node.destroy_node()
                except Exception as e:
                    if hasattr(teleop_node, 'get_logger'):
                        teleop_node.get_logger().error(f"Error al destruir el nodo: {e}")
                    else:
                        print(f"Error al destruir el nodo (logger no disponible): {e}")
            rclpy.shutdown()

if __name__ == '__main__':
    main()
