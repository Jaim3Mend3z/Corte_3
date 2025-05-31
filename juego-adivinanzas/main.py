# main.py (en carpeta bipedo)
import pybullet as p
import pybullet_data
import time
import os

# ---- Inicialización de PyBullet ----
# Conectar al motor de física (con interfaz gráfica GUI)
physicsClient = p.connect(p.GUI)
# Establecer la gravedad (en dirección Z negativa)
p.setGravity(0, 0, -9.8)

# ---- Añadir rutas para encontrar archivos ----
# Ruta estándar de datos de PyBullet (contiene planos, etc.)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# Directorio actual donde se ejecuta el script (para buscar el URDF local)
p.setAdditionalSearchPath('.')

# ---- Carga del Robot Bípedo ----
# Construir la ruta al archivo URDF del robot
# Asume una carpeta 'biped' con 'biped2d.urdf' dentro, al mismo nivel que el script
urdf_folder = "biped"
urdf_filename = "biped2d.urdf"
urdf_path = os.path.join(urdf_folder, urdf_filename)

# Verificar que el archivo URDF existe antes de cargarlo
if not os.path.exists(urdf_path):
    # Si no existe, lanzar un error claro y detener la ejecución
    raise FileNotFoundError(
        f"No se encontró el archivo URDF: '{urdf_path}'. "
        f"Asegúrate de que la carpeta '{urdf_folder}' exista en el mismo directorio que el script "
        f"y contenga el archivo '{urdf_filename}'."
    )

# Cargar el robot desde el archivo URDF
# Posición inicial [x, y, z] = [0, 0, 2] (elevado para no empezar en el suelo)
# useFixedBase=False: permite que la base del robot (torso) se mueva libremente. Esencial para un bípedo.
robot_id = p.loadURDF(urdf_path, [0, 0, 2], useFixedBase=False)

# ---- Configuración del Control PD (Proporcional-Derivativo) ----
# Índices de las articulaciones del robot que se van a controlar
# (Estos índices dependen de cómo esté definido el archivo biped2d.urdf)
joint_indices = [0, 1, 2, 3]
# Ganancia Proporcional (kp): qué tan fuerte el motor intenta alcanzar la posición objetivo.
kp = 100
# Ganancia Derivativa (kd): qué tan fuerte el motor frena el movimiento (amortiguación).
kd = 10
# Lista de ángulos objetivo (en radianes) para cada articulación en joint_indices
# El primer ángulo es para joint_indices[0], el segundo para joint_indices[1], etc.
target_angles = [0.5, -0.5, 0.3, -0.3]

# ---- Bucle Principal de Simulación ----
# Ejecutar la simulación por un número fijo de pasos (1000)
for i in range(1000):
    # Aplicar el control a cada una de las articulaciones especificadas
    # 'enumerate' proporciona tanto el índice (idx) como el valor (joint_index)
    for idx, joint_index in enumerate(joint_indices):
        # Establecer el control del motor para la articulación actual
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,           # ID del cuerpo/robot
            jointIndex=joint_index,          # ID de la articulación específica
            controlMode=p.POSITION_CONTROL,  # Modo de control: intentar alcanzar una posición
            targetPosition=target_angles[idx],# Posición angular objetivo (en radianes)
            positionGain=kp,                 # Ganancia proporcional (kp)
            velocityGain=kd                  # Ganancia derivativa (kd)
        )

    # Avanzar la simulación física un paso de tiempo interno
    p.stepSimulation()
    # Hacer una pequeña pausa para que la simulación sea visible en la GUI
    # 10/240 es aproximadamente 0.0416 segundos (41.6 ms)
    time.sleep(10/240)

# --- Fin de la Simulación ---
# (Opcional pero recomendado) Desconectar del motor de física al terminar
# print("Simulación terminada. Desconectando.")
# p.disconnect()
