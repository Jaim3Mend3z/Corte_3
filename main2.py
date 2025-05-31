# main.py 3 ->
import pybullet as p
import pybullet_data
import numpy as np
import time

# Configuración Inicial
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# La gravedad está desactivada (0, 0, 0) en la imagen, no (0,0,-10) como usualmente
p.setGravity(0, 0, 0) 
# Carga un brazo robótico Kuka IIWA
robot = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True) 

# Objetivo aleatorio
# Genera una posición objetivo [x, y, z] aleatoria dentro de un cubo
target_pos = np.random.uniform([-0.5, -0.5, 0.2], [0.5, 0.5, 0.5]) 

# Cinemática inversa
# Selecciona los índices de las primeras 7 articulaciones (asumiendo que es un Kuka de 7 DOF)
joint_indices = range(p.getNumJoints(robot))[:7] # El comentario dice 6 DOF, pero el código usa 7 juntas.
# Calcula los ángulos de las articulaciones necesarios para alcanzar target_pos
ik_solution = p.calculateInverseKinematics(
    bodyUniqueId=robot,
    endEffectorLinkIndex=6, # Índice del eslabón final (efector final)
    targetPosition=target_pos,
    maxNumIterations=100 # Número máximo de iteraciones para la cinemática inversa
)

# Aplicar ángulos calculados a las articulaciones
# Establece instantáneamente las articulaciones en los ángulos calculados por ik_solution
for i, angle in zip(joint_indices, ik_solution):
    p.resetJointState(robot, i, angle) # Usa resetJointState para teletransportar las juntas

# Simulación simple para visualizar el estado
for _ in range(200): # Corre la simulación por 200 pasos
    p.stepSimulation()
    time.sleep(100/240) # Pausa para visualización

# Nota: El código en la imagen no incluye p.disconnect() al final.
