# main.py 2 x
import pybullet as p
import pybullet_data
import time

# Inicializar simulación
physicsClient = p.connect(p.GUI) # Usar p.DIRECT para modo sin gráficos
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Cargar plano y carro
planeId = p.loadURDF("plane.urdf")
carPos = [0, 0, 0.5]
car = p.loadURDF("cartpole.urdf", carPos, useFixedBase=True) # Parece que se carga un 'cartpole', no un 'carro'

# Configurar ruedas (simplificado)
# Nota: Este código asume una configuración específica del URDF 'cartpole.urdf'
wheelIndices = [1, 3] # Índices de las articulaciones de las ruedas (según el URDF)
steeringIndices = [0, 2] # Índices de las articulaciones de dirección (según el URDF)

# Simulación
for i in range(1000):
  # Aplicar control a las ruedas y dirección
  # Control de velocidad para la primera rueda
  p.setJointMotorControl2(
      bodyUniqueId=car,
      jointIndex=wheelIndices[0], # Aplica a la rueda con índice 1
      controlMode=p.VELOCITY_CONTROL,
      targetVelocity=5, # Velocidad objetivo
      force=10 # Fuerza máxima del motor
  )
  # Control de posición para la primera articulación de dirección
  p.setJointMotorControl2(
      bodyUniqueId=car,
      jointIndex=steeringIndices[0], # Aplica a la dirección con índice 0
      controlMode=p.POSITION_CONTROL,
      targetPosition=0.5 # Posición objetivo (ángulo en radianes)
  )

  # Avanzar un paso en la simulación física
  p.stepSimulation()
  # Pausar brevemente para que la simulación sea visible
  time.sleep(10/240) # Pausa de aprox. 41.6 ms

# Desconectar del motor de física
p.disconnect()
