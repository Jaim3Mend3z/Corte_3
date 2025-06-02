# TAREA 5: Exploración, Personalización y Despliegue de Algoritmos con Docker

## Introducción

Este documento detalla el proceso de desarrollo, personalización y despliegue de tres proyectos distintos como parte de la TAREA 5. Los objetivos principales incluyeron la exploración de algoritmos existentes, la adición de funcionalidades y un "toque personal", y finalmente, la ejecución de estas aplicaciones en entornos aislados y portables utilizando Docker. Los proyectos abordados son:

1.  Un **Circuito Seguidor de Línea** implementado en Tkinter.
2.  Un **Juego de Naves Espaciales** desarrollado con Pygame.
3.  Un ejemplo de **Simulación y Control de Robots con ROS 2 y PyBullet**.

A lo largo de este README, se describirá el funcionamiento, las modificaciones realizadas, los desafíos encontrados y las soluciones implementadas para cada proyecto, con un énfasis especial en el proceso de "dockerización".

*(Nota: Este README resume una extensa interacción de desarrollo y depuración. Los fragmentos de código son ilustrativos; el código completo funcional se encuentra en los archivos correspondientes del proyecto.)*

## Estructura del Repositorio (Sugerida)

Para una organización clara, se sugiere la siguiente estructura de carpetas en un repositorio de GitHub:

/TAREA_5_COMPLETA/
├── Linea_Seguidor_Tkinter/
│   ├── carrito.py
│   ├── Dockerfile
│   └── assets/          # (Si tuviera imágenes para la pista, etc.)
│
├── Juego_Naves_Pygame/
│   ├── Juegos_Naves_Espaciales_Mejorado.py
│   ├── Dockerfile
│   └── assets/
│       ├── images/
│       └── sounds/
│
├── ROS2_Quadrupedo_PyBullet_Docker/
│   ├── Dockerfile
│   └── mi_paquete_quadrupedo_ros2/
│       ├── package.xml
│       ├── setup.py
│       ├── setup.cfg
│       ├── resource/
│       │   └── mi_paquete_quadrupedo_ros2
│       └── scripts/
│           ├── __init__.py
│           ├── sim_quadrupedo.py
│           └── teleop_quadrupedo.py
│
└── README.md  # Este archivo

---

## 1. Proyecto: Circuito Seguidor de Línea (Tkinter)

### Objetivo
Modificar un ejemplo existente de un coche seguidor de línea en Tkinter para que navegue una pista más compleja, con al menos tres curvas, y que esta pista sea un circuito cerrado.

### Repositorio Original de Ejemplo
[https://github.com/dialejobv/Sistemas_Operativos/tree/main/2)%20Carro_tkinter](https://github.com/dialejobv/Sistemas_Operativos/tree/main/2)%20Carro_tkinter)

### Descripción y Modificaciones Realizadas

El script original (`carrito.py`) simulaba un coche con sensores que seguía una línea negra sobre un fondo. Las modificaciones principales se centraron en:

1.  **Diseño de una Nueva Pista:**
    * Se reemplazó la pista original por un circuito cerrado complejo inspirado en una pista de Fórmula 1. Esto se logró modificando la lista de puntos (`stops`) que definen la trayectoria y el ancho de la pista en el canvas de Tkinter.
    * La nueva pista incluye múltiples curvas de diferentes radios y rectas.

2.  **Ajustes en el Vehículo y Control:**
    * Se ajustó la posición inicial (`car_x`, `car_y`) y el ángulo inicial (`car_angle`) del coche para que comenzara correctamente en la nueva línea de salida.
    * Se revisaron y ajustaron ligeramente los parámetros del controlador PID (`Kp`, `Ki`, `Kd`) dentro de la clase `LineFollowerCar` para mejorar la estabilidad y la capacidad de seguimiento en las curvas más pronunciadas de la nueva pista.
    * Se modificó la lógica para que el coche se detuviera al completar una vuelta en la nueva línea de meta.

3.  **Mejoras Visuales:**
    * Se cambió el color del coche y se añadieron elementos decorativos a la pista, como "pianos" en las curvas y un fondo tipo césped.
    * Se añadieron textos personalizables en el canvas, como el título de la pista y el nombre del autor ("Jaime Mendez - Universidad Santo Tomás"), utilizando `canvas.create_text(...)`.

### Cómo Ejecutar (Nativo)

1.  Asegúrate de tener Python 3 y Tkinter instalados (Tkinter suele venir con la instalación estándar de Python).
2.  Guarda el código modificado como `carrito_f1.py` (o el nombre que prefieras).
3.  Ejecuta desde la terminal:
    ```bash
    python3 carrito_f1.py
    ```

### Dockerización del Seguidor de Línea

Para ejecutar la simulación en Docker:

1.  **`Dockerfile` Sugerido** (colocar en la misma carpeta que `carrito_f1.py`):
    ```dockerfile
    # Usar una imagen base de Python delgada
    FROM python:3.11-slim

    # Establecer el directorio de trabajo dentro del contenedor
    WORKDIR /app

    # Copiar el script del seguidor de línea al directorio /app
    COPY carrito_f1.py .

    # Tkinter usualmente viene con Python, pero si hubiera otras dependencias:
    # RUN pip install <otras_dependencias>

    # Configurar la variable de entorno DISPLAY para GUI
    # Esto es crucial para aplicaciones Tkinter/GUI en Docker.
    # El valor :0 es común, pero podría necesitar ser :1 dependiendo de tu sistema.
    ENV DISPLAY=:0

    # Comando para correr la aplicación
    CMD ["python3", "carrito_f1.py"]
    ```

2.  **Construir la Imagen Docker:**
    Navega a la carpeta que contiene el `Dockerfile` y `carrito_f1.py`, y ejecuta:
    ```bash
    docker build -t seguidor-linea-app .
    ```

3.  **Ejecutar el Contenedor Docker:**
    Para que la GUI de Tkinter se muestre en tu pantalla desde el contenedor, necesitas compartir el socket X11 de tu máquina anfitriona.
    **(Solo Linux)** Primero, permite conexiones a tu servidor X (en una terminal de tu host):
    ```bash
    xhost +
    ```
    Luego, ejecuta el contenedor:
    ```bash
    docker run -it --rm \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        seguidor-linea-app
    ```

---

## 2. Proyecto: Juego de Naves Espaciales (Pygame)

### Objetivo
Tomar un juego de naves espaciales existente desarrollado en Pygame y añadirle una "esencia propia" para modificar su jugabilidad y características.

### Repositorio Original de Ejemplo
[https://github.com/dialejobv/Sistemas_Operativos/tree/main/3)%20Nave_espacial](https://github.com/dialejobv/Sistemas_Operativos/tree/main/3)%20Nave_espacial)

### Descripción y Modificaciones Realizadas ("Esencia Propia")

El script base proporcionaba un juego tipo "Space Invaders". Las modificaciones para darle una "esencia propia" se centraron en aumentar la rejugabilidad y el desafío:

1.  **Sistema de Olas (Waves):**
    * Se implementó un sistema donde los enemigos aparecen en olas sucesivas. Una vez que el jugador elimina a todos los enemigos de una ola, se presenta una nueva.
    * Se añadió una pantalla simple de "OLA X" entre rondas para informar al jugador.

2.  **Dificultad Progresiva:**
    * **Velocidad de Enemigos:** Con cada nueva ola, la velocidad base de los enemigos aumenta, haciendo el juego más desafiante.
    * **Movimiento Enemigo Mejorado:** Se cambió el movimiento en bloque de los enemigos. Ahora, cada enemigo tiene un movimiento horizontal individual y una oscilación vertical (usando `math.sin`), creando un efecto de enjambre más orgánico y menos predecible. La velocidad de esta oscilación también podría incrementarse con las olas.
    * **Frecuencia de Disparo Enemigo (Implementada):** Los enemigos ahora disparan balas hacia el jugador. La probabilidad de que un enemigo dispare aumenta ligeramente con cada nueva ola.
    * **Lógica de Avance Enemigo:** Se ajustó la lógica para que todo el enjambre de enemigos descienda un paso cuando uno de ellos alcanza un borde lateral de la pantalla. Si los enemigos llegan a la altura del jugador, este pierde una vida y la ola se reinicia.

3.  **Mejoras Adicionales:**
    * **Gestión de Assets Robusta:** Se implementaron funciones `load_image` y `load_sound` que intentan cargar los archivos y, si fallan (ej. archivo no encontrado), cargan un recurso de emergencia (una superficie de color o un sonido "dummy") y muestran un aviso en la consola, permitiendo que el juego se ejecute incluso con assets faltantes.
    * **Invulnerabilidad del Jugador:** Se añadió un breve periodo de invulnerabilidad (con parpadeo visual) para el jugador después de ser golpeado, evitando la pérdida instantánea de múltiples vidas.
    * **Música de Fondo y Sonidos:** Se integró música de fondo continua y se mantuvieron los sonidos para disparos y explosiones.
    * **Pantalla de Game Over:** Muestra la puntuación final.

### Cómo Ejecutar (Nativo)

1.  Asegúrate de tener Python 3 instalado.
2.  Instala Pygame. Si estás en Ubuntu 24.04 (o similar con protección PEP 668) y no usas un entorno virtual, el método recomendado es:
    ```bash
    sudo apt update
    sudo apt install python3-pygame
    ```
    Si usas un entorno virtual (ej. `venv`), actívalo y luego `pip install pygame`.
3.  Asegúrate de tener la carpeta `assets` con las imágenes y sonidos necesarios en el mismo directorio que el script `Juegos_Naves_Espaciales_Mejorado.py`.
4.  Ejecuta desde la terminal:
    ```bash
    python3 Juegos_Naves_Espaciales_Mejorado.py
    ```

### Dockerización del Juego de Naves

Para ejecutar el juego en Docker:

1.  **`Dockerfile` Sugerido** (colocar en una carpeta junto con `Juegos_Naves_Espaciales_Mejorado.py` y la carpeta `assets`):
    ```dockerfile
    FROM python:3.11-slim-bookworm

    WORKDIR /app

    # Instalar python3-pip y las dependencias de sistema cruciales para Pygame (librerías SDL)
    RUN apt-get update && apt-get install -y \
        python3-pip \
        libsdl2-2.0-0 \
        libsdl2-image-2.0-0 \
        libsdl2-mixer-2.0-0 \
        libsdl2-ttf-2.0-0 \
        libportmidi0 \
        libjpeg-dev \
        libpng-dev \
        libtiff-dev \
        libwebp-dev \
        libsm6 \
        libxext6 \
        ffmpeg \
        && rm -rf /var/lib/apt/lists/*

    # Instalar pygame usando pip
    RUN pip3 install pygame

    # Copiar el juego y los assets
    COPY . /app/

    # Comando para ejecutar el juego
    CMD ["python3", "Juegos_Naves_Espaciales_Mejorado.py"]
    ```

2.  **Construir la Imagen Docker:**
    Navega a la carpeta que contiene el `Dockerfile`, el script y la carpeta `assets`. Ejecuta:
    ```bash
    docker build -t mi-juego-naves .
    ```

3.  **Ejecutar el Contenedor Docker:**
    **(Solo Linux)** Permite conexiones a tu servidor X: `xhost +`
    Luego, ejecuta el contenedor:
    ```bash
    docker run -it --rm \
      -e DISPLAY=$DISPLAY \
      -v /tmp/.X11-unix:/tmp/.X11-unix \
      --device /dev/snd \
      -v /dev/dri:/dev/dri \
      mi-juego-naves
    ```
    * `--device /dev/snd`: Para dar acceso al hardware de sonido.
    * `-v /dev/dri:/dev/dri`: Para dar acceso a la aceleración gráfica (útil para Pygame).

---

## 3. Proyecto: Simulación de Robot Cuadrúpedo con ROS 2, PyBullet y Docker

### Objetivo
Explorar ROS, entender su funcionamiento y exponer un ejemplo de simulación y control robótico que pueda ejecutarse en Docker. Debido a que el sistema operativo de desarrollo fue Ubuntu 24.04, se optó por ROS 2 Jazzy Jalisco.

### Conceptualización de ROS 2
ROS (Robot Operating System) es un framework flexible para escribir software de robots. Proporciona herramientas y librerías para ayudar a los desarrolladores a crear aplicaciones robóticas complejas. Los conceptos clave utilizados en este proyecto incluyen:
* **Nodos:** Programas ejecutables independientes (ej. un nodo para simulación, un nodo para control).
* **Tópicos:** Canales de comunicación con nombre por donde los nodos intercambian datos.
* **Mensajes:** Los datos que se envían a través de los tópicos, con tipos definidos (ej. `Float64MultiArray` para comandos de articulación, `JointState` para el estado de las articulaciones).
* **Publicadores/Suscriptores:** Nodos que envían (publican) o reciben (se suscriben) mensajes en los tópicos.
* **Calidad de Servicio (QoS):** Reglas que definen cómo se entregan los mensajes (ej. fiabilidad `RELIABLE` vs. `BEST_EFFORT`).
* **Workspace y Paquetes:** Estructura de directorios donde se organiza y compila el código ROS 2 (`colcon build`).

### Descripción del Ejemplo Desarrollado: Control del Cuadrúpedo Minitaur

Se desarrolló una aplicación ROS 2 para controlar un robot cuadrúpedo Minitaur en el simulador PyBullet:

1.  **Nodo de Simulación (`sim_quadrupedo.py`):**
    * Utiliza `rclpy` para la funcionalidad de ROS 2 y `pybullet` para la simulación física.
    * Carga el modelo URDF del Minitaur y un plano en un entorno PyBullet con GUI.
    * Identifica los 8 motores principales del Minitaur basándose en una lista de nombres predefinidos (`ACTUAL_MINITAUR_MOTOR_JOINT_NAMES`) y verifica sus índices.
    * **Se suscribe** al tópico `/minitaur_joint_commands` (tipo `std_msgs/msg/Float64MultiArray`) para recibir los ángulos objetivo para los 8 motores.
    * En su función `command_callback`, actualiza los ángulos objetivo internos.
    * Un temporizador de ROS 2 llama periódicamente a `simulation_step_callback`.
    * En `simulation_step_callback`:
        * Aplica los ángulos objetivo a los motores del Minitaur en PyBullet usando `p.setJointMotorControl2()`. Se ajustaron los parámetros de `force`, `positionGain` y `velocityGain` para intentar obtener movimiento.
        * Avanza la simulación con `p.stepSimulation()`.
        * **Publica** el estado actual de las articulaciones (nombre, posición, velocidad) de los 8 motores en el tópico `/minitaur_joint_states` (tipo `sensor_msgs/msg/JointState`).
    * Utiliza perfiles de QoS explícitos (`BEST_EFFORT`) para mejorar la robustez de la comunicación en Docker.

2.  **Nodo de Teleoperación (`teleop_quadrupedo.py`):**
    * Utiliza `rclpy` y la librería `pynput` para una captura robusta de eventos de teclado.
    * Muestra al usuario una lista de los 8 motores controlables y sus nombres.
    * Permite al usuario seleccionar un motor (con teclas numéricas '1'-'8').
    * Permite al usuario modificar el ángulo objetivo del motor seleccionado usando las teclas de flecha arriba/abajo.
    * Mantiene un array interno con los 8 ángulos objetivo.
    * Un temporizador de ROS 2 llama periódicamente a `publish_commands_periodically`.
    * En `publish_commands_periodically`, **publica** el array completo de 8 ángulos objetivo al tópico `/minitaur_joint_commands`.
    * Utiliza el mismo perfil de QoS explícito (`BEST_EFFORT`) que el simulador.

### Proceso de Desarrollo y Depuración (Resumido)

El desarrollo de este ejemplo fue un proceso iterativo extenso, especialmente para la configuración nativa y la posterior dockerización:

* **Configuración Nativa (Ubuntu 24.04 + ROS 2 Jazzy):**
    * Instalación de ROS 2 Jazzy y sus dependencias.
    * Instalación de PyBullet (requirió `sudo apt install python3-pybullet` debido a PEP 668).
    * Creación del workspace (`ros2_ws`) y del paquete `mi_paquete_quadrupedo_ros2` con `package.xml`, `setup.py`, `setup.cfg`, `resource/nombre_paquete`, y `scripts/__init__.py`.
    * Compilación con `colcon build`.
    * Resolución de errores de importación de Python (`ModuleNotFoundError`) asegurando la correcta estructura del paquete, nombres de archivo (`.py`) y el archivo `__init__.py` en la carpeta `scripts`.
    * Solución de problemas de `source install/setup.bash` para que los nodos sean encontrables por `ros2 run`.

* **Dockerización del Ejemplo ROS 2:**
    * **`Dockerfile`:**
        * Se usó `FROM ros:jazzy` como imagen base (ya que `ros:jazzy-desktop` dio problemas de "manifest not found").
        * Se instalaron dependencias del sistema: `python3-pip`, librerías X11/OpenGL (`x11-apps`, `libgl1`, `libegl1`, `mesa-utils`), y `python3-pynput` (vía `apt`).
        * Se instaló PyBullet usando `pip3 install pybullet --break-system-packages` para superar la protección PEP 668 dentro del contenedor.
        * Se creó explícitamente el directorio `src` (`RUN mkdir src`) antes de copiar el paquete para solucionar un error de `COPY`.
        * Se copió el paquete `mi_paquete_quadrupedo_ros2` a `/ros2_ws/src/`.
        * Se compiló el workspace con `colcon build` dentro de la imagen.
    * **Problemas de Ejecución en Docker y Soluciones:**
        * **GUI (X11):** Se enfrentaron errores de `cannot connect to X server` y `Can't open display :0`. Se solucionaron progresivamente con:
            1.  `xhost +` en el anfitrión.
            2.  Pasando `-e DISPLAY=$DISPLAY` y `-v /tmp/.X11-unix:/tmp/.X11-unix` al `docker run`.
            3.  Confirmando que el usuario estaba en una sesión Xorg en el anfitrión (no Wayland).
            4.  Añadiendo `-v $HOME/.Xauthority:/root/.Xauthority:rw`.
        * **Acceso a GPU / Gráficos PyBullet:** Errores como `MESA: error: Failed to query drm device`, `failed to load driver: iris`. Se solucionaron añadiendo `-v /dev/dri:/dev/dri` al `docker run`.
        * **Conflictos de Nombres de Contenedor:** Solucionados usando `docker stop <nombre>` y `docker rm <nombre>` o confiando en `--rm`.
        * **Comunicación ROS 2 / DDS entre Contenedores:** Fue el desafío más persistente.
            * Síntomas: `teleop_node` actualizaba valores, pero `sim_node` no recibía los mensajes (el callback `command_callback` no se disparaba, no se veían los logs "Comando de articulación recibido"). El comando `ros2 node list` desde un contenedor no veía al otro, o el publicador aparecía como `_NODE_NAME_UNKNOWN_`.
            * Soluciones probadas y aplicadas:
                1.  Uso de `--net=host` para **ambos** contenedores para que compartan la pila de red del anfitrión.
                2.  Paso de `-e ROS_DOMAIN_ID=0` a ambos contenedores.
                3.  Paso de `-e ROS_LOCALHOST_ONLY=1` a ambos contenedores.
                4.  Definición y uso de perfiles de **QoS explícitos** en publicadores y suscriptores, probando primero con `ReliabilityPolicy.RELIABLE` y finalmente con `ReliabilityPolicy.BEST_EFFORT` para mayor robustez en el entorno Docker.
        * **Lógica de la Aplicación (Minitaur):**
            * Error inicial donde `sim_quadrupedo` identificaba 16 "motores" mientras `teleop_quadrupedo` esperaba 8. Se corrigió `sim_quadrupedo` para que filtrara y usara una lista predefinida de los 8 nombres de motores actuadores correctos del Minitaur.
            * El robot aún no se movía. Se concluyó que, una vez confirmada la recepción de mensajes en `sim_quadrupedo` (lo cual fue el último punto de depuración), el ajuste de los parámetros `force`, `positionGain` y `velocityGain` en la función `p.setJointMotorControl2` dentro de `sim_quadrupedo.py` sería el paso final para lograr el movimiento.

### Cómo Ejecutar (Docker)

1.  **Preparar la Carpeta:**
    * Asegúrate de tener la carpeta `TAREA_QUADRUPEDO_DOCKER` con el `Dockerfile` y la subcarpeta `mi_paquete_quadrupedo_ros2` (conteniendo los scripts Python corregidos, `package.xml`, `setup.py`, `setup.cfg`, `resource/mi_paquete_quadrupedo_ros2`, y `scripts/__init__.py`).

2.  **Construir la Imagen Docker:**
    Desde la carpeta `TAREA_QUADRUPEDO_DOCKER`:
    ```bash
    docker build -t ros2_quadrupedo_app .
    ```

3.  **Ejecutar los Contenedores:**
    **(Solo Linux)** En una terminal del host: `xhost +`
    * **Terminal 1 (Simulador `sim_quadrupedo`):**
        ```bash
        docker run -it --rm --name sim_quadrupedo_gui \
          --net=host \
          -e DISPLAY=$DISPLAY \
          -e ROS_DOMAIN_ID=0 \
          -e ROS_LOCALHOST_ONLY=1 \
          -v /tmp/.X11-unix:/tmp/.X11-unix \
          -v $HOME/.Xauthority:/root/.Xauthority:rw \
          -v /dev/dri:/dev/dri \
          ros2_quadrupedo_app \
          bash -c "source install/setup.bash && ros2 run mi_paquete_quadrupedo_ros2 sim_quadrupedo"
        ```
    * **Terminal 2 (Teleoperación `teleop_quadrupedo`):**
        ```bash
        docker run -it --rm --name teleop_quadrupedo_control \
          --net=host \
          -e DISPLAY=$DISPLAY \
          -e ROS_DOMAIN_ID=0 \
          -e ROS_LOCALHOST_ONLY=1 \
          -v /tmp/.X11-unix:/tmp/.X11-unix \
          -v $HOME/.Xauthority:/root/.Xauthority:rw \
          ros2_quadrupedo_app \
          bash -c "source install/setup.bash && ros2 run mi_paquete_quadrupedo_ros2 teleop_quadrupedo"
        ```

### Archivos Clave del Proyecto ROS 2
* `mi_paquete_quadrupedo_ros2/scripts/sim_quadrupedo.py`
* `mi_paquete_quadrupedo_ros2/scripts/teleop_quadrupedo.py`
* `mi_paquete_quadrupedo_ros2/package.xml`
* `mi_paquete_quadrupedo_ros2/setup.py`
* `Dockerfile` (en la raíz de `TAREA_QUADRUPEDO_DOCKER`)

---

## Conclusiones Generales del Aprendizaje

* La **exploración y adaptación de algoritmos** existentes permite una comprensión más profunda de su funcionamiento y la capacidad de extenderlos con nuevas funcionalidades personalizadas.
* **ROS 2** proporciona un potente framework para el desarrollo de software robótico modular, pero su configuración, especialmente en lo referente a la comunicación entre nodos (DDS y QoS), puede presentar desafíos, sobre todo en entornos de red no triviales o virtualizados.
* **Docker** es una herramienta invaluable para crear entornos de desarrollo y despliegue consistentes y portables, simplificando enormemente la gestión de dependencias complejas (como las de ROS 2, PyBullet, y librerías gráficas). Sin embargo, la dockerización de aplicaciones con **GUI y acceso a hardware** (gráficos, sonido, dispositivos de entrada) requiere una configuración cuidadosa de reenvío X11, montaje de volúmenes y, en algunos casos, opciones de red permisivas (`--net=host`).
* El **proceso de depuración** es una parte integral y fundamental del desarrollo de software. La capacidad de analizar logs, aislar problemas, formular hipótesis y probar soluciones de forma sistemática es crucial para superar los obstáculos técnicos, como se demostró extensamente en la configuración del entorno ROS 2/PyBullet/Docker.
* La **compatibilidad entre el sistema operativo del anfitrión y las versiones de software** (ej. Ubuntu 24.04 y ROS 2 Jazzy vs. ROS 1 Noetic) es un factor crítico, especialmente para el desarrollo nativo. Docker ayuda a mitigar estos problemas de compatibilidad.
* Detalles como la correcta **estructura de paquetes de Python** (presencia de `__init__.py`), la correcta configuración de los archivos de construcción de ROS 2 (`package.xml`, `setup.py`), y el ciclo de "editar código -> reconstruir (`colcon build` o `docker build`) -> ejecutar" son fundamentales para el éxito del desarrollo.

Este proyecto ha sido una experiencia de aprendizaje intensiva y muy valiosa, cubriendo un amplio espectro de tecnologías y metodologías de desarrollo relevantes en la ingeniería de software y la robótica.

---

## Agradecimientos (Opcional)

Quisiera expresar mi más sincero agradecimiento al Profesor Diego [Apellido del Profesor, si lo deseas], por su invaluable dedicación y guía a lo largo de este curso y, en particular, durante el desarrollo de esta compleja tarea. Su compromiso y la forma de plantear los desafíos han sido fundamentales para mi aprendizaje.

---

## Referencias (Opcional)

1.  dialejobv, "Ejemplo de Carro Seguidor de Línea en Tkinter (Sistemas_Operativos)," (Consultado en 2025). [En línea]. Disponible: `https://github.com/dialejobv/Sistemas_Operativos/tree/main/2)%20Carro_tkinter`
2.  dialejobv, "Ejemplo de Juego de Nave Espacial (Sistemas_Operativos)," (Consultado en 2025). [En línea]. Disponible: `https://github.com/dialejobv/Sistemas_Operativos/tree/main/3)%20Nave_espacial`
3.  Python Software Foundation, "Python Language Reference," (Consultado en 2025). [En línea]. Disponible: `http://www.python.org`
4.  Pygame Developers, "Pygame Documentation," (Consultado en 2025). [En línea]. Disponible: `https://www.pygame.org/docs/`
5.  Open Robotics, "ROS 2 Jazzy Jalisco Documentation," (Consultado en 2025). [En línea]. Disponible: `https://docs.ros.org/en/jazzy/`
6.  E. Coumans y Y. Bai, "PyBullet Physics SDK," (Consultado en 2025). [En línea]. Disponible: `https://pybullet.org`
7.  Docker, Inc., "Docker Official Website," (Consultado en 2025). [En línea]. Disponible: `https://www.docker.com`
8.  M. Tatra, "pynput - Control and monitor input devices," (Consultado en 2025). [En línea]. Disponible: `https://pypi.org/project/pynput/`
