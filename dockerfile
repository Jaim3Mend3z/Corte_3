# Usar una imagen base ligera de Python 3.8
FROM python:3.8-slim

# Instalar dependencias del sistema operativo necesarias para PyBullet (renderizado headless/GUI)
# libgl1-mesa-glx: para OpenGL
# libglib2.0-0: dependencia de algunas bibliotecas GUI
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    # Limpiar caché de apt para reducir el tamaño de la imagen
    && rm -rf /var/lib/apt/lists/*

# Instalar las bibliotecas de Python: PyBullet y NumPy
RUN pip install pybullet numpy

# Copiar el script de Python del seguidor de línea al directorio /app dentro del contenedor
COPY line_follower.py /app/line_follower.py

# Establecer el directorio de trabajo dentro del contenedor a /app
WORKDIR /app

# Comando por defecto que se ejecutará cuando el contenedor inicie: ejecutar el script de Python
CMD ["python", "line_follower.py"]
