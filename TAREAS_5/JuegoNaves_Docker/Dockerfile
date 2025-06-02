# Usar una imagen base de Python. Python 3.11 es una buena opción.
FROM python:3.11-slim-bookworm

# Establecer el directorio de trabajo dentro del contenedor
WORKDIR /app

# Instalar python3-pip y las dependencias de sistema cruciales para Pygame (librerías SDL)
RUN apt-get update && apt-get install -y \
    python3-pip \
    libsdl2-2.0-0 \
    libsdl2-image-2.0-0 \
    libsdl2-mixer-2.0-0 \
    libsdl2-ttf-2.0-0 \
    # Las siguientes son a veces necesarias para funcionalidades completas de Pygame/SDL
    # y para evitar otros posibles errores de "missing library" más adelante.
    # Si la construcción es muy lenta o falla por alguna de estas, puedes probar comentándolas.
    libportmidi0 \
    libjpeg-dev \
    libpng-dev \
    libtiff-dev \
    libwebp-dev \
    # ttf-mscorefonts-installer # Este puede requerir interacción, mejor evitarlo si es posible
    libsm6 \
    libxext6 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Ahora, instalar pygame usando pip
RUN pip3 install pygame

# Copiar todos los archivos del directorio actual (donde está el Dockerfile)
# al directorio de trabajo /app dentro del contenedor.
# Esto incluye tu script .py y la carpeta 'assets'.
COPY . /app/

# Comando para ejecutar el juego cuando el contenedor se inicie
CMD ["python3", "Juegos_Naves_Espaciales_Mejorado.py"]

