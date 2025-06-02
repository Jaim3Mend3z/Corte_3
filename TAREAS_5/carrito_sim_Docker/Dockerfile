# Usa una imagen base de Python. La versión 'slim' es más ligera.
FROM python:3.9-slim

# Instala las dependencias necesarias para tkinter.
# 'tk' es la librería gráfica que Python necesita.
RUN apt-get update && apt-get install -y tk

# Establece el directorio de trabajo dentro del contenedor.
WORKDIR /app

# Copia tu script de Python al directorio de trabajo del contenedor.
COPY carrito_pista_desafiante.py .

# Define el comando que se ejecutará cuando inicie el contenedor.
CMD ["python3", "carrito_pista_desafiante.py"]
