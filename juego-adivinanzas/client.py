import pygame
import socket

# Inicializar Pygame
pygame.init()

# Configuración de la ventana (simulando una consola)
width, height = 640, 480
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Cliente del Juego de Adivinar") # Añadido un título a la ventana
font = pygame.font.Font(None, 36) # Fuente por defecto, tamaño 36

def main():
    # Configurar el socket del cliente
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = ('localhost', 5000) # Dirección del servidor (cambiar si es necesario)
    try:
        client.connect(server_address)
        print(f"Conectado al servidor en {server_address}")
    except ConnectionRefusedError:
        print(f"Error: No se pudo conectar al servidor en {server_address}. ¿Está corriendo?")
        pygame.quit()
        return # Salir si no se puede conectar

    input_text = "" # Variable para almacenar la entrada del usuario
    server_message = "" # Variable para almacenar el último mensaje del servidor
    running = True

    # Recibir el mensaje inicial del servidor (no bloqueante)
    client.setblocking(False) # Evita que recv bloquee si no hay datos iniciales
    try:
        initial_data = client.recv(1024)
        if initial_data:
            server_message = initial_data.decode().strip()
    except BlockingIOError:
        # Es normal si el servidor no envía nada inmediatamente
        pass
    client.setblocking(True) # Volver a modo bloqueante para el bucle principal

    # Bucle principal del juego/cliente
    while running:
        # Manejo de eventos de Pygame (teclado, cierre de ventana)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False # Salir del bucle si se cierra la ventana
            elif event.type == pygame.KEYDOWN: # Si se presiona una tecla
                if event.key == pygame.K_RETURN: # Si es la tecla Enter
                    if input_text: # Solo enviar si hay algo escrito
                        print(f"Enviando: {input_text}")
                        client.send(input_text.encode()) # Enviar el texto al servidor
                        input_text = "" # Limpiar el campo de entrada después de enviar

                        # Esperar y recibir respuesta del servidor (bloqueante)
                        try:
                            data = client.recv(1024)
                            if data:
                                server_message = data.decode().strip()
                                print(f"Recibido: {server_message}")
                                if "Correcto!" in server_message:
                                    # Podríamos hacer algo especial aquí, como mostrar un mensaje final
                                    pass
                            else:
                                # El servidor cerró la conexión
                                print("El servidor cerró la conexión.")
                                running = False
                        except ConnectionResetError:
                            print("Error: Conexión perdida con el servidor.")
                            running = False

                elif event.key == pygame.K_BACKSPACE: # Si es la tecla de borrar
                    input_text = input_text[:-1] # Eliminar el último carácter
                elif event.unicode.isdigit(): # Solo permitir dígitos (mejorado)
                    input_text += event.unicode # Añadir el carácter numérico a la entrada

        # --- Sección de Dibujo en Pantalla ---
        screen.fill((0, 0, 0)) # Limpiar pantalla (fondo negro)

        # Mostrar mensaje del servidor (si existe)
        if server_message:
             server_msg_surface = font.render(server_message, True, (255, 255, 255)) # Blanco
             screen.blit(server_msg_surface, (20, 20)) # Posición superior

        # Mostrar texto de entrada actual del usuario
        input_label_surface = font.render("Tu numero: ", True, (255, 255, 255)) # Blanco
        input_surface = font.render(input_text, True, (0, 255, 0)) # Verde para la entrada
        screen.blit(input_label_surface, (20, height - 50)) # Posición inferior
        screen.blit(input_surface, (20 + input_label_surface.get_width(), height - 50))

        # Actualizar la pantalla completa para mostrar lo dibujado
        pygame.display.flip()
        # --- Fin Sección de Dibujo ---

    # Cerrar la conexión y salir de Pygame
    print("Cerrando conexión y saliendo...")
    client.close()
    pygame.quit()

# Punto de entrada del script
if __name__ == "__main__":
    main()

