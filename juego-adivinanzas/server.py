import socket
import threading
import random

class GameServer:
    def __init__(self, host='0.0.0.0', port=5000):
        # Crear un socket TCP/IP
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Vincular el socket a la dirección y puerto especificados
        # '0.0.0.0' significa que escuchará en todas las interfaces de red disponibles
        self.server.bind((host, port))
        # Poner el servidor en modo de escucha, aceptando hasta 5 conexiones en cola
        self.server.listen(5)
        # Generar el número secreto que los clientes intentarán adivinar
        self.number = random.randint(1, 100)
        # Imprimir en la consola del servidor que está activo y cuál es el número secreto
        print(f"[*] Servidor activo. Numero Secreto: {self.number}")

    def handle_client(self, client_socket):
        # Enviar mensaje inicial al cliente conectado
        client_socket.send(b"Adivina el numero (1-100): \n")
        # Bucle para manejar las adivinanzas del cliente
        while True:
            try:
                # Recibir datos del cliente (hasta 1024 bytes)
                # Decodificar de bytes a string y quitar espacios en blanco/saltos de línea
                guess_str = client_socket.recv(1024).decode().strip()
                # Intentar convertir la respuesta a entero
                guess = int(guess_str)

                # Comparar la adivinanza con el número secreto
                if guess == self.number:
                    client_socket.send(b"Correcto!\n")
                    break # Salir del bucle si adivinó correctamente
                elif guess < self.number:
                    client_socket.send(b"Mayor\n") # Indicar que el número es mayor
                else:
                    client_socket.send(b"Menor\n") # Indicar que el número es menor
            except (ValueError, ConnectionResetError, BrokenPipeError):
                # Manejar errores si el cliente envía datos no numéricos,
                # se desconecta abruptamente o cierra la conexión.
                print(f"[*] Cliente desconectado o error.")
                break # Salir del bucle en caso de error

        # Cerrar la conexión con este cliente una vez que el juego termina o hay un error
        client_socket.close()

    def run(self):
        print(f"[*] Escuchando en {self.server.getsockname()}")
        # Bucle principal del servidor para aceptar conexiones entrantes
        while True:
            # Aceptar una nueva conexión. `accept()` bloquea hasta que llega una.
            # Devuelve el objeto socket del cliente y su dirección (IP, puerto)
            client, addr = self.server.accept()
            print(f"[*] Conexión aceptada desde {addr[0]}:{addr[1]}")
            # Crear un nuevo hilo para manejar a este cliente de forma independiente
            # target: la función que ejecutará el hilo (handle_client)
            # args: los argumentos para la función target (el socket del cliente)
            client_handler = threading.Thread(target=self.handle_client, args=(client,))
            # Iniciar la ejecución del hilo
            client_handler.start()

# Punto de entrada del script
if __name__ == "__main__":
    # Crear una instancia del servidor
    server_instance = GameServer()
    # Ejecutar el método run para empezar a escuchar conexiones
    server_instance.run()
