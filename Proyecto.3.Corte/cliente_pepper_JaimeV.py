# cliente_pepper_JaimeV.py
# -*- coding: utf-8 -*-

import qi
import time
import httplib
import json

# IP del servidor (tu PC)
SERVER_IP = "192.168.0.107"
SERVER_PORT = 9559

# Crear sesión con Pepper
session = qi.Session()
session.connect("tcp://127.0.0.1:9559")  # Conexión local en Pepper

# Obtener servicios
asr = session.service("ALSpeechRecognition")
memory = session.service("ALMemory")
tts = session.service("ALAnimatedSpeech")

# Definir vocabulario amplio
VOCABULARIO = [
    "linux", "pepper", "hola", "adiós", "windows", "gracias", "tiempo", "qué", "cómo", "cuál",
    "día", "noche", "clima", "música", "bailar", "comida", "bebida", "amigo", "trabajo",
    "programación", "código", "robot", "saludo", "despedida", "favor", "ayuda", "juego",
    "película", "libro", "viaje", "fiesta", "casa", "escuela", "coche", "computadora"
]

# Flag para controlar cuando Pepper está hablando
hablando = False

# Tiempo de espera después de hablar (en segundos)
TIEMPO_ESPERA_POST_HABLAR = 3  # Puedes ajustar este valor

# Buffer para acumular el historial de palabras reconocidas
buffer_palabras = []

# Función para enviar la pregunta al servidor Flask, con todo el buffer acumulado
def enviar_pregunta(mensaje):
    try:
        conn = httplib.HTTPConnection(SERVER_IP, SERVER_PORT)
        headers = {"Content-type": "application/json"}
        # Enviar todo el historial concatenado como contexto
        data = json.dumps({"question": mensaje})
        conn.request("POST", "/chat", data, headers)
        response = conn.getresponse()
        respuesta = json.loads(response.read())["respuesta"]
        return respuesta
    except Exception as e:
        return "Error al contactar el servidor: " + str(e)

# Callback cuando Pepper reconoce una palabra
def on_palabra_detectada(value):
    global hablando
    global buffer_palabras

    if hablando:
        return

    if value and isinstance(value, list):
        palabra = value[0]
        print("[Pepper] Palabra detectada:", palabra)
        if palabra.lower() in VOCABULARIO:
            buffer_palabras.append(palabra.lower())  # Añadimos la palabra al buffer
            hablando = True
            # Concatenamos todo el buffer para enviar contexto
            mensaje_completo = ' '.join(buffer_palabras)
            respuesta = enviar_pregunta(mensaje_completo)
            print("[Pepper] ChatSimón:", respuesta)
            tts.say(respuesta)
            time.sleep(TIEMPO_ESPERA_POST_HABLAR)  # Tiempo de espera adicional
            hablando = False

# Detener cualquier sesión previa del ASR para evitar el error
try:
    asr.unsubscribe("cliente_pepper")
except RuntimeError:
    pass  # No estaba suscrito, ignorar

# Configurar reconocimiento de voz
asr.setLanguage("Spanish")
asr.setVocabulary(VOCABULARIO, False)

# Mensaje introductorio
tts.say("Hola, soy ChatSimó a través del cuerpo valioso de Pepper, estoy aquí para servir como tu asistente sarcástico de Linux. Si te digo algo cruel, es por tu bien ;)")

# Suscribirse al evento "WordRecognized"
suscriptor = memory.subscriber("WordRecognized")
suscriptor.signal.connect(on_palabra_detectada)

# Iniciar reconocimiento
asr.subscribe("cliente_pepper")
print("[Pepper] Escuchando...")

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\nDetenido por usuario.")
    asr.unsubscribe("cliente_pepper")

