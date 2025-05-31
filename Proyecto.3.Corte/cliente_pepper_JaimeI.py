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
    # Puedes seguir agregando más palabras
]

# Flag para controlar cuando Pepper está hablando
hablando = False

# Tiempo de espera después de hablar (en segundos)
TIEMPO_ESPERA_POST_HABLAR = 3  # Puedes ajustar este valor

# Función para enviar la pregunta al servidor Flask
def enviar_pregunta(mensaje):
    try:
        conn = httplib.HTTPConnection(SERVER_IP, SERVER_PORT)
        headers = {"Content-type": "application/json"}
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
    if hablando:
        return

    if value and isinstance(value, list):
        palabra = value[0]
        print("[Pepper] Palabra detectada:", palabra)
        if palabra.lower() in VOCABULARIO:
            hablando = True
            respuesta = enviar_pregunta(palabra)
            print("[Pepper] Respuesta:", respuesta)
            tts.say(respuesta)
            time.sleep(TIEMPO_ESPERA_POST_HABLAR)  # Tiempo de espera adicional
            hablando = False

# Configurar reconocimiento de voz
asr.setLanguage("Spanish")
asr.setVocabulary(VOCABULARIO, False)

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

