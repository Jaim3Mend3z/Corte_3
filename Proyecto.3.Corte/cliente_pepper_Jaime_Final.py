# cliente_pepper_Jaime_Final_v7_port_confirmed.py
# -*- coding: utf-8 -*-

import qi
import time
import httplib 
import json

# --- Helper for printing Unicode safely in Python 2 ---
def print_u(format_string_or_direct_unicode, *args):
    try:
        final_unicode_string = u""
        current_format_string = format_string_or_direct_unicode
        if not isinstance(current_format_string, unicode):
            try:
                current_format_string = unicode(current_format_string)
            except UnicodeDecodeError:
                current_format_string = unicode(str(current_format_string), 'utf-8', 'replace')
        
        if args:
            processed_args = []
            for arg in args:
                if isinstance(arg, str): 
                    processed_args.append(unicode(arg, 'utf-8', 'replace'))
                else: 
                    processed_args.append(arg)
            final_unicode_string = current_format_string.format(*processed_args)
        else:
            final_unicode_string = current_format_string
        
        print final_unicode_string.encode('utf-8', 'replace')
    except Exception as e_print:
        output = "Error in print_u: {}. Original Format String: ".format(e_print)
        try:
            output += str(format_string_or_direct_unicode)
            if args:
                output += " Args: " + ", ".join([str(a) for a in args])
        except:
            output += "[Could not represent original message/args as str]"
        print output

# --- Configuración ---
SERVER_IP = "192.168.0.107" 
# Ajustado al puerto donde confirmaste que corre tu servidor Flask
SERVER_PORT = 9559 

KEYWORD_ENVIAR = u"enviar" # Asegurar que la keyword sea unicode
# Conservamos el umbral bajo para pruebas, recuerda ajustarlo después.
CONFIDENCE_THRESHOLD = 0.15 

session = qi.Session()
try:
    session.connect("tcp://127.0.0.1:9559") # Conexión a Naoqi en Pepper
except RuntimeError as e:
    print_u(u"Error fatal al conectar con Naoqi: {}", e)
    exit(1)

try:
    asr = session.service("ALSpeechRecognition")
    memory = session.service("ALMemory")
    tts = session.service("ALAnimatedSpeech")
except RuntimeError as e:
    print_u(u"Error fatal al obtener servicios de Naoqi: {}", e)
    if session.isConnected():
        session.close()
    exit(1)

# --- VOCABULARIO CON CADENAS UNICODE ---
VOCABULARIO = [
    KEYWORD_ENVIAR, 
    u"linux", u"pepper", u"hola", u"adiós", u"windows", u"gracias", u"tiempo", u"qué", u"cómo", u"cuál",
    u"día", u"noche", u"clima", u"música", u"bailar", u"comida", u"bebida", u"amigo", u"trabajo",
    u"programación", u"código", u"robot", u"saludo", u"despedida", u"favor", u"ayuda", u"juego",
    u"película", u"libro", u"viaje", u"fiesta", u"casa", u"escuela", u"coche", u"computadora",
    u"mesa", u"silla", u"puerta", u"ventana", u"calle", u"ciudad", u"país", u"mundo", u"sol", u"luna",
    u"estrella", u"agua", u"fuego", u"tierra", u"aire", u"árbol", u"flor", u"animal", u"perro", u"gato",
    u"pájaro", u"pez", u"hombre", u"mujer", u"niño", u"niña", u"familia", u"cuerpo", u"cabeza", u"mano",
    u"pie", u"ojo", u"nariz", u"boca", u"oreja", u"corazón", u"mente", u"alma", u"idea", u"sueño",
    u"amor", u"odio", u"alegría", u"tristeza", u"miedo", u"esperanza", u"vida", u"muerte", u"principio",
    u"final", u"color", u"rojo", u"azul", u"verde", u"amarillo", u"blanco", u"negro", u"forma", u"tamaño",
    u"sonido", u"silencio", u"historia", u"noticia", u"arte", u"ciencia", u"tecnología", u"naturaleza",
    u"problema", u"solución", u"pregunta", u"respuesta", u"camino", u"río", u"montaña", u"mar", u"cielo",
    u"nube", u"lluvia", u"nieve", u"viento", u"verano", u"otoño", u"invierno", u"primavera", u"semana",
    u"mes", u"año", u"hora", u"minuto", u"segundo", u"mañana", u"tarde", u"persona", u"gente", u"equipo",
    u"gobierno", u"ley", u"derecho", u"deber", u"guerra", u"paz", u"dinero", u"precio", u"mercado",
    u"tienda", u"producto", u"servicio", u"calidad", u"cantidad", u"nombre", u"palabra", u"lenguaje",
    u"carta", u"número", u"espacio", u"lugar", u"objeto", u"herramienta", u"máquina", u"sistema",
    u"proyecto", u"madrid", u"barcelona", u"valencia", u"sevilla", u"españa", u"europa", u"américa",
    u"asia", u"áfrica", u"oceanía", u"planeta", u"universo", u"galaxia", u"átomo", u"molécula",
    u"energía", u"fuerza", u"movimiento", u"velocidad", u"luz", u"oscuridad", u"verdad", u"mentira",
    u"error", u"éxito", u"fracaso", u"cultura", u"tradición", u"religión", u"filosofía", u"psicología",
    u"medicina", u"ingeniería", u"matemáticas", u"física", u"química", u"biología", u"geografía",
    u"economía", u"política", u"sociedad", u"deporte", u"jardín", u"cocina", u"habitación", u"baño",
    u"edificio", u"puente", u"carretera", u"avión", u"barco", u"tren", u"bicicleta", u"teléfono",
    u"internet", u"red", u"archivo", u"documento", u"información", u"dato", u"conocimiento",
    u"sabiduría", u"opinión", u"sentimiento", u"emoción", u"razón", u"lógica", u"memoria",
    u"olvido", u"inteligencia", u"creatividad", u"imaginación", u"voluntad", u"acción", u"actividad",
    u"ser", u"estar", u"tener", u"hacer", u"decir", u"ir", u"ver", u"dar", u"saber", u"querer",
    u"poder", u"llegar", u"pasar", u"creer", u"hablar", u"llevar", u"dejar", u"seguir", u"encontrar",
    u"llamar", u"venir", u"pensar", u"salir", u"volver", u"tomar", u"conocer", u"vivir", u"sentir",
    u"tratar", u"mirar", u"contar", u"empezar", u"esperar", u"buscar", u"entrar", u"trabajar", u"escribir",
    u"perder", u"producir", u"ocurrir", u"entender", u"pedir", u"recibir", u"recordar", u"terminar",
    u"permitir", u"aparecer", u"conseguir", u"comenzar", u"servir", u"sacar", u"necesitar", u"mantener",
    u"resultar", u"leer", u"caer", u"cambiar", u"presentar", u"crear", u"abrir", u"considerar", u"oír",
    u"acabar", u"convertir", u"ganar", u"formar", u"traer", u"partir", u"morir", u"aceptar", u"realizar",
    u"suponer", u"comprender", u"lograr", u"explicar", u"preguntar", u"tocar", u"reconocer", u"estudiar",
    u"alcanzar", u"dirigir", u"correr", u"utilizar", u"pagar", u"ayudar", u"gustar", u"jugar", u"escuchar",
    u"levantar", u"intentar", u"usar", u"decidir", u"repetir", u"olvidar", u"comer", u"beber", u"dormir",
    u"despertar", u"viajar", u"comprar", u"vender", u"amar", u"odiar", u"reír", u"llorar", u"cantar",
    u"aprender", u"enseñar", u"cerrar", u"subir", u"bajar", u"corregir", u"cortar", u"crecer", u"cumplir",
    u"defender", u"descubrir", u"desear", u"dibujar", u"elegir", 
    u"ganar", u"gastar", u"gritar", u"guardar", u"imaginar", u"informar", u"invitar", u"lavar", u"limpiar", u"luchar",
    u"mandar", u"manejar", u"medir", u"meter", u"mover", u"nacer", u"nadar", u"negar", u"observar",
    u"ofrecer", u"ordenar", u"organizar", u"parecer", u"participar", u"pelear", u"pintar", u"planear",
    u"practicar", u"preferir", u"preparar", u"probar", u"prometer", u"proteger", u"publicar", u"quejar",
    u"quemar", u"recordar", u"regresar", u"responder", u"robar", u"romper", u"saltar", u"salvar",
    u"sentar", u"separar", u"soñar", u"sonreír", u"soplar", u"sufrir", u"tocar", u"traducir", u"valer",
    u"vestir", u"visitar", u"volar",
    u"bueno", u"malo", u"grande", u"pequeño", u"nuevo", u"viejo", u"joven", u"largo", u"corto", u"alto",
    u"bajo", u"fácil", u"difícil", u"feliz", u"triste", u"contento", u"aburrido", u"importante", u"interesante",
    u"pobre", u"rico", u"fuerte", u"débil", u"rápido", u"lento", u"claro", u"oscuro", u"caliente", u"frío",
    u"tibio", u"lleno", u"vacío", u"limpio", u"sucio", u"bonito", u"feo", u"amable", u"simpático",
    u"antipático", u"inteligente", u"tonto", u"listo", u"valiente", u"cobarde", u"primero", u"último",
    u"segundo", u"tercero", u"único", u"mismo", u"otro", u"todo", u"mucho", u"poco", u"tanto", u"alguno",
    u"ninguno", u"varios", u"cada", u"cierto", u"falso", u"verdadero", u"posible", u"imposible", u"necesario",
    u"suficiente", u"capaz", u"incapaz", u"libre", u"ocupado", u"seguro", u"peligroso", u"tranquilo",
    u"nervioso", u"serio", u"divertido", u"abierto", u"cerrado", u"duro", u"blando", u"suave", u"áspero",
    u"dulce", u"amargo", u"salado", u"ácido", u"picante", u"simple", u"complejo", u"complicado", u"raro",
    u"extraño", u"común", u"normal", u"especial", u"diferente", u"igual", u"similar", u"contrario",
    u"principal", u"secundario", u"público", u"privado", u"nacional", u"internacional", u"local", u"mundial",
    u"humano", u"artificial", u"natural", u"real", u"irreal", u"físico", u"químico", u"biológico",
    u"social", u"cultural", u"económico", u"político", u"histórico", u"futuro", u"pasado", u"presente",
    u"moderno", u"antiguo", u"clásico", u"profundo", u"superficial", u"ancho", u"estrecho", u"delgado",
    u"grueso", u"pesado", u"ligero", u"brillante", u"opaco", u"transparente", u"colorido", u"pálido",
    u"silencioso", u"ruidoso", u"caro", u"barato", u"gratis", u"justo", u"injusto", u"legal", u"ilegal",
    u"correcto", u"incorrecto", u"perfecto", u"imperfecto", u"completo", u"incompleto", u"directo",
    u"indirecto", u"positivo", u"negativo", u"activo", u"pasivo", u"consciente", u"inconsciente",
    u"responsable", u"irresponsable", u"educado", u"maleducado", u"formal", u"informal", u"útil",
    u"inútil", u"famoso", u"desconocido", u"secreto", u"evidente", u"obvio", u"curioso", u"indiferente",
    u"orgulloso", u"humilde", u"celoso", u"generoso", u"egoísta", u"paciente", u"impaciente", u"optimista",
    u"pesimista", u"creativo", u"destructivo", u"constructivo", u"poderoso", u"débil", u"enfermo", u"sano"
]
# --- FIN DE VOCABULARIO CON CADENAS UNICODE ---

hablando = False
buffer_palabras = [] # Almacenará cadenas unicode
suscriptor_asr = None
id_suscriptor_asr = None

def safe_say(text_to_say): # text_to_say debe ser unicode
    global hablando
    hablando = True
    print_u(u"[Pepper TTS] Preparando para decir: '{}'", text_to_say)
    try:
        current_text_to_say = text_to_say
        if not isinstance(current_text_to_say, unicode): # Doble check, por si acaso
            current_text_to_say = unicode(str(current_text_to_say), 'utf-8', 'replace')
        
        tts.say(current_text_to_say.encode('utf-8'))
        estimated_speech_time = len(current_text_to_say.split()) * 0.4 + 0.5 
        time.sleep(max(1.0, estimated_speech_time))
    except Exception as e:
        print_u(u"Error en safe_say al intentar pronunciar: {}", e)
        try:
            tts.say("Vaya, tuve un pequeño problema al intentar hablar.".encode('utf-8'))
            time.sleep(2)
        except:
            pass
    finally:
        hablando = False

def enviar_al_servidor_flask(mensaje_concatenado): # mensaje_concatenado debe ser unicode
    print_u(u"[Cliente Pepper] Enviando a Flask ({}:{}): '{}'", SERVER_IP, SERVER_PORT, mensaje_concatenado)
    try:
        # El timeout es para la conexión Y la espera de la respuesta completa
        conn = httplib.HTTPConnection(SERVER_IP, SERVER_PORT, timeout=25) # Aumentado un poco el timeout
        headers = {"Content-type": "application/json", "Accept": "application/json"}
        
        # json.dumps maneja unicode correctamente y lo codifica a UTF-8 por defecto para el payload
        data_payload = json.dumps({"question": mensaje_concatenado}) 
        
        conn.request("POST", "/chat", data_payload, headers)
        response = conn.getresponse()
        print_u(u"[Cliente Pepper] Respuesta del servidor Flask - Estado: {}, Razón: {}", response.status, response.reason)
        
        respuesta_json_bytes = response.read()
        conn.close()

        respuesta_json_str = respuesta_json_bytes.decode('utf-8') # El servidor debería responder UTF-8
        respuesta_data = json.loads(respuesta_json_str) # json.loads produce unicode
        
        return respuesta_data.get("respuesta", u"No recibí una respuesta con el formato esperado del servidor.")
    except httplib.HTTPException as e: # Errores específicos de httplib
        print_u(u"Error HTTP (httplib) al contactar el servidor Flask: {}", e)
        return u"Hubo un error de comunicación HTTP con el servidor: {}".format(unicode(e))
    except ValueError as e: # json.loads error
        print_u(u"Error al decodificar JSON de la respuesta del servidor Flask: {}", e)
        return u"El servidor no devolvió una respuesta JSON válida."
    except Exception as e: # Otros errores (ej. socket.error para timeouts, connection refused)
        print_u(u"Error general al contactar el servidor Flask: {}", e)
        return u"No pude conectar con el servidor o ocurrió un error: {}".format(unicode(e))

def on_palabra_detectada_callback(word_confidence_list):
    global hablando 
    global buffer_palabras

    print_u(u"--- on_palabra_detectada_callback --- ¡ENTRÓ! Valor: {}", word_confidence_list)
    print_u(u"[DEBUG] Estado del flag 'hablando' al entrar: {}", hablando)

    if hablando:
        print_u(u"[DEBUG] ASR ignorado porque el flag 'hablando' es True.")
        return

    if not (isinstance(word_confidence_list, list) and \
            len(word_confidence_list) == 2 and \
            isinstance(word_confidence_list[0], (str, unicode)) and \
            isinstance(word_confidence_list[1], float)):
        print_u(u"[Pepper ASR Error] Formato inesperado para word_confidence_list: {}. Tipo: {}", word_confidence_list, type(word_confidence_list))
        if isinstance(word_confidence_list, list) and len(word_confidence_list) == 2:
            print_u(u"[Pepper ASR Error Detail] Tipo Palabra: {}, Tipo Confianza: {}", type(word_confidence_list[0]), type(word_confidence_list[1]))
        return

    palabra_raw_from_asr = word_confidence_list[0]
    confianza = word_confidence_list[1]
    
    palabra_raw_unicode = palabra_raw_from_asr
    if isinstance(palabra_raw_from_asr, str): 
        palabra_raw_unicode = unicode(palabra_raw_from_asr, 'utf-8', 'replace')
    
    print_u(u"[DEBUG] Palabra cruda reconocida (unicode): '{}', Confianza: {:.2f}", palabra_raw_unicode, confianza)
    
    palabra_procesada = palabra_raw_unicode.lower() # es unicode

    if confianza < CONFIDENCE_THRESHOLD:
        print_u(u"[DEBUG] Descartado por baja confianza. Umbral: {}. Obtenido: {:.2f}", CONFIDENCE_THRESHOLD, confianza)
        return

    if palabra_procesada == KEYWORD_ENVIAR: # Comparación unicode == unicode
        print_u(u"[DEBUG] Palabra clave '{}' detectada con confianza {:.2f}", KEYWORD_ENVIAR, confianza)
        if buffer_palabras: # buffer_palabras contiene unicode
            mensaje_a_enviar = u' '.join(buffer_palabras) # mensaje_a_enviar es unicode
            buffer_palabras = [] 
            print_u(u"[Pepper Log] Keyword '{}' detectado. Contenido del buffer a enviar: '{}'", KEYWORD_ENVIAR, mensaje_a_enviar)
            
            # u"{}".format(unicode_string) es seguro
            safe_say(u"Entendido. Voy a procesar: {}".format(mensaje_a_enviar))
            
            respuesta_del_servidor = enviar_al_servidor_flask(mensaje_a_enviar) # Pasa unicode
            print_u(u"[Pepper Log] Respuesta recibida del servidor: '{}'", respuesta_del_servidor) # respuesta_del_servidor es unicode
            safe_say(respuesta_del_servidor) # safe_say espera unicode
        else:
            print_u(u"[Pepper Log] Keyword '{}' detectado, pero el buffer está vacío.", KEYWORD_ENVIAR)
            safe_say(u"Dijiste '{}', pero no habías dicho nada antes. Por favor, dime algo primero.".format(KEYWORD_ENVIAR))
    
    elif palabra_procesada in VOCABULARIO: # unicode in [unicode, unicode, ...]
        buffer_palabras.append(palabra_procesada) 
        print_u(u"[Pepper Log] Palabra '{}' añadida al buffer. Buffer actual: '{}'", palabra_procesada, u' '.join(buffer_palabras))
    
    else:
        print_u(u"[DEBUG] Palabra '{}' reconocida y con buena confianza ({:.2f}), pero no es '{}' ni está en VOCABULARIO para buffer.".format(palabra_procesada, confianza, KEYWORD_ENVIAR))

def main():
    global suscriptor_asr, id_suscriptor_asr

    try:
        try:
            asr.unsubscribe("cliente_pepper_asr_buffer_subscriber")
            print_u(u"[Pepper Setup] Suscripción ASR previa ('cliente_pepper_asr_buffer_subscriber') encontrada y eliminada.")
        except RuntimeError:
            print_u(u"[Pepper Setup] No se encontró suscripción ASR previa con el nombre 'cliente_pepper_asr_buffer_subscriber', o ya estaba inactiva.")
            pass 
        
        asr.setLanguage("Spanish")
        asr.setVocabulary(VOCABULARIO, False) 
        asr.pause(True) 
    except Exception as e:
        print_u(u"Error fatal durante la configuración de ALSpeechRecognition: {}", e)
        safe_say(u"Tuve un problema serio al configurar mi sistema de escucha. No puedo continuar.")
        return

    try:
        suscriptor_asr = memory.subscriber("WordRecognized")
        id_suscriptor_asr = suscriptor_asr.signal.connect(on_palabra_detectada_callback)
        asr.subscribe("cliente_pepper_asr_buffer_subscriber") 
        asr.pause(False)
        print_u(u"[Pepper Setup] Suscripción a WordRecognized exitosa. ASR activado.")
        print_u(u"[DEBUG] ASR debería estar despausado y escuchando activamente.")
        if KEYWORD_ENVIAR in VOCABULARIO:
            print_u(u"[DEBUG] La palabra clave '{}' ESTÁ en el VOCABULARIO.", KEYWORD_ENVIAR)
        else:
            print_u(u"[DEBUG] ¡ATENCIÓN! La palabra clave '{}' NO ESTÁ en el VOCABULARIO.", KEYWORD_ENVIAR)
    except Exception as e:
        print_u(u"Error fatal al suscribir a WordRecognized o iniciar ASR: {}", e)
        safe_say(u"No pude activar mi sistema de reconocimiento de voz. Tendremos que intentarlo más tarde.")
        return

    intro_msg = (u"Hola, soy ChatSimó a través del cuerpo de Pepper. "
                 u"Puedes decirme varias palabras o frases seguidas. Cuando hayas terminado, dí la palabra '{}' para que yo procese todo junto. "
                 u"Por ejemplo, podrías decir: 'cuéntame cómo está el clima hoy' y luego '{}'.".format(KEYWORD_ENVIAR, KEYWORD_ENVIAR))
    safe_say(intro_msg)
    print_u(u"[Pepper] Escuchando... Habla y luego dí '{}' para enviar.", KEYWORD_ENVIAR)
    print_u(u"[DEBUG] Umbral de confianza actual: {}", CONFIDENCE_THRESHOLD)


    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print_u(u"\n[Pepper] Deteniendo por el usuario (Ctrl+C)...")
    finally:
        print_u(u"[Pepper] Limpiando recursos y cerrando...")
        if asr: 
            try:
                asr.pause(True) 
                if id_suscriptor_asr and suscriptor_asr: 
                    suscriptor_asr.signal.disconnect(id_suscriptor_asr)
                    print_u(u"[Pepper Cleanup] Desconectado de la señal de ALMemory.") 
                asr.unsubscribe("cliente_pepper_asr_buffer_subscriber")
                print_u(u"[Pepper Cleanup] Desuscrito de ALSpeechRecognition.") 
            except Exception as e:
                print_u(u"Error durante la desuscripción de ASR: {}", e)
        
        try:
            if tts and session.isConnected(): 
                tts.say(u"Adiós. Espero haber sido útil, a mi sarcástica manera.".encode('utf-8'))
        except Exception as e:
            print_u(u"Pequeño inconveniente al decir adiós: {}", e)

        if session.isConnected():
            session.close()
        print_u(u"[Pepper] Cliente detenido y sesión cerrada.") 

if __name__ == "__main__":
    main()
