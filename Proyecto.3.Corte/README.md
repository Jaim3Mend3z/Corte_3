# Proyecto Final: Pepper como Asistente Linux Interactivo con DeepSeek API

## 1. Resumen del Proyecto (Reto)

Este proyecto se desarrolló como parte de la fase final del curso, con el objetivo de integrar el robot Pepper con la API de DeepSeek para actuar como un asistente de línea de comandos Linux con una personalidad definida (amigable, sarcástica y con humor geek). El sistema permite al usuario interactuar por voz con Pepper, quien acumula las frases del usuario y, al detectar una palabra clave ("enviar"), consulta a la IA de DeepSeek a través de un servidor intermediario para obtener una respuesta coherente y luego la verbaliza.

Los principales desafíos abordados incluyeron:
* Configuración del reconocimiento de voz en Pepper para español.
* Implementación de una comunicación cliente-servidor entre Pepper y un PC.
* Integración con la API externa de DeepSeek.
* Manejo robusto de cadenas de texto Unicode en el entorno Python 2 de Pepper.
* Desarrollo de una lógica de conversación para acumular entradas y procesarlas con una palabra clave.
* Depuración de errores de red, API y lógica de programación.

## 2. Arquitectura de la Solución

La solución implementada se compone de tres módulos principales:

1.  **Cliente Pepper (`cliente_pepper_final.py`):**
    * Se ejecuta directamente en el robot Pepper (Naoqi OS, Python 2.7).
    * **Funciones:**
        * Captura de audio y reconocimiento de voz (`ALSpeechRecognition`).
        * Almacenamiento temporal de las frases reconocidas en un buffer.
        * Detección de la palabra clave "enviar" para finalizar la entrada de voz.
        * Envío del contenido del buffer al servidor Flask mediante una petición HTTP POST.
        * Recepción de la respuesta JSON del servidor.
        * Verbalización de la respuesta de la IA usando `ALAnimatedSpeech`.

2.  **Servidor Intermediario Flask (`server.py`):**
    * Se ejecuta en el PC del estudiante (Python 3.x).
    * **Funciones:**
        * Escucha peticiones HTTP POST en una ruta específica (ej. `/chat`).
        * Recibe el texto del cliente Pepper.
        * Prepara y envía una solicitud a la API de DeepSeek, incluyendo el prompt de personalidad y el texto del usuario.
        * Maneja la respuesta de la API de DeepSeek.
        * Devuelve la respuesta de la IA al cliente Pepper en formato JSON.

3.  **API de DeepSeek:**
    * Servicio externo de inteligencia artificial generativa.
    * **Funciones:**
        * Procesa el prompt del sistema (personalidad de "ChatSimón") y el mensaje del usuario.
        * Genera una respuesta de texto coherente.

**Diagrama de Flujo:**
┌─────────────────┐      HTTP POST      ┌─────────────────┐      API Call      ┌────────────────┐
│ Cliente Pepper  │─────────────────────►│ Servidor Flask  │────────────────────►│ API DeepSeek   │
│ (Python 2.7)    │      (question)     │ (Python 3.x)    │     (prompt + msg) │ (IA Externa)   │
│                 │◀─────────────────────│                 │◀────────────────────│                │
└─────────────────┘    HTTP Response    └─────────────────┘   API Response     └────────────────┘
(respuesta)                           (texto IA)


## 3. Requisitos Previos

* Robot Pepper operativo.
* PC con Python 3.x y conexión a la misma red que Pepper.
* API Key válida de DeepSeek ([https://platform.deepseek.com/](https://platform.deepseek.com/)).
* Software:
    * En el PC: `Flask`, `requests` (instalables vía `pip`).
    * En Pepper: No se requieren instalaciones adicionales más allá del entorno Naoqi.

## 4. Configuración del Entorno

### 4.1. Servidor Flask (PC del Estudiante)
1.  **Crear Entorno Virtual (Recomendado):**
    ```bash
    python3 -m venv venv_pepper_server
    source venv_pepper_server/bin/activate  # En Linux/macOS
    # venv_pepper_server\Scripts\activate   # En Windows
    ```
2.  **Instalar Dependencias:**
    ```bash
    pip install Flask requests
    ```
3.  **Configurar `server.py`:**
    * Reemplazar `API_KEY = 'sk-...'` con la API Key real de DeepSeek. Por seguridad, en un entorno de producción, esto debería manejarse con variables de entorno.
    * Asegurar que `app.run(host='0.0.0.0', port=XXXX)` utilice un puerto `XXXX` que esté libre y sea accesible desde Pepper (ej. `9559` si así se decidió, o `5000`). `0.0.0.0` permite conexiones desde cualquier IP en la red.

### 4.2. Cliente Pepper
1.  **Transferir Script a Pepper:** Copiar el archivo `cliente_pepper_final.py` al robot Pepper (ej. usando `scp` o Choregraphe).
2.  **Configurar `cliente_pepper_final.py`:**
    * Verificar que `SERVER_IP` sea la dirección IP del PC donde corre el servidor Flask.
    * Asegurar que `SERVER_PORT` coincida exactamente con el puerto configurado en `server.py`.

## 5. Estructura de Archivos del Proyecto (Ejemplo)

/
├── cliente_pepper_final.py  # Script para Pepper
├── server.py                # Script del servidor Flask
├── requirements.txt         # (Opcional) Lista de dependencias para el servidor
└── readme.md                # Este archivo


## 6. Paso a Paso Detallado: Formulación de la Solución y Resolución de Problemas

El desarrollo de este proyecto fue un proceso iterativo de implementación, prueba y depuración. A continuación, se describe el camino seguido:

### 6.1. Configuración Inicial y Conexión (Pepper)
1.  **Objetivo:** Establecer la comunicación básica con los módulos de Pepper.
2.  **Implementación:** Se utilizó la librería `qi` para conectar con `ALSpeechRecognition`, `ALMemory` y `ALAnimatedSpeech`.
3.  **Desafío:** Asegurar la correcta conexión al Naoqi de Pepper (`tcp://127.0.0.1:9559` ya que el script corre en el robot).

### 6.2. Reconocimiento de Voz Básico (Pepper)
1.  **Objetivo:** Hacer que Pepper reconociera palabras en español.
2.  **Implementación:**
    * Se definió una lista `VOCABULARIO` inicial.
    * Se configuró `ALSpeechRecognition.setLanguage("Spanish")`.
    * Se usó `ALSpeechRecognition.setVocabulary(VOCABULARIO, False)` para restringir el reconocimiento a las palabras definidas.
    * Se creó una función callback (`on_palabra_detectada_callback`) y se suscribió al evento `WordRecognized` de `ALMemory`.
3.  **Pruebas y Ajustes:** Se verificó que el callback se activara y que las palabras fueran detectadas.

### 6.3. Creación del Servidor Flask Intermediario (PC)
1.  **Objetivo:** Tener un endpoint que Pepper pudiera contactar.
2.  **Implementación (`server.py`):**
    * Se creó una aplicación Flask simple.
    * Se definió una ruta `/chat` que aceptara peticiones `POST`.
    * Inicialmente, esta ruta solo recibía el JSON, extraía la pregunta y devolvía una respuesta fija en formato JSON (`{"respuesta": "..."}`).
3.  **Pruebas:** Se usó `curl` o Postman para enviar datos al servidor Flask y verificar su funcionamiento antes de integrar con Pepper.

### 6.4. Comunicación Pepper -> Flask
1.  **Objetivo:** Enviar la palabra reconocida por Pepper al servidor Flask.
2.  **Implementación (`cliente_pepper_final.py`):**
    * Se creó la función `enviar_al_servidor_flask` usando `httplib` (Python 2).
    * Se manejó la creación de la conexión, el envío de headers (`Content-type: application/json`) y el payload JSON (`{"question": mensaje}`).
    * Se procesó la respuesta del servidor.
3.  **Desafíos y Soluciones:**
    * **Formato de Datos:** Asegurar que Pepper enviara los datos en el formato JSON esperado por Flask.
    * **Errores de Red:** Implementar manejo básico de excepciones para problemas de conexión.

### 6.5. Integración de DeepSeek API en el Servidor Flask
1.  **Objetivo:** Que el servidor Flask consultara a DeepSeek con el mensaje recibido de Pepper.
2.  **Implementación (`server.py`):**
    * Se añadió la función `obtener_respuesta_deepseek` que:
        * Incluye la `API_KEY` y la `API_URL` de DeepSeek.
        * Define el `PROMPT_PERSONALIDAD` para "ChatSimón".
        * Construye el payload para la API de DeepSeek (`model`, `messages` con roles `system` y `user`).
        * Usa la librería `requests` para hacer la llamada POST a DeepSeek.
        * Extrae el contenido de la respuesta de la IA.
        * Maneja errores si la llamada a DeepSeek falla.
    * La ruta `/chat` se actualizó para llamar a `obtener_respuesta_deepseek` y devolver su resultado.

### 6.6. Manejo de Unicode (Iteraciones y Correcciones Clave)
Este fue un desafío recurrente y multifacético:
1.  **TTS y Caracteres Especiales (`UnicodeEncodeError`):**
    * **Problema:** Pepper fallaba al intentar decir respuestas de DeepSeek con tildes/eñes.
    * **Solución:** En `safe_say`, se aseguró que la cadena `current_text_to_say` fuera Unicode y luego se codificara a UTF-8 antes de pasarla a `tts.say()`: `tts.say(current_text_to_say.encode('utf-8'))`.
2.  **Impresión en Consola (`UnicodeEncodeError`):**
    * **Problema:** `print u"Texto con áéíóú"` fallaba.
    * **Solución:** Se implementó la función `print_u` que toma el texto, lo asegura como Unicode, y lo imprime codificado en UTF-8. Se reemplazaron todas las llamadas `print` relevantes.
3.  **Comparaciones en `VOCABULARIO` (`UnicodeWarning`):**
    * **Problema:** `u"música" in ["música"]` (Unicode vs. byte string) causaba advertencias y fallos.
    * **Solución:** Se convirtió toda la lista `VOCABULARIO` (y `KEYWORD_ENVIAR`) para que contuviera explícitamente cadenas Unicode (ej. `u"música"`).
4.  **Formateo de Cadenas con Bytes UTF-8 (`UnicodeDecodeError`):**
    * **Problema:** Al formar `mensaje_a_enviar` con palabras que ASR devolvía como bytes UTF-8 (ej. `'m\xc3\xbasica'`), la operación `u"Procesando: {}".format(mensaje_bytes)` fallaba porque Python 2 intentaba decodificar `mensaje_bytes` con ASCII.
    * **Solución:** En `on_palabra_detectada_callback`, `palabra_raw_from_asr` se convirtió explícitamente a Unicode (`unicode(palabra_raw_from_asr, 'utf-8', 'replace')`) antes de cualquier otro procesamiento. Esto aseguró que `buffer_palabras` siempre contuviera Unicode.
5.  **Formato de Floats en `print_u` (`Unknown format code 'f'`):**
    * **Problema:** La primera versión de `print_u` convertía los números a cadenas Unicode antes de `format`, rompiendo formatos como `:.2f`.
    * **Solución:** Se ajustó `print_u` para que los números se pasaran directamente a `.format()`, permitiendo que los especificadores numéricos funcionaran.

### 6.7. Implementación de Buffer de Palabras y Palabra Clave "enviar"
1.  **Objetivo:** Permitir al usuario hablar varias frases antes de que Pepper procese la entrada completa.
2.  **Implementación:**
    * `buffer_palabras = []`: Lista global para acumular las palabras (Unicode).
    * `KEYWORD_ENVIAR = u"enviar"`: Palabra clave para activar el envío.
    * En `on_palabra_detectada_callback`:
        * Si la palabra reconocida (Unicode, minúsculas) es `KEYWORD_ENVIAR`: se unen las palabras del buffer, se limpia el buffer, y se envía el mensaje.
        * Si es otra palabra del `VOCABULARIO`: se añade al buffer.
3.  **Feedback:** Se añadieron mensajes de Pepper como "Entendido. Voy a procesar..."

### 6.8. Depuración de la Lógica del Callback y API Naoqi
1.  **`AttributeError` con `isRecognitionRunning`:** Se eliminó una llamada incorrecta a un método inexistente de `ALSpeechRecognition`. La limpieza de suscripciones se manejó con `try-except`.
2.  **Parseo del Evento `WordRecognized`:** Se corrigió la forma en que se accedía a la palabra y la confianza desde el valor proporcionado por el evento `WordRecognized`. El callback recibe directamente la lista `[palabra, confianza]`.

### 6.9. Diagnóstico y Solución del Problema de "Timeout"
1.  **Verificación del Puerto del Servidor Flask:** Fue crucial confirmar que el servidor Flask del estudiante estaba, de hecho, corriendo en el puerto `9559`. La variable `SERVER_PORT` en el script de Pepper se ajustó a `9559`.
2.  **Identificación de Latencia de DeepSeek:** La causa más probable del `timeout` (incluso con puertos correctos) se identificó como el tiempo que tarda la API de DeepSeek en responder. Si esta respuesta excede el timeout del cliente Pepper (`httplib`), Pepper reporta un error.
3.  **Medidas Tomadas/Sugeridas:**
    * Se recomendó añadir logging de tiempos en `server.py` para medir la duración de la llamada a DeepSeek.
    * Se añadió un `timeout` a la llamada `requests.post` a DeepSeek dentro de `server.py`.
    * Se aumentó ligeramente el `timeout` en `httplib.HTTPConnection` en el script de Pepper (a 25 segundos).

### 6.10. Ajustes Finales y Pruebas
Se realizaron pruebas exhaustivas, ajustando el `CONFIDENCE_THRESHOLD` (bajándolo a `0.15` para facilitar la depuración del reconocimiento y luego recomendando subirlo), verificando los logs de Pepper y del servidor Flask, hasta alcanzar un comportamiento estable y deseado.

## 7. Cómo Ejecutar la Solución

1.  **Iniciar el Servidor Flask (PC):**
    * Asegurar que `server.py` está configurado con la API Key de DeepSeek y el puerto correcto (ej. `9559`).
    * Activar el entorno virtual (si se usa).
    * Ejecutar: `python3 server.py`.
    * Verificar en la consola que está escuchando en `0.0.0.0:PUERTO_CONFIGURADO`.

2.  **Ejecutar el Cliente Pepper:**
    * Asegurar que Pepper esté conectado a la misma red que el PC.
    * Verificar que `SERVER_IP` y `SERVER_PORT` en `cliente_pepper_final.py` coincidan con la configuración del servidor Flask.
    * Ejecutar el script en Pepper: `python cliente_pepper_final.py`.

3.  **Interactuar:**
    * Pepper dará su mensaje de bienvenida.
    * Hablarle a Pepper. Las palabras reconocidas (y en el vocabulario) se acumularán.
    * Decir "enviar" para que Pepper procese la entrada acumulada.
    * Pepper confirmará, contactará al servidor, y luego dirá la respuesta de ChatSimón.

## 8. Posibles Mejoras y Futuras Líneas de Trabajo

* **Gestión Avanzada de Errores y Reintentos:** Mejorar la robustez ante fallos temporales de red o de la API de DeepSeek.
* **Detección de Fin de Frase Más Natural:** Explorar alternativas a la palabra clave, como la detección de silencio o pausas largas.
* **Manejo de Contexto de Conversación:** Permitir que el servidor Flask o DeepSeek recuerden turnos anteriores de la conversación para respuestas más coherentes en diálogos largos.
* **Seguridad:** Externalizar la API Key del código (usar variables de entorno).
* **Interfaz Gráfica en Tablet:** Utilizar la tablet de Pepper para mostrar información adicional, como la transcripción o la respuesta.
* **Despliegue del Servidor Flask:** Para un uso más continuo, usar un servidor WSGI de producción (Gunicorn, uWSGI) en lugar del servidor de desarrollo de Flask.

## 9. Autores (Estudiantes)

* Jaime Andres Mendez Jimenez
---
