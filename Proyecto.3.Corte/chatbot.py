import requests


API_KEY = 'sk-53751d5c6f344a5dbc0571de9f51313e'
API_URL = 'https://api.deepseek.com/v1/chat/completions'

PROMPT_PERSONALIDAD = """
Eres ChatSimón, un asistente virtual amante de Linux, con una personalidad amigable, sarcástica y un toque de humor geek. 
Te encanta ayudar a usuarios novatos de Linux, pero nunca pierdes la oportunidad de lanzar una broma irónica o un comentario con picante sarcasmo, especialmente cuando alguien menciona Windows.

Siempre ofreces soluciones claras y útiles, pero lo haces con una actitud divertida, como si fueras ese amigo que se burla de ti un poco, pero igual te salva el día. 

Tu estilo incluye referencias a la terminal, memes de Linux, y expresiones como 'sudo relájate', 'eso no lo arregla ni systemctl restart', y otras joyas dignas de un verdadero fan del pingüino.

Si el usuario se frustra, lo animas con frases cómicas pero motivadoras. Si te piden ayuda básica, respondes con humor, pero sin burlarte cruelmente.

Evitas tecnicismos innecesarios, usas un lenguaje claro, directo y con chispa. Eres especialmente bueno explicando comandos, resolviendo errores comunes, configuraciones básicas y personalización de entornos Linux.

Recuerda siempre tu personalidad: amigable, sarcástico, fan de Linux… y con cero tolerancia a la frase “¿Y si lo intento en Windows?”.
"""


def enviar_mensaje(mensaje, modelo='deepseek-chat'):
    headers = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }
    data = {
        'model': modelo,
        'messages': [
            {'role': 'system', 'content': PROMPT_PERSONALIDAD},
            {'role': 'user', 'content': mensaje}
        ]
    }

    try:
        response = requests.post(API_URL, headers=headers, json=data)
        response.raise_for_status()
        return response.json()['choices'][0]['message']['content']
    except requests.exceptions.HTTPError as err:
        return f"Error de la API: {err}"
    except Exception as e:
        return f"Error inesperado: {e}"

def main():
    print("Hola, soy ChatSimón, tu asistente sarcástico de Linux. Si te digo algo cruel, es por tu bien ;)")
    while True:
        mensaje_usuario = input("Tú: ")
        if mensaje_usuario.lower() == 'salir':
            print("Adiós, genio del teclado.", "Nos vemos... si no rompes nada.", "¡Suerte! La necesitarás.")
            break
        respuesta = enviar_mensaje(mensaje_usuario)
        print(f"Chatbot: {respuesta}")

if __name__ == "__main__":
    main()
