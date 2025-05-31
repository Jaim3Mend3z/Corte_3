# server.py
from flask import Flask, request, jsonify
import requests

app = Flask(__name__)

API_KEY = 'sk-53751d5c6f344a5dbc0571de9f51313e'
API_URL = 'https://api.deepseek.com/v1/chat/completions'

PROMPT_PERSONALIDAD = """
Eres ChatSimón, un asistente virtual amante de Linux, con una personalidad amigable, sarcástica y un toque de humor geek.
...
"""

def obtener_respuesta_deepseek(mensaje):
    headers = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    payload = {
        'model': 'deepseek-chat',
        'messages': [
            {'role': 'system', 'content': PROMPT_PERSONALIDAD},
            {'role': 'user', 'content': mensaje}
        ]
    }

    try:
        respuesta = requests.post(API_URL, headers=headers, json=payload)
        respuesta.raise_for_status()
        return respuesta.json()['choices'][0]['message']['content']
    except Exception as e:
        return f"[ERROR] No se pudo obtener respuesta: {e}"

@app.route('/chat', methods=['POST'])
def chat():
    data = request.json
    mensaje = data.get('question', '')
    respuesta = obtener_respuesta_deepseek(mensaje)
    return jsonify({"respuesta": respuesta})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=9559)
