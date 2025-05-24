import requests

API_KEY = 'sk-53751d5c6f344a5dbc0571de9f51313e'
API_URL = 'https://api.deepseek.com/v1/chat/completions'

def generar_prompt_interactivo():
    """Recolecta parámetros para la historia y construye el prompt"""
    print("\n=== Configura tu historia ===")
    genero = input("Género (ej. fantasía, sci-fi, misterio): ")
    protagonista = input("Protagonista (ej. princesa guerrera, robot poeta): ")
    escenario = input("Escenario (ej. ciudad flotante, bosque de cristal): ")
    tono = input("Tono (ej. épico, oscuro, humorístico): ")
    elementos = input("Elementos especiales (ej. magia, tecnología alienígena): ")

    return f"""Escribe un cuento creativo con estos elementos:
- Género: {genero}
- Protagonista: {protagonista}
- Escenario: {escenario}
- Tono: {tono}
- Incluir: {elementos}
- Estructura: introducción, conflicto, desenlace
- Longitud: 500 palabras aproximadamente
- Estilo narrativo: descriptivo y envolvente
Incluye título creativo y marca los diálogos con guiones."""

def generar_cuento(prompt, temperatura=0.85):
    """Genera la historia con parámetros creativos"""
    headers = {
        'Authorization': f'Bearer {API_KEY}',
        'Content-Type': 'application/json'
    }

    data = {
        'model': 'deepseek-chat',
        'temperature': temperatura,
        'messages': [{
            'role': 'user',
            'content': prompt
        }]
    }

    try:
        response = requests.post(API_URL, headers=headers, json=data)
        response.raise_for_status()
        return response.json()['choices'][0]['message']['content']

    except requests.exceptions.HTTPError as err:
        return f"Error: {err.response.text}"
    except Exception as e:
        return f"Error: {e}"

def menu_modificaciones():
    """Ofrece opciones para modificar la historia generada"""
    print("\n¿Qué deseas hacer ahora?")
    print("1. Agregar un giro dramático")
    print("2. Cambiar el final")
    print("3. Ampliar una escena")
    print("4. Generar nueva historia")
    print("5. Salir")
    return input("Elección: ")

def aplicar_modificacion(historia_original, opcion):
    """Crea nuevos prompts basados en modificaciones"""
    modificaciones = {
        '1': "Agrega un giro dramático inesperado a esta historia:",
        '2': "Reescribe el final de esta historia con un cambio radical:",
        '3': "Amplía la escena más importante con detalles descriptivos:"
    }
    return f"{modificaciones[opcion]}\n\n{historia_original}"

def main():
    print("""¡Bienvenido al Generador de Cuentos Creativos!
(escribe 'salir' en cualquier momento para terminar)""")

    while True:
        # Generar historia base
        prompt = generar_prompt_interactivo()
        historia = generar_cuento(prompt)
        print("\n=== TU CUENTO ===")
        print(historia)

        # Ciclo de modificaciones
        while True:
            opcion = menu_modificaciones()

            if opcion == '4':
                break  # Volver a generar nueva historia
            if opcion == '5':
                print("¡Hasta la próxima aventura!")
                return

            nuevo_prompt = aplicar_modificacion(historia, opcion)
            historia = generar_cuento(nuevo_prompt)
            print("\n=== HISTORIA MODIFICADA ===")
            print(historia)

if __name__ == "__main__":
    main()
