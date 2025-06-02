import tkinter as tk
import math
import time

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.last_output = 0

    def compute(self, error, dt):
        if dt <= 0:
            derivative = 0
        else:
            derivative = (error - self.prev_error) / dt
        self.integral = max(-50, min(self.integral + error * dt, 50))
        raw_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        smoothed_output = 0.4 * raw_output + 0.6 * self.last_output
        self.last_output = smoothed_output
        self.prev_error = error
        return smoothed_output

class LineFollowerCar:
    def __init__(self, canvas, start_x, start_y, start_angle):
        self.canvas = canvas
        self.car_x = start_x
        self.car_y = start_y
        self.car_angle = start_angle
        self.speed = 1.5
        self.sensor_distance = 30
        self.sensor_offset = 20
        self.pid = PIDController(3.5, 0.0001, 0.01)
        self.last_time = time.time()
        self.trail = []
        self.last_angle_change = 0
        self.max_turn_rate = 4.0
        self.stopping = False

        self.body = self.create_car_body()
        self.left_wheel = self.create_wheel()
        self.right_wheel = self.create_wheel()
        self.sensor_left = self.create_sensor()
        self.sensor_right = self.create_sensor()
        self.update_car()

    def create_car_body(self):
        return self.canvas.create_polygon([0,0,50,0,50,30,0,30], fill='#D32F2F', outline='#B71C1C', width=2)

    def create_wheel(self):
        return self.canvas.create_oval(0,0,12,12, fill='#212121', outline='#000000')

    def create_sensor(self):
        return self.canvas.create_oval(0,0,10,10, fill='#FFC107', outline='#FFA000', width=2)

    def update_car(self):
        angle = math.radians(self.car_angle)
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        coords = [
            self.car_x + 15*cos_a - 10*sin_a, self.car_y + 15*sin_a + 10*cos_a,
            self.car_x + 15*cos_a + 10*sin_a, self.car_y + 15*sin_a - 10*cos_a,
            self.car_x - 15*cos_a + 10*sin_a, self.car_y - 15*sin_a - 10*cos_a,
            self.car_x - 15*cos_a - 10*sin_a, self.car_y - 15*sin_a + 10*cos_a
        ]
        self.canvas.coords(self.body, *coords)
        self._update_wheel(self.left_wheel, -12, angle)
        self._update_wheel(self.right_wheel, 12, angle)
        sl, sr = self.get_sensor_positions()
        self.canvas.coords(self.sensor_left, sl[0]-5, sl[1]-5, sl[0]+5, sl[1]+5)
        self.canvas.coords(self.sensor_right, sr[0]-5, sr[1]-5, sr[0]+5, sr[1]+5)

    def _update_wheel(self, wheel, offset, angle):
        x = self.car_x - offset * math.sin(angle)
        y = self.car_y + offset * math.cos(angle)
        self.canvas.coords(wheel, x-6, y-6, x+6, y+6)

    def get_sensor_positions(self):
        a = math.radians(self.car_angle)
        fx = self.car_x + self.sensor_distance * math.cos(a)
        fy = self.car_y + self.sensor_distance * math.sin(a)
        ls = (fx - self.sensor_offset * math.sin(a), fy + self.sensor_offset * math.cos(a))
        rs = (fx + self.sensor_offset * math.sin(a), fy - self.sensor_offset * math.cos(a))
        return ls, rs

    def check_stop_bar(self, sl, sr):
        left_stop = self.check_sensor_stop_bar(*sl)
        right_stop = self.check_sensor_stop_bar(*sr)
        return left_stop and right_stop

    def check_sensor_stop_bar(self, x, y):
        overlap = self.canvas.find_overlapping(x-5, y-5, x+5, y+5)
        return stop_bar in overlap

    def move(self):
        if self.stopping:
            return

        now = time.time()
        dt = now - self.last_time if now - self.last_time > 0 else 1e-3
        self.last_time = now

        sl, sr = self.get_sensor_positions()

        if self.check_stop_bar(sl, sr):
            self.stopping = True
            self.canvas.create_text(self.car_x, self.car_y - 40,
                                      text="¡VUELTA COMPLETA!",
                                      font=("Arial", 16, "bold"),
                                      fill="green")
            return

        left_active = self.check_sensor(*sl)
        right_active = self.check_sensor(*sr)

        error = 0
        if left_active and not right_active:
            error = 3
            self.speed = 1.2
        elif not left_active and right_active:
            error = -3
            self.speed = 1.2
        elif left_active and right_active:
            error = 0
            self.speed = 2.0
        else:
            error = 4 * (1 if self.pid.prev_error > 0 else -1)
            self.speed = 1.0

        steering = self.pid.compute(error, dt)
        steering = max(-self.max_turn_rate, min(steering, self.max_turn_rate))
        self.car_angle += steering

        rad = math.radians(self.car_angle)
        self.car_x += self.speed * math.cos(rad)
        self.car_y += self.speed * math.sin(rad)

        self.car_x = max(10, min(self.car_x, 790))
        self.car_y = max(10, min(self.car_y, 590))

        self.update_car()

    def check_sensor(self, x, y):
        overlap = self.canvas.find_overlapping(x-5, y-5, x+5, y+5)
        return any(item == guide_line for item in overlap)

def create_track_path(stops):
    pts = []
    for i in range(len(stops)):
        p1 = stops[i]
        pts.extend([p1[0], p1[1]])
    pts.extend([stops[0][0], stops[0][1]])
    return pts

# --- Configuración de la Ventana y Pista ---
global guide_line, stop_bar
window = tk.Tk()
# Título de la ventana personalizado
window.title("Pista F1 - Jaime Mendez")
canvas = tk.Canvas(window, width=800, height=600, bg='#388E3C')
canvas.pack()

# Puntos para definir el circuito estilo Suzuka
stops = [
    (100, 500),  # Recta principal
    (150, 450),  # Curva 1
    (200, 400),  # Curva 2
    (250, 350),  # Curva 3
    (300, 300),  # Curva 4
    (350, 250),  # Curva 5
    (400, 200),  # Curva 6
    (450, 150),  # Curva 7
    (500, 100),  # Curva 8
    (550, 150),  # Curva 9
    (600, 200),  # Curva 10
    (650, 250),  # Curva 11
    (700, 300),  # Curva 12
    (750, 350),  # Curva 13
    (700, 400),  # Curva 14
    (650, 450),  # Curva 15
    (600, 500),  # Curva 16
    (550, 550),  # Curva 17
    (500, 500),  # Curva 18
    (450, 450),  # Curva 19
    (400, 400),  # Curva 20
    (350, 350),  # Curva 21
    (300, 400),  # Curva 22
    (250, 450),  # Curva 23
    (200, 500),  # Curva 24
    (150, 550),  # Curva 25
    (100, 500)   # Meta
]

track_path = create_track_path(stops)
canvas.create_line(track_path, fill='#546E7A', width=50, smooth=True, capstyle=tk.ROUND, joinstyle=tk.ROUND)
guide_line = canvas.create_line(track_path, fill='black', width=3, smooth=True, capstyle=tk.ROUND, joinstyle=tk.ROUND, dash=(10, 5))

# Línea de Meta
start_pos = stops[0]
stop_bar = canvas.create_rectangle(start_pos[0] - 30, start_pos[1] - 5, start_pos[0] + 30, start_pos[1] + 5,
                                   fill='white', outline='black', width=2)
for i in range(6):
    for j in range(2):
        if (i+j)%2 == 0:
            x1 = (start_pos[0] - 30) + i*10
            y1 = (start_pos[1] - 5) + j*5
            canvas.create_rectangle(x1, y1, x1+10, y1+5, fill='black', outline='')

car = LineFollowerCar(canvas, start_x=100, start_y=530, start_angle=0)

# --- TEXTOS AÑADIDOS ---
# Título en la parte superior
canvas.create_text(400, 30, text="Pista Formula 1", font=("Arial", 18, "bold"), fill="white")
# Nombre en la parte inferior
canvas.create_text(400, 580, text="Jaime Mendez - Universidad Santo Tomás", font=("Arial", 12, "italic"), fill="white")

# Loop principal del juego
def game_loop():
    car.move()
    window.after(20, game_loop)

game_loop()
window.mainloop()
