import tkinter as tk
import math
import time
import random

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.last_output = 0

    def compute(self, error, dt):
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        self.integral = max(-50, min(self.integral + error * dt, 50))
        raw_output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        smoothed_output = 0.3 * raw_output + 0.7 * self.last_output
        self.last_output = smoothed_output
        self.prev_error = error
        return smoothed_output

class LineFollowerCar:
    def __init__(self, canvas):
        self.canvas = canvas
        self.car_x = 400
        self.car_y = 500
        self.car_angle = 270
        self.speed = 1.5
        self.sensor_distance = 30
        self.sensor_offset = 20
        self.pid = PIDController(3, 0.00001, 0.00001)
        self.last_time = time.time()
        self.trail = []
        self.last_angle_change = 0
        self.max_turn_rate = 3.5

        self.body = self.create_car_body()
        self.left_wheel = self.create_wheel()
        self.right_wheel = self.create_wheel()
        self.sensor_left = self.create_sensor()
        self.sensor_right = self.create_sensor()
        self.update_car()

    def create_car_body(self):
        return self.canvas.create_polygon([0,0,50,0,50,30,0,30], fill='#2E86C1', outline='#1B4F72', width=2)
    def create_wheel(self):
        return self.canvas.create_oval(0,0,12,12, fill='#2C3E50', outline='#1B2631')
    def create_sensor(self):
        return self.canvas.create_oval(0,0,10,10, fill='#E74C3C', outline='#922B21', width=2)

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
        self._update_wheel(self.left_wheel, -10, angle)
        self._update_wheel(self.right_wheel, 10, angle)
        sl, sr = self.get_sensor_positions()
        self.canvas.coords(self.sensor_left, sl[0]-5, sl[1]-5, sl[0]+5, sl[1]+5)
        self.canvas.coords(self.sensor_right, sr[0]-5, sr[1]-5, sr[0]+5, sr[1]+5)

    def _update_wheel(self, wheel, offset, angle):
        x = self.car_x + offset * math.sin(angle)
        y = self.car_y - offset * math.cos(angle)
        self.canvas.coords(wheel, x-6, y-6, x+6, y+6)

    def get_sensor_positions(self):
        a = math.radians(self.car_angle)
        fx = self.car_x + self.sensor_distance * math.cos(a)
        fy = self.car_y + self.sensor_distance * math.sin(a)
        ls = (fx - self.sensor_offset * math.sin(a), fy + self.sensor_offset * math.cos(a))
        rs = (fx + self.sensor_offset * math.sin(a), fy - self.sensor_offset * math.cos(a))
        return ls, rs

    def check_sensor(self, x, y):
        overlap = self.canvas.find_overlapping(x-5, y-5, x+5, y+5)
        return guide_line in overlap

    def move(self):
        now = time.time()
        dt = now - self.last_time if now - self.last_time > 0 else 1e-3
        self.last_time = now

        sl, sr = self.get_sensor_positions()
        sc = ((sl[0]+sr[0])/2, (sl[1]+sr[1])/2)

        left = self.check_sensor(*sl)
        right = self.check_sensor(*sr)
        center = self.check_sensor(*sc)

        if left and right:
            error = 0
            self.speed = 1.6
        elif left:
            error = 3
            self.speed = 1.3
        elif right:
            error = -3
            self.speed = 1.3
        elif center:
            error = 0.5 * self.pid.prev_error
            self.speed = 1.2
        else:
            error = 0
            self.speed = 0.8
            if time.time() - self.last_angle_change > 0.4:
                self.car_angle += 3 if int(time.time()*2)%2==0 else -3
                self.last_angle_change = time.time()

        steering = self.pid.compute(error, dt)
        steering = max(-self.max_turn_rate, min(steering, self.max_turn_rate))
        self.car_angle += steering

        rad = math.radians(self.car_angle)
        self.car_x = max(20, min(self.car_x + self.speed * math.cos(rad), 780))
        self.car_y = max(20, min(self.car_y + self.speed * math.sin(rad), 580))
        self.trail.append((self.car_x, self.car_y))
        if len(self.trail) > 100: self.trail.pop(0)
        self.update_car()

# Crear nueva pista en forma de "8"
def create_eight_track():
    path = []
    cx, cy = 400, 300
    r = 200
    for t in range(0, 360, 5):
        angle = math.radians(t)
        x = cx + r * math.sin(angle)
        y = cy - r * math.cos(angle)
        path.append((x, y))
    for t in range(0, 360, 5):
        angle = math.radians(t)
        x = cx + r * math.sin(angle)
        y = cy + 2*r - r * math.cos(angle)
        path.append((x, y))
    return [coord for point in path for coord in point]

# Tkinter setup
global guide_line
window = tk.Tk()
window.title("Seguidor de Línea - Pista en Forma de 8")
canvas = tk.Canvas(window, width=1920, height=1080)
canvas.pack()

# Fondo
canvas.create_rectangle(0, 0, 1920, 1080, fill='#A9DFBF')

# Crear pista
track = canvas.create_line(create_eight_track(), fill='#5D6D7E', width=28, smooth=True)
guide_line = canvas.create_line(create_eight_track(), fill='black', width=14, smooth=True)

# Inicializar auto
car = LineFollowerCar(canvas)

# Bucle principal
def game_loop():
    car.move()
    if random.random() > 0.4:
        canvas.create_oval(car.car_x-1, car.car_y-1, car.car_x+1, car.car_y+1,
                           fill='#F39C12', outline='')
    window.after(30, game_loop)

# Título
canvas.create_text(400, 30, text="Seguidor de Línea - Pista en Forma de 8 - Jaime Mendez", font=("Arial", 14, "bold"))
game_loop()
window.mainloop()
