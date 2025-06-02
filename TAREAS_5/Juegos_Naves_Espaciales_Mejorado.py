import pygame
import random
import os
import math
import time

# Inicializar Pygame
pygame.init()
pygame.mixer.init()

# Configuración de pantalla
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Guerras Espaciales - por Jaime Mendez")

# --- FUNCIONES DE CARGA A PRUEBA DE FALLOS ---

class DummySound:
    def play(self):
        pass

def load_image(path, scale=None):
    try:
        image = pygame.image.load(path).convert_alpha()
        if scale:
            image = pygame.transform.scale(image, scale)
        return image
    except pygame.error:
        print(f"¡AVISO! No se pudo cargar la imagen: {path}")
        fallback_surface = pygame.Surface(scale or (40, 40))
        fallback_surface.fill((255, 0, 255))
        return fallback_surface

def load_sound(path):
    if not os.path.exists(path):
        print(f"¡AVISO! No se pudo encontrar el archivo de sonido: {path}")
        return DummySound()
    try:
        return pygame.mixer.Sound(path)
    except pygame.error:
        print(f"¡AVISO! No se pudo cargar el sonido (puede ser un formato incorrecto): {path}")
        return DummySound()

# --- Rutas y Carga de Assets ---
current_path = os.path.dirname(os.path.abspath(__file__))
assets_path = os.path.join(current_path, 'assets')
img_path = os.path.join(assets_path, 'images')
sound_path = os.path.join(assets_path, 'sounds')

player_img = load_image(os.path.join(img_path, 'player.png'))
enemy_imgs = [
    load_image(os.path.join(img_path, 'enemy1.png')),
    load_image(os.path.join(img_path, 'enemy2.png')),
    load_image(os.path.join(img_path, 'enemy3.png'))
]
bullet_img = load_image(os.path.join(img_path, 'bullet.png'))
enemy_bullet_img = load_image(os.path.join(img_path, 'enemy_bullet.png'))
background_img = load_image(os.path.join(img_path, 'background.jpg'))
background = pygame.transform.scale(background_img, (WIDTH, HEIGHT))

laser_sound = load_sound(os.path.join(sound_path, 'laser.mp3'))
enemy_laser_sound = load_sound(os.path.join(sound_path, 'enemy_laser.mp3'))
explosion_sound = load_sound(os.path.join(sound_path, 'explosion.mp3'))
try:
    pygame.mixer.music.load(os.path.join(sound_path, 'background_music.mp3'))
    pygame.mixer.music.set_volume(0.3)
    pygame.mixer.music.play(loops=-1)
except pygame.error:
    print(f"¡AVISO! No se pudo cargar la música de fondo.")


# --- Clases de Sprites ---
class Player(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()
        self.image = pygame.transform.scale(player_img, (50, 40))
        self.rect = self.image.get_rect(centerx=WIDTH // 2, bottom=HEIGHT - 10)
        self.speed = 8
        self.lives = 3
        self.score = 0
        self.last_shot = pygame.time.get_ticks()
        self.shoot_delay = 250
        self.invulnerable = False
        self.hide_timer = pygame.time.get_ticks()

    def update(self):
        now = pygame.time.get_ticks()
        if self.invulnerable and now - self.hide_timer > 2000:
            self.invulnerable = False
            self.image.set_alpha(255)
        if self.invulnerable:
            self.image.set_alpha(128 if (now // 100) % 2 == 0 else 255)
        
        keys = pygame.key.get_pressed()
        if not self.invulnerable:
            if keys[pygame.K_LEFT] and self.rect.left > 0: self.rect.x -= self.speed
            if keys[pygame.K_RIGHT] and self.rect.right < WIDTH: self.rect.x += self.speed
        if keys[pygame.K_SPACE]: self.shoot()

    def shoot(self):
        if self.invulnerable: return
        now = pygame.time.get_ticks()
        if now - self.last_shot > self.shoot_delay:
            self.last_shot = now
            bullet = Bullet(self.rect.centerx, self.rect.top)
            all_sprites.add(bullet)
            player_bullets.add(bullet)
            laser_sound.play()

    def get_hit(self):
        if not self.invulnerable:
            self.lives -= 1
            self.invulnerable = True
            self.hide_timer = pygame.time.get_ticks()
            explosion_sound.play()

class Enemy(pygame.sprite.Sprite):
    def __init__(self, x, y, type):
        super().__init__()
        self.image = pygame.transform.scale(enemy_imgs[type], (45, 45))
        self.rect = self.image.get_rect(topleft=(x, y))
        # Posición base, se actualiza con el descenso del enjambre
        self.base_y = float(y)
        # Parámetros para el movimiento horizontal individual
        self.center_x = float(x)
        self.horizontal_range = random.uniform(30, 80)
        self.horizontal_angle = random.uniform(0, 2 * math.pi)
        self.horizontal_speed = random.uniform(0.01, 0.04)
        
    def update(self, dy):
        # Actualizar la posición base vertical con el descenso del enjambre
        self.base_y += dy

        # Calcular el movimiento horizontal individual y sinusoidal
        self.horizontal_angle += self.horizontal_speed
        current_x = self.center_x + math.sin(self.horizontal_angle) * self.horizontal_range
        
        # Asegurarse de que no se salga de los bordes de la pantalla
        current_x = max(self.rect.width / 2, min(current_x, WIDTH - self.rect.width / 2))

        # Establecer la posición final del rectángulo
        self.rect.centerx = current_x
        self.rect.centery = self.base_y

class Bullet(pygame.sprite.Sprite):
    def __init__(self, x, y, speed=-10, img=bullet_img):
        super().__init__()
        self.image = img
        self.rect = self.image.get_rect(centerx=x, bottom=y)
        self.speed = speed
    def update(self):
        self.rect.y += self.speed
        if not screen.get_rect().contains(self.rect):
            self.kill()

# --- Funciones de Ayuda ---
def create_wave():
    for bullet in enemy_bullets: bullet.kill()
    for e in enemies: e.kill()
    for row in range(4):
        for column in range(8):
            enemy = Enemy(100 + column * 80, 50 + row * 60, row % 3)
            all_sprites.add(enemy)
            enemies.add(enemy)

# --- Inicialización del Juego ---
all_sprites = pygame.sprite.Group()
enemies = pygame.sprite.Group()
player_bullets = pygame.sprite.Group()
enemy_bullets = pygame.sprite.Group()
player = Player()
all_sprites.add(player)

game_state = 'INTRO'
state_timer = pygame.time.get_ticks()
wave = 1
# Variables para el descenso temporizado del enjambre
last_downward_move_time = 0
downward_move_interval = 5000 # 5 segundos
enemy_shoot_chance = 0.01

# --- Bucle Principal ---
clock = pygame.time.Clock()
running = True
while running:
    dt = clock.tick(60) / 1000.0
    now = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # --- Máquina de Estados ---
    if game_state == 'INTRO':
        screen.blit(background, (0, 0))
        font_title = pygame.font.Font(None, 70)
        text_title = font_title.render("Guerras Espaciales", True, (255, 255, 0))
        font_author = pygame.font.Font(None, 30)
        text_author = font_author.render("Jaime Mendez - Universidad Santo Tomás", True, (255, 255, 255))
        font_wave = pygame.font.Font(None, 50)
        text_wave = font_wave.render(f"Inicia la Ola {wave}", True, (50, 255, 50))
        
        rect_title = text_title.get_rect(center=(WIDTH/2, HEIGHT/2 - 80))
        rect_author = text_author.get_rect(center=(WIDTH/2, HEIGHT/2))
        rect_wave = text_wave.get_rect(center=(WIDTH/2, HEIGHT/2 + 80))
        screen.blit(text_title, rect_title); screen.blit(text_author, rect_author); screen.blit(text_wave, rect_wave)
        
        if now - state_timer > 3000:
            create_wave()
            last_downward_move_time = now # Reiniciar el temporizador de descenso
            game_state = 'PLAYING'

    elif game_state == 'PLAYING':
        # Actualización de Sprites
        player.update()
        player_bullets.update()
        enemy_bullets.update()
        
        # Descenso del enjambre basado en tiempo
        dy = 0
        if now - last_downward_move_time > downward_move_interval:
            dy = 10 # Descenso más lento y controlado
            last_downward_move_time = now
        
        enemies.update(dy) # Solo se pasa el delta Y
        
        # Lógica de Disparo Enemigo
        if random.random() < enemy_shoot_chance and enemies.sprites():
            shooter = random.choice(enemies.sprites())
            enemy_bullet = Bullet(shooter.rect.centerx, shooter.rect.bottom, 5, enemy_bullet_img)
            all_sprites.add(enemy_bullet)
            enemy_bullets.add(enemy_bullet)
            enemy_laser_sound.play()
            
        # Colisiones
        if pygame.sprite.groupcollide(enemies, player_bullets, True, True):
            explosion_sound.play()
            player.score += 100
        if pygame.sprite.spritecollide(player, enemy_bullets, True): player.get_hit()
        if pygame.sprite.spritecollide(player, enemies, True): player.get_hit()
            
        # Condiciones de cambio de estado
        wave_reset_needed = False
        for enemy in enemies:
            if enemy.rect.bottom >= player.rect.top:
                player.get_hit()
                if player.lives > 0: wave_reset_needed = True
                break
                
        if wave_reset_needed:
            game_state = 'INTRO'
            state_timer = now
            
        if not enemies and player.lives > 0:
            wave += 1
            downward_move_interval = max(1000, downward_move_interval - 400) # Se mueven hacia abajo más rápido
            enemy_shoot_chance += 0.005
            game_state = 'INTRO'
            state_timer = now
            
        if player.lives <= 0:
            game_state = 'GAME_OVER'
            state_timer = now
            
        # Dibujar
        screen.blit(background, (0, 0))
        all_sprites.draw(screen)
        font = pygame.font.Font(None, 36)
        text = font.render(f"Puntuación: {player.score} Vidas: {player.lives} Ola: {wave}", True, (255, 255, 255))
        screen.blit(text, (10, 10))

    elif game_state == 'GAME_OVER':
        screen.blit(background, (0, 0))
        font_go = pygame.font.Font(None, 80)
        text_go = font_go.render("GAME OVER", True, (255, 0, 0))
        rect_go = text_go.get_rect(center=(WIDTH/2, HEIGHT/2 - 40))
        font_sc = pygame.font.Font(None, 50)
        text_sc = font_sc.render(f"Puntuación Final: {player.score}", True, (255, 255, 255))
        rect_sc = text_sc.get_rect(center=(WIDTH/2, HEIGHT/2 + 40))
        screen.blit(text_go, rect_go)
        screen.blit(text_sc, rect_sc)
        
        if now - state_timer > 4000:
            running = False
    
    pygame.display.flip()

pygame.quit()
