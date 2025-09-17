import pygame
import math
import sys

# Setup Pygame
pygame.init()
SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 800
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Simulazione Piano Inclinato con Attrito")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 18)
FPS=60

# Colori
WHITE = (255, 255, 255)
GRAY = (180, 180, 180)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

# Costanti fisiche
g = 9.81            # Accelerazione gravitazionale (m/s^2)
mass = 1.0          # Massa del blocco (kg)
mu = 0.2            # Coefficiente di attrito iniziale
angle_deg = 45      # Angolo del piano (gradi)
angle_rad = math.radians(angle_deg)
start_x = 100
start_y = SCREEN_HEIGHT//2 + 250
initial_velocity = 0.0
posizione_calcolata_lineare = 0.00

def piano_inclinato():
    base_length = 500
    height = math.tan(angle_rad) * base_length
    a = (start_x, start_y)
    b = (start_x + base_length, start_y)
    c = (start_x, start_y - height)
    pygame.draw.polygon(screen, GRAY, [a, b, c])
    return c

def blocco(position_def , posizione_vertice_C):
    x = posizione_vertice_C[0] + position_def * math.cos(angle_rad)
    y = posizione_vertice_C[1] - position_def * -math.sin(angle_rad)

    a = x , y  
    b = (a[0] + 50 * math.cos(angle_rad), a[1] + 50 * math.sin(angle_rad))
    c = (b[0] + -50 * -math.sin(angle_rad), b[1] + -50 * math.cos(angle_rad))
    d = (a[0] + -50 * -math.sin(angle_rad), a[1] + -50 * math.cos(angle_rad))
    pygame.draw.polygon(screen, RED, [a, b, c, d] )
    screen.blit(font.render("a" ,True, WHITE), (a))
    screen.blit(font.render("b" ,True, WHITE), (b))
    screen.blit(font.render("c" ,True, WHITE), (c))
    screen.blit(font.render("d" ,True, WHITE), (d))
    return b

def logic():
    Fp_x = mass * g * math.sin(angle_rad)
    Fp_y = mass * g * math.cos(angle_rad)
    F_attrito = mu * Fp_y
    acccelerazione = (Fp_x - F_attrito) / mass
    if F_attrito >= Fp_x:
        acccelerazione = 0
    return Fp_x , Fp_y , F_attrito , acccelerazione

def draw_info(Fp_x , Fp_y , F_attrito , acccelerazione , velocity , posizione):
    text = font.render(f"Attrito: {mu:.2f} | F_attrito {F_attrito:.2f} N | Fp_x : {Fp_x:.2f} N | fp_y : {Fp_y:.2f} N | accelerazione {acccelerazione:.2f} m/s**2 | posizione lineare : {posizione:.2f} m | vcelocità : {velocity:.2f} m/s", True, WHITE)
    screen.blit(text, (10, 10))

clock = pygame.time.Clock()
running = True
while running:
    dt = clock.tick(FPS) / 1000
    posizione_vertice_C = piano_inclinato()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Game logic here

    screen.fill(BLACK)
    piano_inclinato()
    Fp_x , Fp_y , F_attrito , acccelerazione = logic()

    if blocco(posizione_calcolata_lineare , posizione_vertice_C)[0] > 600:
        acccelerazione=0
    else:
        initial_velocity = initial_velocity + acccelerazione * dt         #v=v+a⋅Δt
        posizione_calcolata_lineare = posizione_calcolata_lineare + initial_velocity * dt             #s=s+v⋅Δt

        print(
        f"tempo{dt} s\n"
        f"{'Fp_x (componente parallela forza peso)':40}: {Fp_x:8.3f} N\n"
        f"{'Fp_y (componente perpendicolare forza peso)':40}: {Fp_y:8.3f} N\n"
        f"{'F_attrito':40}: {F_attrito:8.3f} N\n"
        f"{'Accelerazione':40}: {acccelerazione:8.3f} m/s²\n"
        f"{'Velocità':40}: {initial_velocity:8.3f} m/s\n"
        f"{'Posizione lungo piano':40}: {posizione_calcolata_lineare:8.3f} m\n")

    blocco(posizione_calcolata_lineare , posizione_vertice_C)
    draw_info(Fp_x , Fp_y , F_attrito , acccelerazione, initial_velocity , posizione_calcolata_lineare)

    pygame.display.flip()

pygame.quit()
sys.exit()
