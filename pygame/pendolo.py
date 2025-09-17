import pygame
import sys
import math

pygame.init()

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

# === Costanti fisiche ===
G = 9.81                   # Gravità [m/s²]
L_METRI = 2                      # Lunghezza pendolo [metri]
L_PIXEL = L_METRI * 100  # Lunghezza pendolo in pixel (1 metro = 100 pixel)
Theta0 = math.pi / 4       # Angolo iniziale [rad]
print(f"Lunghezza pendolo in pixel: {L_PIXEL} , Angolo iniziale: {math.degrees(Theta0)} gradi , lunghezza in metri: {L_METRI}")

# === Colori ===
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# === Pygame setup ===
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Pendolo (piccole oscillazioni)")
clock = pygame.time.Clock()

# === Origine ===
inizio = (SCREEN_WIDTH // 2, 100)

def draw(inizio, fine):
    pygame.draw.circle(screen, color=WHITE, center=inizio, radius=10)
    pygame.draw.line(screen, color=WHITE, start_pos=inizio, end_pos=fine, width=2)
    pygame.draw.circle(screen, color=WHITE, center=fine, radius=10)

def logic(t, Theta0):
    angolo = Theta0 * math.cos(math.sqrt(G / L_METRI) * t)
    print(f"Tempo: {t:.2f} s, Angolo: {math.degrees(angolo):.2f} gradi")
    return angolo

running = True
start_time = pygame.time.get_ticks() / 1000  # momento iniziale in secondi

while running:
    clock.tick(FPS)
    screen.fill(BLACK)

    # === Eventi ===
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # === Tempo corrente ===
    time = pygame.time.get_ticks() / 1000 - start_time

    # === Fisica del pendolo ===
    Theta = logic(time, Theta0)
    fine = (inizio[0] + L_PIXEL * math.sin(Theta), inizio[1] + L_PIXEL * math.cos(Theta)) # coordinate polari del pendolo

    # === Disegna ===
    draw(inizio, fine)

    pygame.display.flip()

pygame.quit()
sys.exit()
