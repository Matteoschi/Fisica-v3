import pygame
import sys
import math
import random

pygame.init()

SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600
FPS = 60

WHITE   = (255, 255, 255)
BLACK   = (0, 0, 0)
RED     = (255, 0, 0)
BLUE    = (0, 0, 255)
YELLOW  = (255, 200, 0)
ORANGE  = (255, 165, 0)
PURPLE  = (128, 0, 128)
CYAN    = (0, 255, 255)
MAGENTA = (255, 0, 255)
GRAY    = (128, 128, 128)
BROWN   = (139, 69, 19)
PINK    = (255, 192, 203)
DARKRED = (139, 0, 0)
DARKBLUE= (0, 0, 139)

font = pygame.font.SysFont(None, 18)
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Pygame Starter")

clock = pygame.time.Clock()
running = True

class GameObject:
    def __init__(self, x , y , z , massa , colore):
        self.x = x
        self.y= y
        self.z = z
        self.massa  = massa
        self.colore = colore
        self.Vx = 0
        self.Vy=0
        self.Vz=0
        self.range_sensore = 85  
        self.n_sensori = 8

    def draw(self, rectangles):
        # Disegna il drone
        
        label = font.render(f"X:{round(self.x,1)} Y:{round(self.y,1)} Z:{round(self.z,1)}", True, WHITE)
        screen.blit(label, (10, 10))

        # Ciclo sui sensori
        for i in range(self.n_sensori):
            angolo = 2 * math.pi * i / self.n_sensori  # angolo in radianti
            # Lunghezza massima del sensore
            fine_x = self.x + self.range_sensore * math.cos(angolo)
            fine_y = self.y + self.range_sensore * math.sin(angolo)
            colore_linea = WHITE

            # Variabili per memorizzare il punto di collisione più vicino
            distanza_min = self.range_sensore
            collision_min_x = fine_x
            collision_min_y = fine_y

            # Controllo collisione con tutti i rettangoli
            for rettangolo in rectangles:
                collision_x, collision_y = punto_collisione(self, rettangolo, angolo)
                distanza_collisione = math.hypot(collision_x - self.x, collision_y - self.y)
                if distanza_collisione < distanza_min:
                    distanza_min = int(distanza_collisione)
                    collision_min_x = collision_x
                    collision_min_y = collision_y
                    colore_linea = RED  # ostacolo rilevato

            # Aggiorna la fine del sensore
            fine_x, fine_y = collision_min_x, collision_min_y

            # Disegna la linea del sensore
            proporzione = distanza_min / self.range_sensore
            rosso = int((1 - proporzione) * 255)
            verde = int(proporzione * 255)
            colore_linea = (rosso, verde, 0)

            pygame.draw.line(screen, colore_linea, (self.x, self.y), (fine_x, fine_y), 2)

            # Mostra la distanza del sensore
            label = font.render(f"{distanza_min}", True, WHITE)
            screen.blit(label, (fine_x, fine_y))
            if i == 0:
                if distanza_min < self.range_sensore:
                    label = font.render(f"Ostacolo davanti : {distanza_min}", True, RED)
                    if distanza_min < 30:
                        self.x -= 4.5

                    screen.blit(label, (650, 15))
            if i == 4:
                if distanza_min < self.range_sensore:
                    label = font.render(f"Ostacolo dietro : {distanza_min}", True, RED)
                    screen.blit(label, (650, 30))
                    if distanza_min < 30:
                        self.x += 4.5
            if i == 2:
                if distanza_min < self.range_sensore:
                    label = font.render(f"Ostacolo a destra : {distanza_min}", True, RED)
                    screen.blit(label, (650, 45))
                    if distanza_min < 30:
                        self.y -= 4.5
            if i == 6:
                if distanza_min < self.range_sensore:
                    label = font.render(f"Ostacolo a sinistra : {distanza_min}", True, RED)
                    screen.blit(label, (650, 60))
                    if distanza_min < 30:
                        self.y += 4.5
            if i == 1:
                if distanza_min < 50:
                    label = font.render(f"Ostacolo basso-sinistra : {distanza_min}", True, RED)
                    screen.blit(label, (475, 15))
                    if distanza_min < 30:
                        self.x -= 3
                        self.y -= 3
            if i == 3:
                if distanza_min < 50:
                    label = font.render(f"Ostacolo basso-sinistra : {distanza_min}", True, RED)
                    screen.blit(label, (475, 30))
                    if distanza_min < 30:
                        self.x += 3
                        self.y -= 3
            if i == 5:
                if distanza_min < 50:
                    label = font.render(f"Ostacolo alto-sinistra : {distanza_min}", True, RED)
                    screen.blit(label, (475, 45))
                    if distanza_min < 30:
                        self.x += 3
                        self.y += 3
            if i == 7:
                if distanza_min < 50:
                    label = font.render(f"Ostacolo alto-destra : {distanza_min}", True, RED)
                    screen.blit(label, (475, 60))
                    if distanza_min < 30:
                        self.x -= 3
                        self.y += 3
        pygame.draw.circle(screen, self.colore, (int(self.x), int(self.y)), 10)




    def move(self):
        self.x += self.Vx
        self.y += self.Vy
        self.z += self.Vz
        if self.z > 50 :
            self.z = 50
        if self.z < 0:
            self.z =0
        if self.y > SCREEN_HEIGHT:
            self.y = SCREEN_HEIGHT
        if self.x > SCREEN_WIDTH:
            self.x = SCREEN_WIDTH       
        if self.y < 0:
            self.y = 0
        if self.x < 0:
            self.x = 0 

drone = GameObject(20,35,0,1,WHITE)


def punto_collisione(drone, rect, angolo):
    distanza_min = drone.range_sensore
    fine_x = drone.x + drone.range_sensore * math.cos(angolo)
    fine_y = drone.y + drone.range_sensore * math.sin(angolo)

    # Campioniamo punti lungo il sensore
    for t in range(0, drone.range_sensore, 2):  # step più piccolo = più precisione
        px = drone.x + t * math.cos(angolo)
        py = drone.y + t * math.sin(angolo)
        if rect[0] <= px <= rect[0]+rect[2] and rect[1] <= py <= rect[1]+rect[3]:
            distanza_min = t
            break  # fermati al primo ostacolo
    if drone.z <= rect[5]:
        fine_collisione_x = drone.x + distanza_min * math.cos(angolo)
        fine_collisione_y = drone.y + distanza_min * math.sin(angolo)
        return fine_collisione_x, fine_collisione_y
    
    return fine_x, fine_y





rectangles = [
    (50, 50, 100, 60, DARKBLUE , random.randint(20,75)),
    (200, 80, 120, 50, CYAN, random.randint(20,75)),
    (400, 100, 80, 100, BLUE, random.randint(20,75)),
    (600, 150, 90, 70, YELLOW, random.randint(20,75)),
    (100, 250, 70, 90, PURPLE, random.randint(20,75)),
    (300, 300, 110, 60, ORANGE, random.randint(20,75)),
    (500, 350, 60, 120, BLUE, random.randint(20,75)),
    (650, 400, 80, 80, YELLOW, random.randint(20,75)),
    (150, 450, 100, 50, GRAY, random.randint(20,75)),
    (400, 500, 120, 60, PINK, random.randint(20,75))
]


while running:
    screen.fill(BLACK)
    clock.tick(FPS)

    for i in rectangles:
        pygame.draw.rect(screen, i[4], (i[0], i[1], i[2], i[3]))
        label = font.render(f"H{i[5]}", True, WHITE)

        # Coordina per centrare il testo
        text_rect = label.get_rect(center=(i[0] + i[2]//2, i[1] + i[3]//2))

        screen.blit(label, text_rect)
        

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                drone.Vy -= 5  # inverti perché y aumenta verso il basso
            if event.key == pygame.K_s:
                drone.Vy += 5
            if event.key == pygame.K_a:
                drone.Vx -= 5
            if event.key == pygame.K_d:
                drone.Vx += 5
            if event.key == pygame.K_DOWN:
                drone.Vz -= 1
            if event.key == pygame.K_UP:
                drone.Vz += 1
            
        if event.type == pygame.KEYUP:
            if event.key in [pygame.K_w, pygame.K_s]:
                drone.Vy = 0
            if event.key in [pygame.K_a, pygame.K_d]:
                drone.Vx = 0
            if event.key in [pygame.K_UP, pygame.K_DOWN]:
                drone.Vz = 0

    # Muovi il drone
    drone.move()

    # Disegna il drone
    drone.draw(rectangles)

    pygame.display.flip()

