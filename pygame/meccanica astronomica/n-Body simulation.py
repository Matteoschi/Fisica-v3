import pygame
import sys
import math

# === Costanti di configurazione ===
G = 0.667430               # Costante gravitazionale semplificata
DT = 1                     # Delta time (tempo per frame)
FPS = 60                   # Frame per secondo
MASSA_DEFAULT = 60         # Massa predefinita dei corpi

# === Colori ===
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)

# === Inizializzazione Pygame ===
pygame.init()
info = pygame.display.Info()
WIDTH, HEIGHT = info.current_w, info.current_h
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Simulazione gravitazionale - Matteo")
font = pygame.font.SysFont(None, 18)

# === Classe Corpo Celeste ===
class Body:
    def __init__(self, x, y, vx, vy, mass, color, raggio, name="Corpo"):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy
        self.mass = mass
        self.color = color
        self.raggio = raggio
        self.name = name

    def draw(self, screen, fx, fy):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.raggio)
        velocita = math.sqrt(self.vx ** 2 + self.vy ** 2)
        label = font.render(
            f"{self.name}, Fx: {round(fx,1)}, Fy: {round(fy,1)}, Vel: {round(velocita, 3)}",
            True, WHITE
        )
        screen.blit(label, (int(self.x + self.raggio + 5), int(self.y - 10)))

    def update(self):
        self.x += self.vx * DT
        self.y += self.vy * DT

    def apply_gravity(self, other):
        dx = other.x - self.x
        dy = other.y - self.y
        distanza = math.hypot(dx, dy)
        if distanza == 0:
            return 0, 0  # Evita divisioni per zero
        forza = G * (self.mass * other.mass) / distanza ** 2
        angolo = math.atan2(dy, dx)
        return forza * math.cos(angolo), forza * math.sin(angolo)

    def draw_force(self, fx, fy):
        forza_modulo = math.hypot(fx, fy)
        angolo = math.atan2(fy, fx)
        lunghezza = max(20, min(forza_modulo * 100, 50))

        start = (self.x, self.y)
        end = (
            self.x + lunghezza * math.cos(angolo),
            self.y + lunghezza * math.sin(angolo)
        )

        pygame.draw.line(screen, GREEN, start, end, 2)

        punta_sx = (
            end[0] - 10 * math.cos(angolo - math.pi / 6),
            end[1] - 10 * math.sin(angolo - math.pi / 6)
        )
        punta_dx = (
            end[0] - 10 * math.cos(angolo + math.pi / 6),
            end[1] - 10 * math.sin(angolo + math.pi / 6)
        )

        pygame.draw.polygon(screen, GREEN, [end, punta_sx, punta_dx])

# === Inizializzazione simulazione ===
corpi = []
clock = pygame.time.Clock()
running = True

# === Loop principale ===
while running:
    clock.tick(FPS)
    screen.fill(BLACK)
    mouse_x, mouse_y = pygame.mouse.get_pos()

    # Gestione eventi
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            nuovo_corpo = Body(
                x=mouse_x, y=mouse_y,
                vx=0, vy=0,
                mass=MASSA_DEFAULT,
                color=WHITE,
                raggio=10,
                name=f"Corpo {len(corpi)+1}"
            )
            corpi.append(nuovo_corpo)

    # Calcolo forze
    forces = []
    for i, corpo in enumerate(corpi):
        fx_tot, fy_tot = 0, 0
        for j, altro in enumerate(corpi):
            if i != j:
                fx, fy = corpo.apply_gravity(altro)
                fx_tot += fx
                fy_tot += fy
        forces.append((fx_tot, fy_tot))

    # Aggiornamento stato dei corpi
    for i, corpo in enumerate(corpi):
        fx, fy = forces[i]
        ax = fx / corpo.mass
        ay = fy / corpo.mass
        corpo.vx += ax * DT
        corpo.vy += ay * DT
        corpo.update()
        corpo.draw(screen, fx, fy)
        corpo.draw_force(fx, fy)

    # Output console
    print("\n--- Stato attuale dei corpi ---")
    for i, (corpo, (fx, fy)) in enumerate(zip(corpi, forces)):
        print(f"Corpo {i + 1}:")
        print(f"  Posizione: ({round(corpo.x, 3)}, {round(corpo.y, 3)})")
        print(f"  Velocità:  ({round(corpo.vx, 3)}, {round(corpo.vy, 3)})")
        print(f"  Forza netta: ({round(fx, 3)}, {round(fy, 3)})")
        print(f"  velocità_modulo: {round(math.sqrt(corpo.vx ** 2 + corpo.vy ** 2), 3)}")
        print(f"  Forza modulo: {round(math.hypot(fx, fy), 3)}")

    pygame.display.flip()

pygame.quit()
sys.exit()
