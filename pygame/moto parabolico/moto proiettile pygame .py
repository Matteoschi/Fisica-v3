import pygame
import sys
import math

pygame.init()

# === Schermo ===
SCREEN_WIDTH = 800  # larghezza finestra
SCREEN_HEIGHT = 600  # altezza finestra
FPS = 60  # fotogrammi al secondo
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
font = pygame.font.SysFont(None, 18)  # font per testo a schermo

# === Colori RGB ===
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED   = (255, 0, 0)
BLU   = (0, 128, 255)

# === Parametri Fisici ===
INITIAL_VELOCITY = 50          # velocità iniziale in m/s
ANGLE = math.radians(45)       # angolo di lancio convertito in radianti
G = 9.81                       # accelerazione gravitazionale in m/s^2
SCALE = 2                      # fattore scala: 1 metro = 2 pixel

# === Posizione iniziale (origine spostata) ===
S0 = 0            # x iniziale in pixel
Y0 = 30             # y iniziale in metri
YO_NUOVO = SCREEN_HEIGHT - Y0  # conversione da y fisico a y per pygame
YF=0  # y finale in metri (terra)

in_volo = True
start_time = pygame.time.get_ticks() / 1000  # tempo iniziale (secondi)

# === Funzione logica per calcolare posizione del proiettile ===
def logic(time):
    # Posizione in metri
    x_m = INITIAL_VELOCITY * math.cos(ANGLE) * time
    y_m = INITIAL_VELOCITY * math.sin(ANGLE) * time - 0.5 * G * time**2

    # Calcolo del tempo totale di volo (risolvendo y(t) = 0)
    tempo_calcolato_1 = (-INITIAL_VELOCITY * math.sin(ANGLE) -
                         math.sqrt((INITIAL_VELOCITY * math.sin(ANGLE))**2 - 4*(-0.5*G)*(Y0-YF))) / (2*(-0.5*G))
    
    # Conversione in pixel
    x_px = x_m * SCALE + S0
    y_px = YO_NUOVO - y_m  # inversione asse y
    return x_px, y_px, tempo_calcolato_1, x_m, y_m

# === Funzione per disegnare oggetti principali ===
def draw(x, y):
    pygame.draw.circle(screen, WHITE, (S0, YO_NUOVO), 5)  # punto iniziale
    pygame.draw.line(screen, WHITE, (S0, YO_NUOVO), (S0, YO_NUOVO - 20), 2)  # asse y
    pygame.draw.line(screen, WHITE, (S0, YO_NUOVO), (S0 + 20, YO_NUOVO), 2)  # asse x
    pygame.draw.circle(screen, WHITE, (int(x), int(y)), 5)  # proiettile

# === Disegna traiettoria memorizzata ===
def trajectory_draw(trajectory):
    for point in trajectory:
        pygame.draw.circle(screen, RED, (int(point[0]), int(point[1])), 2)

# === Inizializzazione traiettoria e variabili ===
trajectory = []
clock = pygame.time.Clock()
running = True
apex_position_px = None  # coordinata dell'apice in pixel

# === Loop principale ===
while running:
    clock.tick(FPS)
    screen.fill(BLACK)

    # Gestione eventi
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if in_volo:
        # Tempo corrente normalizzato
        current_time = pygame.time.get_ticks() / 1000
        normal_time = current_time - start_time

        # Calcolo posizione e parametri
        pixel_x, pixel_y, tempo_calcolato_1, x_m, y_m = logic(normal_time)
        vy = INITIAL_VELOCITY * math.sin(ANGLE) - G * normal_time
        velocità_istantanea = math.sqrt((INITIAL_VELOCITY * math.cos(ANGLE))**2 + vy**2) # totale 

        if normal_time <= tempo_calcolato_1:
            # Salva la posizione attuale
            trajectory.append((pixel_x, pixel_y))

            # Stampa info sul terminale
            print(f"Tempo: {normal_time:.2f} s, Posizione: ({x_m:.2f} m, {y_m:.2f} m) , velocità: {velocità_istantanea:.2f} m/s , velocità y: {vy:.2f} m/s , angolo: {math.degrees(math.atan2(vy, INITIAL_VELOCITY * math.cos(ANGLE))):.2f} °")

            # Rileva apice della traiettoria (una sola volta)
            if apex_position_px is None and math.isclose(vy, 0, abs_tol=0.1):
                print("Il proiettile ha raggiunto il punto più alto della traiettoria.")
                apex_position_px = (int(pixel_x), int(pixel_y))  # posizione pixel
                tempo_apice = normal_time
                apex_position_m = (round(x_m, 2), round(y_m, 2))  # posizione metri
        else:
            # Fine del volo
            in_volo = False
            print(f"\n>>> Il proiettile ha toccato terra:")
            print(f"Tempo: {normal_time:.2f} s")
            print(f"Posizione finale: ({x_m:.2f} m, {y_m:.2f} m)")
            print(f"Tempo di volo calcolato teorico: {tempo_calcolato_1:.2f} s")
            print(f"velocità finale: {velocità_istantanea:.2f} m/s")
            print(f"angolo finale: {math.degrees(math.atan2(vy, INITIAL_VELOCITY * math.cos(ANGLE))):.2f} °")

    # Linee guida per impatto
    pygame.draw.line(screen, WHITE, (S0, YO_NUOVO), (S0, pixel_y), 2)
    pygame.draw.line(screen, WHITE, (S0, pixel_y), (pixel_x, pixel_y), 2)

    # Info testuali a schermo
    screen.blit(font.render(
        f"velocità iniziale: {INITIAL_VELOCITY} m/s, angolo: {math.degrees(ANGLE):.2f}° , altezza iniziale: {Y0} m , S0 {S0} px , altezza finale {YF}, gravità {G} m/s² " ,
        True, WHITE), (10, 30))
    
    screen.blit(font.render(
        f"Tempo: {normal_time:.2f} s velocità: {velocità_istantanea:.2f} m/s , posizione {round(x_m, 2), round(y_m, 2)} m , angolo: {math.degrees(math.atan2(vy, INITIAL_VELOCITY * math.cos(ANGLE))):.2f} °",
        True, WHITE), (10, 10))

    # Disegna oggetti dinamici
    draw(pixel_x, pixel_y)
    trajectory_draw(trajectory)

    # Titolo dinamico
    pygame.display.set_caption(f"Moto Parabolico Reale , sistema dimetrico asse x segue scale: 1 m = {SCALE} pixel")

    # Disegna apice (se rilevato)
    if apex_position_px is not None:
        pygame.draw.circle(screen, BLU, apex_position_px, 5)
        screen.blit(font.render(
            f"Apex {apex_position_m} m , tempo {tempo_apice:.2f} s",
            True, WHITE), (apex_position_px[0] + 15, apex_position_px[1] - 10))

    pygame.display.flip()

# === Uscita dal programma ===
pygame.quit()
sys.exit()
