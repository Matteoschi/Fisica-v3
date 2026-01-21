import pygame
import sys
import math

# === Setup iniziale ===
pygame.init()
LARGHEZZA, ALTEZZA = 1000, 600
schermo = pygame.display.set_mode((LARGHEZZA, ALTEZZA))
pygame.display.set_caption("Moto Parabolico: Ideale vs Reale")
orologio = pygame.time.Clock()
font = pygame.font.SysFont(None, 20)

# === Colori ===
BIANCO = (255, 255, 255)
ROSSO  = (255, 50, 50)
BLU    = (50, 150, 255)
NERO   = (0, 0, 0)

# === Costanti fisiche ===
GRAVITA = 9.81  # Accelerazione gravitazionale (m/s²)
SCALA_PIXEL = 2  # Scala di conversione metri → pixel
altezza_iniziale = 50  # Altezza iniziale in metri
altezza_finale = 0     # Altezza finale del suolo
velocita_iniziale = 65  # Velocità iniziale (m/s)
angolo_gradi = 45
angolo_radianti = math.radians(angolo_gradi)

# Componenti della velocità iniziale:
# vx = v0 * cos(θ), vy = v0 * sin(θ)
velocita_iniziale_x = velocita_iniziale * math.cos(angolo_radianti)
velocita_iniziale_y = velocita_iniziale * math.sin(angolo_radianti)

# === Parametri per la resistenza dell’aria ===
coeff_drag = 0.47  # coefficiente di drag per una sfera
densita_aria = 1.225  # densità dell'aria (kg/m³)
raggio_proiettile = 2  # raggio in metri
area_sezione = math.pi * raggio_proiettile**2  # A = πr²
massa_proiettile = 100 # massa in kg

# === Stati iniziali ===
pos_x_ideale, pos_y_ideale = 0, altezza_iniziale
pos_x_aria, pos_y_aria = 0, altezza_iniziale
vel_x_aria, vel_y_aria = velocita_iniziale_x, velocita_iniziale_y
traiettoria_ideale = []
traiettoria_aria = []

# === Calcolo moto ideale (senza resistenza dell'aria) ===
def calcola_ideale(tempo):
    # Formule: x = v₀x * t, y = y₀ + v₀y * t - ½gt²
    x = velocita_iniziale_x * tempo
    y = altezza_iniziale + velocita_iniziale_y * tempo - 0.5 * GRAVITA * tempo**2

    # Soluzione dell’equazione quadratica per il tempo di volo
    tempo_totale = (-velocita_iniziale_y - math.sqrt((velocita_iniziale_y)**2 - 4*(-0.5*GRAVITA)*(altezza_iniziale-altezza_finale))) / (2*(-0.5*GRAVITA))
    
    # vy(t) = v₀y - g * t
    vel_y = velocita_iniziale_y - GRAVITA * tempo

    # v(t) = √(vx² + vy²)
    velocita_totale = math.sqrt(velocita_iniziale_x**2 + vel_y**2)
    return x, y, vel_y, velocita_totale, tempo_totale

# === Calcolo moto reale (con resistenza dell’aria) ===
def aggiorna_con_aria(x, y, vx, vy, tempo):
    # Calcolo della velocità istantanea
    velocita = math.hypot(vx, vy)
    
    # Forza di resistenza: F_drag = ½ * ρ * Cd * A * v²
    if velocita != 0:
        forza_drag = 0.5 * coeff_drag * densita_aria * area_sezione * velocita**2

        # Accelerazioni (con resistenza): a = F_drag / m, direzione opposta alla velocità
        acc_x = -forza_drag * (vx / velocita) / massa_proiettile
        acc_y = -GRAVITA - (forza_drag * (vy / velocita)) / massa_proiettile
    else:
        acc_x = 0
        acc_y = -GRAVITA

    # Integrazione: v += a * dt, x += v * dt
    vx += acc_x * tempo
    vy += acc_y * tempo
    x += vx * tempo
    y += vy * tempo

    velocita_totale = math.sqrt(vx**2 + vy**2)
    return x, y, vx, vy, velocita_totale

# === Funzioni di disegno ===
def disegna_proiettile(x, y):
    pygame.draw.circle(schermo, BIANCO, (int(x), int(y)), 5)

def disegna_testo(superficie, testo, x, y, colore=BIANCO):
    superficie.blit(font.render(testo, True, colore), (x, y))

# Trasforma da coordinate fisiche (metri) a coordinate schermo (pixel)
def trasforma_coordinate(x1, y1, x2, y2):
    px1 = int(x1 * SCALA_PIXEL)
    py1 = ALTEZZA - int(y1 * SCALA_PIXEL)
    px2 = int(x2 * SCALA_PIXEL)
    py2 = ALTEZZA - int(y2 * SCALA_PIXEL)
    return px1, py1, px2, py2

def disegna_traiettoria(coordinate, colore):
    for punto in coordinate:
        pygame.draw.circle(schermo, colore, punto, 2)

def disegna_apice(posizione_px, posizione_m, tempo_apice):
    pygame.draw.circle(schermo, BLU, posizione_px, 5)
    schermo.blit(font.render(
        f"Apex {posizione_m} m , tempo {tempo_apice:.2f} s", True, BIANCO),
        (posizione_px[0] + 15, posizione_px[1] - 10)
    )

# === Loop principale ===
tempo_inizio = pygame.time.get_ticks() / 1000
in_esecuzione = True
apice_px_ideale = None
apice_px_aria = None
in_volo = False
in_volo_aria = False
tempo_finale = None

while in_esecuzione:
    dt = orologio.tick(60) / 1000  # Delta time per fisica realistica
    schermo.fill(NERO)

    if tempo_finale is None:
        tempo = pygame.time.get_ticks() / 1000 - tempo_inizio
    else:
        tempo = tempo_finale  # Blocca il tempo dopo l’impatto

    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            in_esecuzione = False

    # === Moto ideale ===
    if not in_volo:
        pos_x_ideale, pos_y_ideale, vel_y_ideale, vel_tot_ideale, tempo_ideale = calcola_ideale(tempo)
        px1, py1, _, _ = trasforma_coordinate(pos_x_ideale, pos_y_ideale, pos_x_aria, pos_y_aria)
        traiettoria_ideale.append((px1, py1))

        if apice_px_ideale is None and math.isclose(vel_y_ideale, 0, abs_tol=0.1):
            apice_px_ideale = (int(px1), int(py1))
            tempo_apice_ideale = tempo
            apice_m_ideale = (round(pos_x_ideale, 2), round(pos_y_ideale, 2))

        if py1 >= ALTEZZA:
            in_volo = True  # Il proiettile ha colpito il suolo (ideale)

    # === Moto reale (con resistenza dell’aria) ===
    if not in_volo_aria:
        pos_x_aria, pos_y_aria, vel_x_aria, vel_y_aria, vel_tot_aria = aggiorna_con_aria(
            pos_x_aria, pos_y_aria, vel_x_aria, vel_y_aria, dt)
        _, _, px2, py2 = trasforma_coordinate(pos_x_ideale, pos_y_ideale, pos_x_aria, pos_y_aria)
        traiettoria_aria.append((px2, py2))

        if apice_px_aria is None and math.isclose(vel_y_aria, 0, abs_tol=0.1):
            apice_px_aria = (int(px2), int(py2))
            tempo_apice_aria = tempo
            apice_m_aria = (round(pos_x_aria, 2), round(pos_y_aria, 2))

        if py2 >= ALTEZZA:
            in_volo_aria = True  # Proiettile ha colpito il suolo (con aria)

    # === Stampa dati solo al termine dei moti ===
    if in_volo and in_volo_aria and tempo_finale is None:
        tempo_finale = tempo
        print(f"""\n=== RISULTATI FINALI ===
        → Tempo totale: {tempo:.2f} s

        → Moto Ideale:
        x  = {pos_x_ideale:.2f} m   |  y  = {pos_y_ideale:.2f} m
        vy = {vel_y_ideale:.2f} m/s |  v  = {vel_tot_ideale:.2f} m/s
        angolo finale = {math.degrees(math.atan2(vel_y_ideale, velocita_iniziale_x)):.2f}°
        apex = ({apice_m_ideale[0]:.2f} m, {apice_m_ideale[1]:.2f} m), tempo {tempo_apice_ideale:.2f} s

        → Moto Reale (con resistenza dell’aria):
        x  = {pos_x_aria:.2f} m     |  y  = {pos_y_aria:.2f} m
        vy = {vel_y_aria:.2f} m/s   |  v  = {vel_tot_aria:.2f} m/s
        angolo finale = {math.degrees(math.atan2(vel_y_aria, vel_x_aria)):.2f}°
        apex = ({apice_m_aria[0]:.2f} m, {apice_m_aria[1]:.2f} m), tempo {tempo_apice_aria:.2f} s

        → Costanti usate nella simulazione con aria:
        - Massa del proiettile      = {massa_proiettile} kg
        - Raggio del proiettile     = {raggio_proiettile} m
        - Area sezione frontale     = {area_sezione:.5f} m²
        - Densità dell’aria         = {densita_aria} kg/m³
        - Coefficiente di resistenza (drag) = {coeff_drag}
        """)

    # === Stampa in tempo reale se ancora in volo ===
    if not in_volo or not in_volo_aria:
        print(f"t={tempo:.2f}s | Ideale → x={pos_x_ideale:.2f}m, y={pos_y_ideale:.2f}m, v={vel_tot_ideale:.2f}m/s, vy={vel_y_ideale:.2f}m/s, angolo={math.degrees(math.atan2(vel_y_ideale, velocita_iniziale_x)):.2f}° | Reale → x={pos_x_aria:.2f}m, y={pos_y_aria:.2f}m, v={vel_tot_aria:.2f}m/s, vy={vel_y_aria:.2f}m/s")

    # === Disegno elementi ===
    if apice_px_ideale: 
        disegna_apice(apice_px_ideale, apice_m_ideale, tempo_apice_ideale)
    if apice_px_aria:   
        disegna_apice(apice_px_aria, apice_m_aria, tempo_apice_aria)

    disegna_traiettoria(traiettoria_ideale, BIANCO)
    disegna_traiettoria(traiettoria_aria, ROSSO)
    disegna_proiettile(px1, py1)
    disegna_proiettile(px2, py2)

    disegna_testo(schermo, f"Tempo: {tempo:.2f} s", 10, 10)
    disegna_testo(schermo, f"Ideale → x: {pos_x_ideale:.2f} m, y: {pos_y_ideale:.2f} m", 10, 30)
    disegna_testo(schermo, f"Reale  → x: {pos_x_aria:.2f} m, y: {pos_y_aria:.2f} m", 10, 50)
    disegna_testo(schermo, "Bianco = Moto Ideale | Rosso = Moto Reale con Aria", 10, 580)

    pygame.display.flip()

pygame.quit()
sys.exit()
