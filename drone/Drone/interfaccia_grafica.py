import pygame
import sys
import math
import random
import re
import os
import time

try:
    import serial
except ImportError:
    serial = None
pygame.init()


def carica_config_da_header(percorso_file):
    if not os.path.exists(percorso_file):
        raise FileNotFoundError(f"ERRORE CRITICO: Il file {percorso_file} non esiste. Impossibile avviare la telemetria.")

    config_cpp = {}
    
    # Questa regex cerca stringhe nel formato: const [tipo] [NOME] = [valore][f];
    # Supporta numeri negativi e decimali, ignorando gli spazi.
    pattern = re.compile(r'const\s+(float|int)\s+([A-Za-z0-9_]+)\s*=\s*([-0-9.]+)[fF]?\s*;')

    with open(percorso_file, 'r') as file:
        for linea in file:
            # Rimuove eventuali commenti sulla stessa riga per evitare falsi positivi
            linea_pulita = linea.split('//')[0].strip()
            
            if not linea_pulita:
                continue
                
            match = pattern.search(linea_pulita)
            if match:
                tipo_var = match.group(1)
                nome_var = match.group(2)
                valore_str = match.group(3)
                
                # Cast dinamico in base al tipo dichiarato in C++
                if tipo_var == 'float':
                    config_cpp[nome_var] = float(valore_str)
                elif tipo_var == 'int':
                    config_cpp[nome_var] = int(valore_str)
                    
    return config_cpp


CONFIG = carica_config_da_header(os.path.join(os.path.dirname(__file__), "config.h"))

CONFIG.update({
    "GAS_PWM_WARN": CONFIG["GAS_MASSIMO_us"] - 100,
    "T_MOTOR_WARN": CONFIG["MIN_THROTTLE_START_TEMP_C"],
    "T_MOTOR_CRIT": CONFIG["MAX_THROTTLE_END_TEMP_C"],
    "T_TEENSY_WARN": CONFIG["MIN_THROTTLE_START_TEMP_C"],
    "T_TEENSY_CRIT": CONFIG["MAX_THROTTLE_END_TEMP_C"],
    "T_EST_WARN": CONFIG["MIN_THROTTLE_START_TEMP_C"],
    "T_EST_CRIT": CONFIG["MAX_THROTTLE_END_TEMP_C"],
    "T_ESC_WARN": CONFIG["MIN_ESC_START_TEMP_C"],
    "T_ESC_CRIT": CONFIG["MIN_ESC_END_TEMP_C"],
    "DIST_TGT_WARN": CONFIG["RAGGIO_ACCETTAZIONE_MINIMO_m"],
    "SERVO_V_MIN": 3.0,
    "SERVO_V_MAX": 3.6,
    "VBAR_MOTOR_MIN": CONFIG["VALORE_BATT_MOTORE_BASSA_V"],
    "VBAR_MOTOR_MAX": CONFIG["VALORE_BATT_MOTORE_BASSA_V"] * 1.25,
    "VBAR_TEENSY_MIN": CONFIG["VALORE_BATT_TEENSY_BASSA_V"],
    "VBAR_TEENSY_MAX": CONFIG["VALORE_BATT_TEENSY_BASSA_V"] * 1.25,
})

C_BG     = ( 8,  11,  18)
C_PANEL  = (14,  18,  28)
C_BORDER = (36,  50,  76)
C_ACCENT = ( 0, 188, 255)
C_TEXT   = (205, 218, 240)
C_DIM    = ( 82,  98, 126)
C_WHITE  = (242, 246, 255)
C_GREEN  = ( 40, 210,  88)
C_YELLOW = (255, 200,   0)
C_ORANGE = (255, 140,   0)
C_RED    = (218,  48,  48)
C_SKY    = ( 28,  96, 178)
C_GROUND = (124,  80,  30)
C_CROSS  = (255, 210,   0)
C_PURPLE = (200,  80, 255)
F_HEAD  = pygame.font.SysFont("consolas", 21, bold=True)
F_TITLE = pygame.font.SysFont("consolas", 13, bold=True)
F_VAL   = pygame.font.SysFont("consolas", 15, bold=True)
F_LABEL = pygame.font.SysFont("consolas", 13)
F_SMALL = pygame.font.SysFont("consolas", 12)
# ================================================================
# FINESTRA
# ================================================================

W, H = 1500, 950
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("GCS - Ground Control Station")
clock = pygame.time.Clock()


def clamp(v, a, b):
    return max(a, min(b, v))


def testo(surf, s, pos, font, color, anchor="topleft"):
    """Scrive un testo sullo schermo."""
    img = font.render(str(s), True, color)
    surf.blit(img, img.get_rect(**{anchor: pos}))


def testo_valori(surf, label, valore, x, y, colore=C_WHITE):
    """Scrive una riga tipo:  Label:      valore"""
    testo(surf, label, (x, y), F_LABEL, C_DIM)
    testo(surf, valore, (x + 150, y), F_VAL, colore)


def draw_pannel(surf, x, y, w, h, title="", color=C_ACCENT):
    """Disegna il rettangolo di un pannello, con barra del titolo sopra."""
    rect = pygame.Rect(x, y, w, h)
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, rect, 2, border_radius=8)
    if title:
        barra = pygame.Rect(x, y, w, 28)
        pygame.draw.rect(surf, C_BORDER, barra, 2,
                          border_top_left_radius=8, border_top_right_radius=8)
        testo(surf, title, barra.center, F_TITLE, color, anchor="center")
    return rect


def draw_kv(surf, label, value, x, y, col_v=C_WHITE):
    testo(surf, label, (x, y), F_LABEL, C_DIM)
    testo(surf, value, (x + 120, y), F_VAL, col_v)



def draw_battery(surf, x, y, w, h, valore, valore_min, valore_max, colore, Name, unita=""):
    cap_h = 8
    corpo = pygame.Rect(x, y + cap_h + 2, w, h - cap_h - 2)
    pygame.draw.rect(surf, C_PANEL, corpo, border_radius=6)
    pygame.draw.rect(surf, C_BORDER, corpo, 2, border_radius=6)

    pct = clamp((valore - valore_min) / (valore_max - valore_min), 0.0, 1.0) if valore_max > valore_min else 0
    fill_h = int((corpo.height - 8) * pct)
    if fill_h > 0:
        fill = pygame.Rect(corpo.x + 4, corpo.bottom - 4 - fill_h, corpo.width - 8, fill_h)
        pygame.draw.rect(surf, colore, fill, border_radius=4)

    testo(surf, Name, (corpo.centerx, y ), F_SMALL, C_DIM, anchor="center")
    testo(surf, f"{valore}{unita}", (corpo.centerx, corpo.centery), F_VAL, C_WHITE, anchor="center")


def draw_servo_leds(surf, x, y, degrees, label, v, error=False, warning=False):
    """Disegna i 5 LED (grandi) della posizione servo + gradi + tensione, sempre visibili."""
    LED_W, LED_H, LED_GAP, N = 80, 36, 10, 5
    
    leds = [False] * N
    
    # Il centro è sempre acceso per mostrare che il sistema è attivo
    leds[2] = True
    
    # Logica per accendere i LED in base all'angolo (-90 a +90)
    # Sopra lo zero (Positivi)
    if degrees > 20: leds[3] = True
    if degrees > 60: leds[4] = True
    
    # Sotto lo zero (Negativi)
    if degrees < -20: leds[1] = True
    if degrees < -60: leds[0] = True

    testo(surf, label, (x + LED_W // 2, y - 20), F_VAL, C_TEXT, anchor="center")
    
    for i in range(N):
        # L'indice 0 sarà disegnato più in basso, il 4 più in alto
        y_led = y + (N - 1 - i) * (LED_H + LED_GAP)
        
        if error:
            col = C_RED  # servo NOT OK -> tutti i led rossi
        elif warning:
            col = C_YELLOW
        elif leds[i]:
            col = C_GREEN if i == 2 else C_ACCENT
        else:
            col = (22, 28, 42) # Colore spento
            
        pygame.draw.rect(surf, col, (x, y_led, LED_W, LED_H), border_radius=5)
        pygame.draw.rect(surf, C_BORDER, (x, y_led, LED_W, LED_H), 1, border_radius=5)

    y_info = y + N * (LED_H + LED_GAP) + 10
    testo(surf, "ERR!" if error else f"{int(degrees)}°",
          (x + LED_W // 2, y_info), F_VAL, C_RED if error else C_WHITE, anchor="center")
    testo(surf, f"{v:.2f} V", (x + LED_W // 2, y_info + 24), F_SMALL, C_DIM, anchor="center")


def draw_hud_box(surf, rect, value, label="", unit=""):
    pygame.draw.rect(surf, (22, 28, 44), rect, border_radius=5)
    pygame.draw.rect(surf, C_YELLOW, rect, 2, border_radius=5)
    if label:
        testo(surf, label, (rect.centerx, rect.top - 13), F_SMALL, C_DIM, anchor="center")
    testo(surf, f"{value:.1f}", rect.center, F_VAL, C_YELLOW, anchor="center")
    if unit:
        testo(surf, unit, (rect.centerx, rect.bottom + 6), F_SMALL, C_DIM, anchor="center")


def draw_horizon(surf, cx, cy, r, pitch_deg, roll_deg):
    diam = r * 2
    size = int(r * 4.2)
    ppd = r / 25.0
    split = size // 2 - int(pitch_deg * ppd)

    bg = pygame.Surface((size, size))
    bg.fill(C_SKY)
    if split < size:
        pygame.draw.rect(bg, C_GROUND, (0, max(0, split), size, size))
    if split <= 0:
        bg.fill(C_GROUND)
    if 0 <= split <= size:
        pygame.draw.line(bg, (205, 222, 255), (0, split), (size, split), 2)

    for p in [-20, -10, 10, 20]:
        py = split - int(p * ppd)
        if 4 < py < size - 4:
            lw = r // 4
            pygame.draw.line(bg, C_WHITE, (size // 2 - lw, py), (size // 2 + lw, py), 1)
            lbl = F_SMALL.render(f"{abs(p)}", True, C_WHITE)
            bg.blit(lbl, (size // 2 + lw + 4, py - lbl.get_height() // 2))
            bg.blit(lbl, (size // 2 - lw - lbl.get_width() - 4, py - lbl.get_height() // 2))

    rot = pygame.transform.rotate(bg, -roll_deg)
    rw, rh = rot.get_size()
    crop = pygame.Surface((diam, diam))
    crop.blit(rot, (0, 0), (rw // 2 - r, rh // 2 - r, diam, diam))
    mask = pygame.Surface((diam, diam), pygame.SRCALPHA)
    pygame.draw.circle(mask, (255, 255, 255, 255), (r, r), r)
    ca = crop.convert_alpha()
    ca.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    surf.blit(ca, (cx - r, cy - r))

    pygame.draw.circle(surf, C_BORDER, (cx, cy), r, 2)
    hw = r // 2
    pygame.draw.line(surf, C_CROSS, (cx - hw, cy), (cx - r // 6, cy), 3)
    pygame.draw.line(surf, C_CROSS, (cx + r // 6, cy), (cx + hw, cy), 3)
    pygame.draw.circle(surf, C_CROSS, (cx, cy), 4)


def draw_header(surf, stato):
    barra = pygame.Rect(0, 0, W, HEADER_H)
    pygame.draw.rect(surf, C_PANEL, barra)
    pygame.draw.line(surf, C_BORDER, (0, HEADER_H), (W, HEADER_H), 2)

    testo(surf, stato["MODALITA_VOLO"], (W // 2, HEADER_H // 2), F_HEAD, C_WHITE, anchor="center")

    if stato["Drone_in_volo"]:
        testo(surf, "IN FLIGHT", (W - 30, HEADER_H // 2), F_HEAD, C_ACCENT, anchor="midright")
    else:
        testo(surf, "ON GROUND", (W - 30, HEADER_H // 2), F_HEAD, C_DIM, anchor="midright")

    if stato["failsafe"]:
        testo(surf, "FAILSAFE", (W - 190, HEADER_H // 2), F_HEAD, C_RED, anchor="midright")

    colore = C_GREEN if stato["seriale_ok"] else C_RED
    etichetta = f"SERIALE {stato['porta_seriale']}" if stato["seriale_ok"] else "NO SERIALE"
    testo(surf, etichetta, (30, HEADER_H // 2), F_TITLE, colore, anchor="midleft")

    # numero satelliti, ben visibile: >3 verde, 2-3 giallo, 1-0 rosso
    n_sat = stato["num_satelliti"]
    if n_sat > 3:
        colore_sat = C_GREEN
    elif n_sat >= 2:
        colore_sat = C_YELLOW
    else:
        colore_sat = C_RED
    testo(surf, f"SATELLITI {n_sat}", (260, HEADER_H // 2), F_HEAD, colore_sat, anchor="midleft")


# Assicurati di avere il colore arancione definito tra le tue costanti
C_ORANGE = (255, 165, 0)

def draw_chip(surf, x, y, w, h, label, colore):
    """Piccolo indicatore diagnostico: riceve direttamente il colore da renderizzare."""
    pygame.draw.rect(surf, C_BG, (x, y, w, h), border_radius=6)
    pygame.draw.rect(surf, colore, (x, y, w, h), 2, border_radius=6)
    pygame.draw.circle(surf, colore, (x + 16, y + h // 2), 5)
    testo(surf, label, (x + 30, y + h // 2), F_LABEL, C_TEXT, anchor="midleft")

def draw_status_strip(surf, x, y, w, h, stato):
    """Riga di stato con tutti i flag diagnostici della telemetria."""
    draw_pannel(surf, x, y, w, h)

    # 1. Valutazione dedicata per lo stato del GPS
    err_gps = stato.get("ERRORE_GPS", 0) 
    if err_gps <= 0 and stato["gps_ok"] == False:
        colore_gps = C_RED      # Nessun fix o hardware bloccato
    elif err_gps == 1:
        colore_gps = C_ORANGE   # HDOP Scarso
    elif err_gps == 2:
        colore_gps = C_YELLOW   # HDOP Accettabile
    else:
        colore_gps = C_GREEN    # HDOP Eccellente (>=3)

    # 2. Creazione della lista dei chip accoppiando Etichetta e Colore
    chips = [
        ("IMU PRONTA", C_GREEN if stato["imu_pronto"] else C_RED),
        ("FLUSSO OTTICO", C_GREEN if stato["flusso_ottico_ok"] else C_YELLOW),
        ("BAROMETRO", C_GREEN if stato["baro_pronto"] else C_RED),
        ("SENS. CORRENTE", C_GREEN if stato["sensori_corrente_ok"] else C_YELLOW),
        ("NO SCHIANTO", C_GREEN if not stato["stato_schianto_rilevato"] else C_RED),
        ("In Volo", C_GREEN if stato["Drone_in_volo"] else C_RED),
        ("GPS", colore_gps)
    ]
    
    chip_w, chip_h, gap = 170, 26, 16
    start_x = x + 20
    cy = y + (h - chip_h) // 2
    
    # 3. Disegno dei chip
    for i, (label, colore) in enumerate(chips):
        draw_chip(surf, start_x + i * (chip_w + gap), cy, chip_w, chip_h, label, colore)



def draw_power_panel(surf, x, y, w, h, stato):
    draw_pannel(surf, x, y, w, h, "USCITA MIXER / GAS")

    thr = pygame.Rect(x + 30, y + 40, 75, h - 50)
    colore_gas = C_YELLOW if stato["out_gas_us"] >= CONFIG["GAS_PWM_WARN"] else C_GREEN
    

    kx = x + 145
    ky = y + 40
    if stato["MODALITA_VOLO"].strip().upper() != "MANUALE":
        draw_battery(surf, thr.x, thr.y, thr.width, thr.height, stato["out_gas_us"], CONFIG["GAS_MASSIMO_us"], CONFIG["GAS_MASSIMO_us"], colore_gas, "GAS", "us")
        testo(surf, "OUTPUT PID" , (kx, ky), F_TITLE, C_ACCENT)
        ky += 30
        draw_kv(surf, "PITCH", f"{stato['out_pitch']:.1f}", kx, ky)
        ky += 28
        draw_kv(surf, "ROLL", f"{stato['out_roll']:.1f}", kx, ky)
        ky += 32
        
    else:
        draw_battery(surf, thr.x, thr.y, thr.width, thr.height, stato["rc_2"], CONFIG["GAS_MASSIMO_us"], CONFIG["GAS_MASSIMO_us"], colore_gas, "GAS", "us")
        testo(surf, "OUTPUT MANUALE", (kx, ky), F_TITLE, C_ACCENT)
        ky += 30
        draw_kv(surf, "PITCH", f"{stato['rc_1']:.1f}", kx, ky)
        ky += 28
        draw_kv(surf, "ROLL", f"{stato['rc_0']:.1f}", kx, ky)
        ky += 32

    # limite termico: usa i due dati reali della telemetria (interruttore + soglia gas attuale)
    limite_agisce = stato["gas_limite_termico_us"] < 1950
    attivato = stato["limite_termico_attivato"]
    if attivato and limite_agisce:
        colore_limite, testo_limite = C_GREEN, "ATTIVO"
    elif attivato and not limite_agisce:
        colore_limite, testo_limite = C_WHITE, "PRONTO"
    elif not attivato and limite_agisce:
        colore_limite, testo_limite = C_RED, "DISATTIVO!"
    else:
        colore_limite, testo_limite = C_WHITE, "OFF"
    draw_kv(surf, "LIM.TERMICO", f"{testo_limite} ({stato['gas_limite_termico_us']}us)", kx, ky, col_v=colore_limite)
    ky += 30


def draw_panel_temperature(surf, x, y, w, h, stato):
    draw_pannel(surf, x, y, w, h, "TEMPERATURE")
    start_y = y + 55
    start_x = x + 20
    gap = 16
    w_batt = (w - 40 - gap * 3) // 4
    h_batt = h - 85

    if stato["temp_motore"] >= CONFIG["MIN_THROTTLE_START_TEMP_C"] and stato["temp_motore"] < CONFIG["MAX_THROTTLE_END_TEMP_C"]:
        colore_motore = C_YELLOW 
    elif stato["temp_motore"] >= CONFIG["MAX_THROTTLE_END_TEMP_C"]:
        colore_motore = C_RED
    else:
        colore_motore = C_GREEN

    if stato["temp_esc"] >= CONFIG["MIN_ESC_START_TEMP_C"] and stato["temp_esc"] < CONFIG["MIN_ESC_END_TEMP_C"]:
        colore_esc = C_YELLOW 
    elif stato["temp_esc"] >= CONFIG["MIN_ESC_END_TEMP_C"]:
        colore_esc = C_RED
    else:
        colore_esc = C_GREEN
    
    draw_battery(surf, start_x + (w_batt + gap) * 0, start_y, w_batt, h_batt, stato["temp_motore"], 0, 100, colore_motore, "ENGINE", " C°")
    draw_battery(surf, start_x + (w_batt + gap) * 1, start_y, w_batt, h_batt, stato["temp_esc"], 0, 100, colore_esc, "ESC", " C°")
    draw_battery(surf, start_x + (w_batt + gap) * 2, start_y, w_batt, h_batt, stato["temp_fusoliera"], 0, 100, C_GREEN, "FUSOLIERA", " C°")
    draw_battery(surf, start_x + (w_batt + gap) * 3, start_y, w_batt, h_batt, stato["temp_esterna"], 0, 100, C_GREEN, "ESTERNA", " C°")



def draw_panel_attitude(surf, x, y, w, h, stato):
    draw_pannel(surf, x, y, w, h, "ASSETTO / AMBIENTE")
    start_x = x + 20
    start_y = y + 34
    riga = 25


    lidar_ok = stato["alt_baro"] <= CONFIG["ALTITUDINE_MAX_LIDAR_m"]
    colore_lidar = C_GREEN if lidar_ok else C_WHITE
    colore_baro = C_DIM if not stato["baro_pronto"] else (C_WHITE if lidar_ok else C_GREEN)

    ottico_disp = stato["flusso_ottico_ok"]
    ottico_in_range = stato["alt_baro"] <= CONFIG["ALTITUDINE_MAX_OTTICO_m"]
    colore_ottico = C_DIM if not ottico_disp else (C_GREEN if ottico_in_range else C_WHITE)

    modulo = math.sqrt(stato["vel_ottica_x"] ** 2 + stato["vel_ottica_y"] ** 2)

    testo_valori(surf, "Alt Lidar :", f"{stato['alt_lidar']:.2f} m", start_x, start_y + riga * 0, colore_lidar)
    testo_valori(surf, "Alt Baro :", f"{stato['alt_baro']:.2f} m", start_x, start_y + riga * 1, colore_baro)
    testo_valori(surf, "Vento:", f"{stato['vento_vel_kmh']:.1f} km/h", start_x, start_y + riga * 2, C_WHITE)
    testo_valori(surf, "Direzione Vento:", f"{stato['vento_dir']:.0f}° ", start_x, start_y + riga * 3, C_WHITE)
    testo_valori(surf, "Vel ott X:", f"{stato['vel_ottica_x']:.2f}", start_x, start_y + riga * 4, C_WHITE if ottico_disp else C_DIM)
    testo_valori(surf, "Vel ott Y:", f"{stato['vel_ottica_y']:.2f}", start_x, start_y + riga * 5, C_WHITE if ottico_disp else C_DIM)
    testo_valori(surf, "Vel mod. ott:", f"{modulo*3.6:.2f} km/h", start_x, start_y + riga * 6, colore_ottico)

def draw_pfd(surf, cx, cy, r, panel_x, panel_w, panel_top, stato, is_pid):
    draw_horizon(surf, cx, cy, r, stato["pitch"], stato["roll"])

    spd_box = pygame.Rect(cx - r - 95, cy - 22, 75, 44)
    alt_box = pygame.Rect(cx + r + 20, cy - 22, 75, 44)
    draw_hud_box(surf, spd_box, stato["vel_aria_kmh"], "AIRSPD", "km/h")
    draw_hud_box(surf, alt_box, stato["altitudine"], "ALT", "m")

    # posizione GPS del velivolo, sempre visibile, angolo in alto a destra del pannello
    gx = panel_x + panel_w - 150
    info_y = cy + r + 40
    riga = 22
    lx = cx - 250
    rx = cx + 20

    testo(surf, "GPS", (gx, panel_top + 36), F_TITLE, C_ACCENT)
    testo(surf, f"Lat: {stato['lat']:.5f}", (gx, panel_top + 56), F_LABEL, C_WHITE)
    testo(surf, f"Lon: {stato['lon']:.5f}", (gx, panel_top + 74), F_LABEL, C_WHITE)
    testo(surf, f"Crociera : {stato['vel_crociera_kmh']:.1f} km/h", (lx-20, panel_top + 56), F_LABEL, C_WHITE)
    testo(surf, f"Avvicin : {stato['vel_avvicinamento_kmh']:.1f} km/h", (lx-20, panel_top + 74), F_LABEL, C_WHITE)

    # zona informativa sotto l'orizzonte: velocità (sinistra) e navigazione (destra)


    testo(surf, "VELOCITA'", (lx, info_y), F_TITLE, C_ACCENT)
    testo_valori(surf, "Aria km/h:", f"{stato['vel_aria_kmh']:.1f}", lx, info_y + 26, C_WHITE)
    testo_valori(surf, "Suolo km/h:", f"{stato['vel_suolo_gps_kmh']:.1f}", lx, info_y + 26 + riga, C_WHITE)
    testo_valori(surf, "Rotta attuale :", f"{stato['rotta_attuale_deg']:.1f} °", lx, info_y + 26 + riga * 2, C_WHITE)

    testo(surf, "ANGOLI EULERO", (rx, info_y+150), F_TITLE, C_ACCENT)
    pitch_abs = abs(stato["pitch"])
    roll_abs = abs(stato["roll"])
    colore_pitch = C_RED if pitch_abs >= CONFIG["MAX_PITCH_deg"] else (C_YELLOW if pitch_abs >= CONFIG["MAX_PITCH_deg"] - 5 else C_WHITE)
    colore_roll = C_RED if roll_abs >= CONFIG["MAX_ROLL_deg"] else (C_YELLOW if roll_abs >= CONFIG["MAX_ROLL_deg"] - 5 else C_WHITE)

    testo_valori(surf, "PITCH:", f"{stato['pitch']:.1f}", rx, info_y + 26+150, colore_pitch)
    testo_valori(surf, "YAW:", f"{stato['yaw']:.1f}", rx, info_y + 26 + riga+150, C_WHITE)
    testo_valori(surf, "ROLL:", f"{stato['roll']:.1f}", rx, info_y + 26 + riga * 2+150, colore_roll)

    testo(surf, "IMU CALL", (lx, info_y+150), F_TITLE, C_ACCENT)
    testo_valori(surf, "SYS:", f"{stato['imu_cal_sys']:.1f}", lx, info_y + 26+150, C_GREEN if stato['imu_cal_sys'] >= 3 else (C_YELLOW if stato['imu_cal_sys'] == 2 else C_RED))
    testo_valori(surf, "GYRO:", f"{stato['imu_cal_gyro']:.1f}", lx, info_y + 26+150 + riga, C_GREEN if stato['imu_cal_gyro'] >= 3 else (C_YELLOW if stato['imu_cal_gyro'] == 2 else C_RED))
    testo_valori(surf, "ACCEL :", f"{stato['imu_cal_accel']:.1f}", lx, info_y + 26+150 + riga * 2, C_GREEN if stato['imu_cal_accel'] >= 3 else (C_YELLOW if stato['imu_cal_accel'] == 2 else C_RED))
    testo_valori(surf, "MAH :", f"{stato['imu_cal_mag']:.1f}", lx, info_y + 26+150 + riga * 3, C_GREEN if stato['imu_cal_mag'] >= 3 else (C_YELLOW if stato['imu_cal_mag'] == 2 else C_RED))

    if is_pid:
        testo(surf, "NAVIGAZIONE", (rx, info_y), F_TITLE, C_ACCENT)
        colore_dist = C_GREEN if stato["dist_target"] > CONFIG["DIST_TGT_WARN"] else C_PURPLE
        testo_valori(surf, "Dist target m:", f"{stato['dist_target']:.0f}", rx, info_y + 26, colore_dist)
        testo_valori(surf, "Rotta target °:", f"{stato['rotta_target']:.0f}", rx, info_y + 26 + riga, C_WHITE)
        testo_valori(surf, "Errore rotta °:", f"{stato['errore_rotta']:.1f}", rx, info_y + 26 + riga * 2, C_WHITE)
        testo_valori(surf, "Roll target °:", f"{stato['roll_target']:.1f}", rx, info_y + 26 + riga * 3, C_WHITE)
        testo_valori(surf, "altitudine_target:", f"{stato['altitudine_target']:.1f} m", rx, info_y + 26 + riga * 4, C_WHITE)


def draw_servi_panel(surf, x, y, w, h, stato):
    draw_pannel(surf, x, y, w, h, "SERVI" if stato["servo_sicurezza"] else "SICUREZZA SERVI DISATTIVATA", C_ACCENT if stato["servo_sicurezza"] else C_YELLOW)

    # Estrazione degli stati logici (generati e inviati dal codice C)
    esx_ok = stato.get("servo_est_sx_ok", True)
    isx_ok = stato.get("servo_int_sx_ok", True)
    idx_ok = stato.get("servo_int_dx_ok", True)
    edx_ok = stato.get("servo_est_dx_ok", True)

    # Struttura: (Gradi, Etichetta, Tensione, Stato Proprio, Stato del Simmetrico)
    servi = [
        (stato["deg_esx"], "Est SX", stato["v_est_sx"], esx_ok, edx_ok),
        (stato["deg_isx"], "Int SX", stato["v_int_sx"], isx_ok, idx_ok),
        (stato["deg_idx"], "Int DX", stato["v_int_dx"], idx_ok, isx_ok),
        (stato["deg_edx"], "Est DX", stato["v_est_dx"], edx_ok, esx_ok),
    ]

    gap = (w - 60) // 4
    start_x = x + 37
    start_y = y + 70

    for i, (deg, lbl, v, is_ok, simmetrico_ok) in enumerate(servi):
        errore = False  # Condizione critica (Rosso)
        avviso = False  # Condizione riflessa dal simmetrico (Giallo)
        
        if stato["servo_sicurezza"]:
            if not is_ok:
                errore = True
            elif not simmetrico_ok:
                avviso = True
                
        # Passa l'argomento warning (o equivalente) alla funzione che disegna i led
        draw_servo_leds(surf, start_x + i * gap, start_y, deg, lbl, v, error=errore, warning=avviso)

def draw_battery_panel(surf, x, y, w, h, stato):
    draw_pannel(surf, x, y, w, h, "ALIMENTAZIONE" if stato["alimentazione_sicurezza"] else "SICUREZZA ALIMENTAZIONE DISATTIVATA", C_ACCENT if stato["alimentazione_sicurezza"] else C_RED)

    kx = x + 40
    ky = y + 45
    bar_w, bar_h = 60, 100

    rele_attivo = stato["rele_attivato"]  # dato reale (releAttivato)

    colore_v_motor = C_YELLOW if stato["v_motore"] < CONFIG["VBAR_MOTOR_MIN"] else C_GREEN
    colore_v_teensy = C_YELLOW if stato["v_teensy"] < CONFIG["VBAR_TEENSY_MIN"] else C_GREEN
    colore_corrente = C_TEXT if stato["sensori_corrente_ok"] else C_RED

    v_motor_rect = pygame.Rect(kx, ky, bar_w, bar_h)
    draw_battery(surf, v_motor_rect.x, v_motor_rect.y, v_motor_rect.width, v_motor_rect.height, stato["v_motore"], CONFIG["VBAR_MOTOR_MIN"], CONFIG["VBAR_MOTOR_MAX"], colore_v_motor, "MOTORE", "V")
    testo(surf, "V MOTORE", (v_motor_rect.centerx, v_motor_rect.bottom + 12), F_SMALL, C_DIM, anchor="center")
    testo(surf, f"{stato['i_motore']:.1f} A", (v_motor_rect.centerx, v_motor_rect.bottom + 28), F_SMALL, colore_corrente, anchor="center")

    v_teensy_rect = pygame.Rect(x + w - 40 - bar_w, ky, bar_w, bar_h)
    draw_battery(surf, v_teensy_rect.x, v_teensy_rect.y, v_teensy_rect.width, v_teensy_rect.height, stato["v_teensy"], CONFIG["VBAR_TEENSY_MIN"], CONFIG["VBAR_TEENSY_MAX"], colore_v_teensy, "TEENSY", "V")
    testo(surf, "V TEENSY", (v_teensy_rect.centerx, v_teensy_rect.bottom + 12), F_SMALL, C_DIM, anchor="center")
    testo(surf, f"{stato['i_teensy']:.2f} A", (v_teensy_rect.centerx, v_teensy_rect.bottom + 28), F_SMALL, colore_corrente, anchor="center")

    # asse/linea di collegamento: BATTERIA MOTORE -> RELE' -> BATTERIA TEENSY
    linea_y = ky + bar_h // 2
    x1 = v_motor_rect.right + 8
    x2 = v_teensy_rect.left - 8
    colore_linea = C_GREEN if rele_attivo else C_BORDER
    pygame.draw.line(surf, colore_linea, (x1, linea_y), (x2, linea_y), 3 if rele_attivo else 2)

    rele_cx = (x1 + x2) // 2
    pygame.draw.circle(surf, C_PANEL, (rele_cx, linea_y), 14)
    pygame.draw.circle(surf, colore_linea, (rele_cx, linea_y), 14, 2)
    testo(surf, "R", (rele_cx, linea_y), F_SMALL, colore_linea, anchor="center")

    if rele_attivo:
        # piccolo flusso animato dalla batteria motore verso la batteria teensy
        t = (pygame.time.get_ticks() % 1000) / 1000.0
        for i in range(3):
            fase = (t + i / 3.0) % 1.0
            if fase < 0.5:
                px = int(x1 + (rele_cx - x1) * (fase * 2))
            else:
                px = int(rele_cx + (x2 - rele_cx) * ((fase - 0.5) * 2))
            pygame.draw.circle(surf, C_GREEN, (px, linea_y), 3)

    status_y = ky + bar_h + 46


    # carica residua e autonomia: motore e teensy affiancati
    info_y = status_y + 10
    col1_x = x + 30
    col2_x = x + w // 2 + 15

    testo(surf, "MOTORE", (col1_x, info_y), F_TITLE, C_ACCENT)
    testo(surf, f"Carica: {stato['carica_motore_pct']:.0f}%", (col1_x, info_y + 24), F_LABEL, colore_v_motor)
    testo(surf, f"Autonomia: {stato['autonomia_motore']:.0f} min ", (col1_x, info_y + 44), F_LABEL, C_WHITE)

    testo(surf, "TEENSY", (col2_x, info_y), F_TITLE, C_ACCENT)
    testo(surf, f"Carica: {stato['carica_teensy_pct']:.0f}%", (col2_x, info_y + 24), F_LABEL, colore_v_teensy)
    testo(surf, f"Autonomia: {stato['autonomia_teensy']:.0f} min ", (col2_x, info_y + 44), F_LABEL, C_WHITE)


MARGIN = 18
HEADER_H = 46
STATUS_H = 40

LEFT_X = MARGIN
LEFT_W = 430

RIGHT_W = 430
RIGHT_X = W - MARGIN - RIGHT_W

CENTER_X = LEFT_X + LEFT_W + MARGIN
CENTER_W = RIGHT_X - CENTER_X - MARGIN

TOP_Y = HEADER_H + MARGIN + STATUS_H + MARGIN
LOG_H = 120
BOTTOM_Y = H - MARGIN - LOG_H
CONTENT_H = BOTTOM_Y - MARGIN - TOP_Y

# colonna sinistra: power / temperature / assetto-ambiente
POWER_H = 205
TEMP_H = 210
STATES_H = CONTENT_H - POWER_H - TEMP_H - MARGIN * 2

# colonna destra: servi / battery
BATTERY_H = 300
SERVI_H = CONTENT_H - BATTERY_H - MARGIN

# raggio orizzonte artificiale (spostato più in alto, non centrato)
PFD_R = min((CENTER_W - 260) // 2, 150)
PFD_CX = CENTER_X + CENTER_W // 2
PFD_CY = TOP_Y + 40 + PFD_R



SERIAL_PORT = "COM5"
SERIAL_BAUDRATE = CONFIG["BAUD_RATE_LORA"]
SERIAL_TIMEOUT = 0.02
TELEMETRY_FIELDS = 70
TELEMETRY_TIMEOUT = 2.0
MAX_SERIAL_LINES_PER_FRAME = 24
MAX_LOG_LINES = 10
log_messages = []


def aggiungi_log(messaggio):
    timestamp = time.strftime("%H:%M:%S")
    log_messages.append(f"[{timestamp}] {messaggio}")
    del log_messages[:-MAX_LOG_LINES]


def draw_log_panel(surf, x, y, w, h):
    draw_pannel(surf, x, y, w, h, "LOG")
    offset_x = 20
    start_y = 45
    line_height = 20

    if not log_messages:
        testo(surf, "( Nessun log ricevuto )", (x + offset_x, y + start_y), F_LABEL, C_DIM)
        return

    visible_lines = max(1, (h - start_y - 8) // line_height)
    for i, msg in enumerate(log_messages[-visible_lines:]):
        testo(surf, msg[:150], (x + offset_x, y + start_y + i * line_height), F_LABEL, C_DIM)


def genera_stato_iniziale():
    """Crea uno stato neutro: nessun dato viene mostrato come valido all'avvio."""
    return {
        "seriale_ok": False,
        "porta_seriale": SERIAL_PORT,
        # --- STATO GENERALE ---
        "MODALITA_VOLO": "SCONOSCIUTO",
        "Drone_in_volo": False,
        "failsafe": False,
        "stato_schianto_rilevato": False,
        "imu_pronto": False,
        "flusso_ottico_ok": False,
        "baro_pronto": False,
        "sensori_corrente_ok": False,
        "gps_ok": False,
        "imu_cal_sys": 0,
        "imu_cal_gyro": 0,
        "imu_cal_accel": 0,
        "imu_cal_mag": 0,

        # --- ALIMENTAZIONE ---
        "v_motore": 0.0,
        "v_teensy": 0.0,
        "carica_teensy_pct": 0.0,
        "carica_motore_pct": 0.0,
        "autonomia_teensy": 0.0,
        "autonomia_motore": 0.0,
        "i_teensy": 0.0,
        "i_motore": 0.0,
        "v_int_sx": 0.0, "v_int_dx": 0.0, "v_est_sx": 0.0, "v_est_dx": 0.0,
        "servo_sicurezza": False,

        # --- ASSETTO E QUOTA ---
        "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
        "altitudine": 0.0,
        "alt_lidar": 0.0,
        "alt_baro": 0.0,
        "altitudine_target": 0.0,

        # --- VELOCITA' ---
        "vel_aria_kmh": 0.0,
        "vel_suolo_gps_kmh": 0.0,
        "vento_vel_kmh": 0.0,
        "vento_dir": 0.0,
        "vel_crociera_kmh": 0.0,
        "vel_avvicinamento_kmh": 0.0,

        # --- NAVIGAZIONE ---
        "dist_target": 0.0,
        "rotta_target": 0.0,
        "roll_target": 0.0,
        "rotta_attuale_deg": 0.0,

        # --- INPUT RADIOCOMANDO ---
        "rc_1": 0, "rc_0": 0, "rc_2": 0,

        # --- OUTPUT PID/MIXER ---
        "out_pitch": 0, "out_roll": 0, "out_gas_us": 0,

        # --- POSIZIONE FISICA SERVI ---
        "deg_isx": 0, "deg_idx": 0, "deg_esx": 0, "deg_edx": 0,

        # --- TEMPERATURE E LIMITI ---
        "limite_termico_attivato": False,
        "temp_motore": 0.0, "temp_fusoliera": 0.0, "temp_esterna": 0.0, "temp_esc": 0.0,
        "gas_limite_termico_us": 0,

        # --- SATELLITI E COORDINATE GPS ---
        "num_satelliti": 0,
        "lat": 0.0,
        "lon": 0.0,
        "ERRORE_GPS": 0,
        "errore_rotta": 0.0,

        # --- STATI FINALI ---
        "alimentazione_sicurezza": False,
        "rele_attivato": False,
        "motore_disabilitato_terra": False,

        # --- FLUSSO OTTICO ---
        "vel_ottica_x": 0.0, "vel_ottica_y": 0.0,
    }


def azzera_stato(stato):
    """Porta lo stato in una condizione sicura quando la telemetria scade."""
    for key in list(stato):
        if key == "porta_seriale":
            continue
        if isinstance(stato[key], bool):
            stato[key] = False
        elif isinstance(stato[key], str):
            stato[key] = "SCONOSCIUTO"
        elif isinstance(stato[key], float):
            stato[key] = 0.0
        elif isinstance(stato[key], int):
            stato[key] = 0
    stato["seriale_ok"] = False


def apri_seriale():
    if serial is None:
        aggiungi_log("Modulo pyserial non installato: modalità demo")
        return None
    try:
        connessione = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=SERIAL_TIMEOUT)
        aggiungi_log(f"Seriale {SERIAL_PORT} connessa a {SERIAL_BAUDRATE} baud")
        return connessione
    except serial.SerialException as errore:
        aggiungi_log(f"Seriale non disponibile: {errore}")
        return None


def chiudi_seriale(connessione):
    if connessione and connessione.is_open:
        connessione.close()


def aggiorna_stato_da_telemetria(stato, riga_seriale):
    if not riga_seriale or not riga_seriale.startswith("$,"):
        aggiungi_log(f"Pacchetto telemetria non valido: {riga_seriale}")
        return False

    parti = riga_seriale.split(",")
    if len(parti) != TELEMETRY_FIELDS:
        aggiungi_log(f"Numero di campi nella telemetria non valido: {len(parti)}")
        return False

    try:
        valori = {
            "MODALITA_VOLO": parti[1],
            "Drone_in_volo": parti[2] == "1",
            "failsafe": parti[3] == "1",
            "stato_schianto_rilevato": parti[4] == "1",
            "imu_pronto": parti[5] == "1",
            "flusso_ottico_ok": parti[6] == "1",
            "baro_pronto": parti[7] == "1",
            "sensori_corrente_ok": parti[8] == "1",
            "gps_ok": parti[9] == "1",
            "imu_cal_sys": int(parti[10]), "imu_cal_gyro": int(parti[11]),
            "imu_cal_accel": int(parti[12]), "imu_cal_mag": int(parti[13]),
            "v_motore": float(parti[14]), "v_teensy": float(parti[15]),
            "carica_teensy_pct": float(parti[16]), "carica_motore_pct": float(parti[17]),
            "autonomia_teensy": float(parti[18]), "autonomia_motore": float(parti[19]),
            "i_teensy": float(parti[20]), "i_motore": float(parti[21]),
            "v_int_sx": float(parti[22]), "v_int_dx": float(parti[23]),
            "v_est_sx": float(parti[24]), "v_est_dx": float(parti[25]),
            "servo_sicurezza": parti[26] == "1",
            "pitch": float(parti[27]), "roll": float(parti[28]), "yaw": float(parti[29]),
            "altitudine": float(parti[30]), "alt_lidar": float(parti[31]),
            "alt_baro": float(parti[32]), "altitudine_target": float(parti[33]),
            "vel_aria_kmh": float(parti[34]), "vel_suolo_gps_kmh": float(parti[35]),
            "vento_vel_kmh": float(parti[36]), "vento_dir": float(parti[37]),
            "vel_crociera_kmh": float(parti[38]), "vel_avvicinamento_kmh": float(parti[39]),
            "dist_target": float(parti[40]), "rotta_target": float(parti[41]),
            "roll_target": float(parti[42]), "rotta_attuale_deg": float(parti[43]),
            "rc_1": int(parti[44]), "rc_0": int(parti[45]), "rc_2": int(parti[46]),
            "out_pitch": int(parti[47]), "out_roll": int(parti[48]), "out_gas_us": int(parti[49]),
            "deg_isx": int(parti[50]), "deg_idx": int(parti[51]),
            "deg_esx": int(parti[52]), "deg_edx": int(parti[53]),
            "limite_termico_attivato": parti[54] == "1",
            "temp_motore": float(parti[55]), "temp_fusoliera": float(parti[56]),
            "temp_esterna": float(parti[57]), "temp_esc": float(parti[58]),
            "gas_limite_termico_us": int(parti[59]),
            "num_satelliti": int(parti[60]), "lat": float(parti[61]), "lon": float(parti[62]),
            "ERRORE_GPS": int(float(parti[63])), "errore_rotta": float(parti[64]),
            "alimentazione_sicurezza": parti[65] == "1", "rele_attivato": parti[66] == "1",
            "motore_disabilitato_terra": parti[67] == "1",
            "vel_ottica_x": float(parti[68]), "vel_ottica_y": float(parti[69]),
        }
    except (ValueError, IndexError):
        aggiungi_log(f"Errore durante l'elaborazione del pacchetto telemetria: {riga_seriale}")
        return False

    stato.update(valori)
    stato["seriale_ok"] = True
    stato["porta_seriale"] = SERIAL_PORT
    return True


def leggi_seriale(connessione, stato):
    ultimo_pacchetto = None
    if not connessione:
        return ultimo_pacchetto

    for _ in range(MAX_SERIAL_LINES_PER_FRAME):
        if connessione.in_waiting <= 0:
            break
        linea = connessione.readline().decode("utf-8", errors="ignore").strip()
        if linea.startswith("$,"):
            if aggiorna_stato_da_telemetria(stato, linea):
                ultimo_pacchetto = time.monotonic()
            continue
        if linea.startswith("MSG,"):
            aggiungi_log(linea.split("MSG,", 1)[1].strip())
    return ultimo_pacchetto


def main():
    stato = genera_stato_iniziale()
    connessione = apri_seriale()
    ultimo_pacchetto = None

    try:
        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return

            try:
                ricevuto = leggi_seriale(connessione, stato)
                if ricevuto is not None:
                    ultimo_pacchetto = ricevuto
                if ultimo_pacchetto and time.monotonic() - ultimo_pacchetto > TELEMETRY_TIMEOUT:
                    azzera_stato(stato)
                    ultimo_pacchetto = None
                    aggiungi_log("Telemetria scaduta: stato portato in sicurezza")
            except (OSError, getattr(serial, "SerialException", OSError)) as errore:
                aggiungi_log(f"Errore seriale: {errore}")
                chiudi_seriale(connessione)
                connessione = None
                azzera_stato(stato)

            is_pid = stato["MODALITA_VOLO"].strip().upper() != "MANUALE"
            screen.fill(C_BG)
            draw_header(screen, stato)
            draw_status_strip(screen, MARGIN, HEADER_H + MARGIN // 2, W - 2 * MARGIN, STATUS_H, stato)
            draw_power_panel(screen, LEFT_X, TOP_Y, LEFT_W, POWER_H, stato)
            draw_panel_temperature(screen, LEFT_X, TOP_Y + POWER_H + MARGIN, LEFT_W, TEMP_H, stato)
            draw_panel_attitude(screen, LEFT_X, TOP_Y + POWER_H + MARGIN + TEMP_H + MARGIN, LEFT_W, STATES_H, stato)
            draw_pannel(screen, CENTER_X, TOP_Y, CENTER_W, CONTENT_H, "ORIZZONTE ARTIFICIALE")
            draw_pfd(screen, PFD_CX, PFD_CY, PFD_R, CENTER_X, CENTER_W, TOP_Y, stato, is_pid)
            draw_servi_panel(screen, RIGHT_X, TOP_Y, RIGHT_W, SERVI_H, stato)
            draw_battery_panel(screen, RIGHT_X, TOP_Y + SERVI_H + MARGIN, RIGHT_W, BATTERY_H, stato)
            draw_log_panel(screen, MARGIN, BOTTOM_Y, W - 2 * MARGIN, LOG_H)
            pygame.display.flip()
            clock.tick(30)
    finally:
        chiudi_seriale(connessione)
        pygame.quit()

if __name__ == "__main__":
    main()