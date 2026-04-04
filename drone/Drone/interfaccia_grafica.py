import pygame
import sys
import serial  
import threading 
SERIAL_PORT = "COM3"  
BAUD_RATE = 9600

pygame.init()

W, H   = 1280, 720
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("GCS – Primary Flight Display")
clock  = pygame.time.Clock()

C_BG     = ( 8,  11,  18)   # sfondo principale
C_PANEL  = (14,  18,  28)   # sfondo pannello
C_BORDER = (36,  50,  76)   # bordi
C_ACCENT = ( 0, 188, 255)   # ciano – titoli e accenti
C_TEXT   = (205, 218, 240)  # testo standard
C_DIM    = ( 82,  98, 126)  # testo secondario / etichette
C_WHITE  = (242, 246, 255)  # testo luminoso / valori
C_GREEN  = ( 40, 210,  88)  # stato OK
C_YELLOW = (255, 200,   0)  # throttle / HUD
C_RED    = (218,  48,  48)  # allarme / errore
C_SKY    = ( 28,  96, 178)  # cielo orizzonte
C_GROUND = (124,  80,  30)  # terra orizzonte
C_CROSS  = (255, 210,   0)  # crosshair orizzonte
F_HEAD  = pygame.font.SysFont("consolas", 21, bold=True)  # header principale
F_TITLE = pygame.font.SysFont("consolas", 13, bold=True)  # titoli pannelli
F_VAL   = pygame.font.SysFont("consolas", 15, bold=True)  # valori numerici
F_LABEL = pygame.font.SysFont("consolas", 13)             # etichette
F_SMALL = pygame.font.SysFont("consolas", 12)             # testo piccolo

# ── Orizzonte artificiale 
PFD_CX, PFD_CY, PFD_R = W // 2, 245, 142

# ── Box velocità e quota ai lati dell'orizzonte 
BOX_W, BOX_H, BOX_GAP = 66, 34, 18
SPD_BOX = pygame.Rect(PFD_CX - PFD_R - BOX_GAP - BOX_W, PFD_CY - BOX_H // 2, BOX_W, BOX_H)
ALT_BOX = pygame.Rect(PFD_CX + PFD_R + BOX_GAP,          PFD_CY - BOX_H // 2, BOX_W, BOX_H)

# ── Pannelli laterali 
POWER_PANEL = pygame.Rect( 24, 108, 378, 272)
TEMP_PANEL  = pygame.Rect( 24, 390, 378, 122)
SPEED_PANEL = pygame.Rect( 24, 522, 378, 182)
SERVI_PANEL = pygame.Rect(878, 108, 378, 200)
VSERV_PANEL = pygame.Rect(878, 320, 378, 193)
NAV_PANEL   = pygame.Rect(428, 522, 424, 140)
ORIENTATION_PANEL = pygame.Rect(428, 391, 424, 100)
BATTERY_PANNEL =pygame.Rect(878, 522, 378, 180)

# ── Colonne servi nel pannello SERVI
SERVI_X   = SERVI_PANEL.x + 28
SERVI_Y   = SERVI_PANEL.y + 52
SERVI_GAP = 88


T = {
    "mode":       "ATTESA",
    "satellites":  0,
    "v_motor":     0.0,   
    "throttle":    0.0,    
    "pitch":       0.0,
    "roll":        0.0,
    "yaw":         0.0,
    "altitude":    0.0,   
    "spd_pitot":   0.0,  
    "spd_gps":     0.0,   
    "spd_ms":      0.0,   
    "dist_target": 0.0,
    "hdg_target":  0.0,
    "roll_target": 0.0,
    "rc_pitch":    0.0,
    "rc_roll":     0.0,
    "rc_gas":      0.0,
    "pid_pitch":   0.0,
    "pid_roll":    0.0,
    "pid_gas":     0.0,
    "deg_isx":    90.0,
    "deg_idx":    90.0,
    "deg_esx":    90.0,
    "deg_edx":    90.0,
    "v_isx":       0.0,
    "v_idx":       0.0,
    "v_esx":       0.0,
    "v_edx":       0.0,
    "t_motor":     0.0,
    "t_teensy":    0.0,
    "v_teensy":        0.0,
    "alarm_failsafe":  False,
    "alarm_batt_motor":False,
    "alarm_relay":     False,
    "alarm_batt_teensy":False,
    "alarm_crash":     False,
    "in_flight":       False,
    "ok_isx": True, "ok_idx": True, "ok_esx": True, "ok_edx": True,
    "spd_fused":       0.0,
    "lat":             0.0,
    "lon":             0.0,
    "relay":           False,
}

def clamp(v, lo, hi):
    return max(lo, min(v, hi))

def parse_telemetry(line):
    """
    Formato CSV (36 campi, indice 0 = '$') — sincronizzato con inviaTelemetria() v2:
      0  : $
      1  : modalità volo        (1=Manuale, 2=Auto, 3=Failsafe)
      2  : codiceAllarme        bitmask (b0=failsafe, b1=battMot, b2=relè, b3=battTsy, b4=schianto, b5=inVolo)
      3  : V Motore             [V]
      4  : V Teensy             [V]
      5  : V Servo Int SX       [V]
      6  : V Servo Int DX       [V]
      7  : V Servo Est SX       [V]
      8  : V Servo Est DX       [V]
      9  : Salute servi         stringa 4 char "1111" (IntSX IntDX EstSX EstDX)
      10 : Pitch                [°]
      11 : Roll                 [°]
      12 : Yaw                  [°]
      13 : Altitudine relativa  [m]
      14 : Velocità Pitot       [km/h]
      15 : Velocità GPS         [km/h]
      16 : Velocità stimata     [km/h]
      17 : Distanza target      [m]
      18 : Rotta verso target   [°]
      19 : Target roll (L1)     [°]
      20 : RC Pitch             [µs 172-1811]
      21 : RC Roll              [µs]
      22 : RC Gas               [µs]
      23 : PID out Pitch        [µs]
      24 : PID out Roll         [µs]
      25 : PID out Gas          [µs ~1000-2000]
      26 : Pos Servo Int SX     [°]
      27 : Pos Servo Int DX     [°]
      28 : Pos Servo Est SX     [°]
      29 : Pos Servo Est DX     [°]
      30 : Temperatura motore   [°C]
      31 : Temperatura avionica [°C]
      32 : Satelliti GPS
      33 : Latitudine           [°]
      34 : Longitudine          [°]
      35 : Relè attivato        (0/1)
    """
    global T
    try:
        f = line.strip().split(',')
        if len(f) < 36 or f[0] != '$':
            print(f"[WARN] Pacchetto non valido (len={len(f)})")
            return
        # ── Modalità volo ────────────────────────────────────────────────
        T["mode"] = {1: "MANUALE", 2: "AUTO PID", 3: "FAILSAFE!"}.get(int(f[1]), "SCONOSCIUTA")
        
        # ── Bitmask allarmi ──────────────────────────────────────────────
        alarm = int(f[2])
        T["alarm_failsafe"] = bool(alarm & 1)
        T["alarm_batt_motor"] = bool(alarm & 2)
        T["alarm_relay"]    = bool(alarm & 4)
        T["alarm_batt_teensy"]= bool(alarm & 8)
        T["alarm_crash"]   = bool(alarm & 16)
        T["in_flight"]     = bool(alarm & 32)

        # ── Alimentazione ────────────────────────────────────────────────
        T["v_motor"]  = float(f[3])
        T["v_teensy"] = float(f[4])
        T["v_isx"] = float(f[5])
        T["v_idx"] = float(f[6])
        T["v_esx"]  = float(f[7])
        T["v_edx"]  = float(f[8])

        # ── Salute servi (stringa "1111") ────────────────────────────────
        health    = f[9].strip()
        T["ok_isx"] = len(health) > 0 and health[0] == '1'
        T["ok_idx"] = len(health) > 1 and health[1] == '1'
        T["ok_esx"] = len(health) > 2 and health[2] =='1'
        T["ok_edx"] = len(health) > 3 and health[3] == '1'

        # ── Assetto ──────────────────────────────────────────────────────
        T["pitch"]  = float(f[10])
        T["roll"]  = float(f[11])
        T["yaw"] = float(f[12])
        T["altitude"] = float(f[13])

        # ── Velocità ─────────────────────────────────────────────────────
        T["spd_pitot"]= float(f[14])
        T["spd_gps"] = float(f[15])
        T["spd_fused"]= float(f[16])
        T["spd_ms"]  = T["spd_fused"] / 3.6   # km/h → m/s per l'HUD

        # ── Navigazione ──────────────────────────────────────────────────
        T["dist_target"] = float(f[17])
        T["hdg_target"]  = float(f[18])
        T["roll_target"] = float(f[19])

        # ── Input RC grezzo [µs] ─────────────────────────────────────────
        T["rc_pitch"] = float(f[20])
        T["rc_roll"] = float(f[21])
        T["rc_gas"] = float(f[22])

        # ── Output PID/mixer ─────────────────────────────────────────────
        T["pid_pitch"]= float(f[23])
        T["pid_roll"] = float(f[24])
        # outGas in µs (range tipico 1000-2000); normalizzato 0.0-1.0 per la barra
        T["pid_gas"] = clamp((float(f[25]) - 1000.0) / 1000.0, 0.0, 1.0)
        T["throttle"]  = T["pid_gas"]

        # ── Posizione fisica servi [°] ───────────────────────────────────
        T["deg_isx"] = float(f[26])
        T["deg_idx"]= float(f[27])
        T["deg_esx"]= float(f[28])
        T["deg_edx"] = float(f[29])

        # ── Temperature ──────────────────────────────────────────────────
        T["t_motor"]  = float(f[30])
        T["t_teensy"] = float(f[31])

        # ── GPS ──────────────────────────────────────────────────────────
        T["satellites"] = int(f[32])
        T["lat"]  = float(f[33])
        T["lon"]  = float(f[34])

        # ── Relè ─────────────────────────────────────────────────────────
        T["relay"] = f[35].strip() == '1'

    except Exception as e:
        print(f"[ERR] Pacchetto corrotto saltato: {e}")

def read_from_serial():
    try:

        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"[INFO] Connesso a {SERIAL_PORT} a {BAUD_RATE} baud.")
        
        while True:
            line_bytes = ser.readline()
            
            if line_bytes:

                line_str = line_bytes.decode('utf-8', errors='ignore').strip()
                
                parse_telemetry(line_str)
                
    except serial.SerialException as e:
        print(f"[ERRORE CRITICO] Impossibile aprire la porta {SERIAL_PORT}: {e}")
        print("Assicurati che l'USB sia collegata e di aver scritto la COM giusta.")

def text(surf, s, pos, font, color, anchor="topleft"):

    img = font.render(s, True, color)
    surf.blit(img, img.get_rect(**{anchor: pos}))

def draw_panel(surf, rect, title=""):

    BAR_H = 26
    pygame.draw.rect(surf, C_PANEL,rect, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, rect, 2, border_radius=8)
    if title:
        bar = pygame.Rect(rect.x, rect.y, rect.w, BAR_H)
        pygame.draw.rect(surf, C_BORDER, bar, 2,border_top_left_radius=8, border_top_right_radius=8)
        text(surf, title, bar.center, F_TITLE, C_ACCENT, anchor="center")

def draw_kv(surf, label, value, x, y, col_v=C_WHITE, col_k=C_DIM,):
    text(surf, label, (x, y), F_LABEL, col_k)
    text(surf, value, (x + 110, y), F_VAL, col_v)

def draw_vbar(surf, rect, value, vmin, vmax, color=C_GREEN):
    pygame.draw.rect(surf, C_PANEL,  rect, border_radius=5)
    pygame.draw.rect(surf, C_BORDER, rect, 2, border_radius=5)
    if vmax > vmin:
        pct = clamp((value - vmin) / (vmax - vmin), 0.0, 1.0)
        fill_h = int((rect.height - 6) * pct)
        if fill_h > 0:
            bar_col = C_RED if pct < 0.2 else color
            fill    = pygame.Rect(rect.x + 3, rect.bottom - 3 - fill_h,rect.width - 6, fill_h)
            pygame.draw.rect(surf, bar_col, fill, border_radius=3)
    text(surf, f"{value:.2f}", rect.center, F_SMALL, C_WHITE, anchor="center")

def draw_servo_leds(surf, x, y, degrees, label, v_servo, error=False):
    LED_W, LED_H, LED_GAP, N = 40, 20, 5, 5
    delta = degrees - 90
    leds  = [False] * N
    leds[2] = True
    if delta >  10: leds[3] = True
    if delta >  25: leds[4] = True
    if delta < -10: leds[1] = True
    if delta < -25: leds[0] = True

    text(surf, label, (x + LED_W // 2, y - 18), F_SMALL, C_TEXT, anchor="center")
    for i in range(N):
        y_led = y + (N - 1 - i) *(LED_H + LED_GAP)
        if error:
            col = C_RED
        elif leds[i]:
            col = C_GREEN if i == 2 else C_ACCENT
        else:
            col = (22, 28, 42)
        pygame.draw.rect(surf, col, (x, y_led, LED_W, LED_H), border_radius=3)
        pygame.draw.rect(surf, C_BORDER, (x, y_led, LED_W, LED_H), 1, border_radius=3)

    y_info = y + N * (LED_H + LED_GAP) + 6
    text(surf, "ERR!" if error else f"{int(degrees)}°",
         (x + LED_W // 2, y_info), F_SMALL,C_RED if error else C_WHITE, anchor="center")

def draw_hud_box(surf, rect, value, label="", unit=""):
    pygame.draw.rect(surf, (22, 28, 44), rect, border_radius=5)
    pygame.draw.rect(surf, C_YELLOW,rect, 2,  border_radius=5)
    if label:
        text(surf, label, (rect.centerx,rect.top - 13), F_SMALL, C_DIM,  anchor="center")
    text(surf, f"{value:.1f}", rect.center, F_VAL, C_YELLOW, anchor="center")
    if unit:
        text(surf, unit,(rect.centerx, rect.bottom + 6), F_SMALL, C_DIM, anchor="center")

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
            lw  = r // 4
            pygame.draw.line(bg, C_WHITE, (size // 2 - lw, py), (size // 2 + lw, py), 1)
            lbl = F_SMALL.render(f"{abs(p)}", True, C_WHITE)
            bg.blit(lbl, (size // 2 + lw + 4,py - lbl.get_height() // 2))
            bg.blit(lbl, (size // 2 - lw - lbl.get_width() - 4, py - lbl.get_height() // 2))

    rot  = pygame.transform.rotate(bg, -roll_deg)
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
    pygame.draw.line(surf, C_CROSS, (cx + r // 6, cy), (cx + hw,cy), 3)
    pygame.draw.circle(surf, C_CROSS, (cx, cy), 4)

def draw_header(surf, t):
    """Barra superiore: satelliti, modalità, yaw."""

    text(surf, t["mode"],(W // 2, 12),   F_HEAD, C_WHITE,anchor="midtop")
    if t["in_flight"]:
        text(surf, f"IN FLIGHT",(W - 28, 12), F_HEAD, C_ACCENT, anchor="topright")
    elif t["alarm_failsafe"]:
        text(surf, f"FAILSAFE!",(W - 28, 12), F_HEAD, C_RED,anchor="topright")
    elif t["alarm_crash"]:
        text(surf, f"CRASH!", (W - 28, 12),F_HEAD, C_RED,anchor="topright")
    else:
        text(surf,f"ON GROUND",(W - 28, 12), F_HEAD, C_DIM,anchor="topright")
    mx, my = pygame.mouse.get_pos()
    text(surf, f"({mx},{my})", (28, 48), F_SMALL, C_DIM)


def draw_power_panel(surf, t):
    draw_panel(surf, POWER_PANEL, "POWER")


    thr = pygame.Rect(POWER_PANEL.x + 20, POWER_PANEL.y + 40, 100, 196)
    draw_vbar(surf, thr, t["throttle"], 0.0, 1.0, C_GREEN if t["throttle"] < 0.8 else C_YELLOW)
    text(surf, "THR", (thr.centerx, thr.bottom + 14), F_SMALL, C_DIM, anchor="center")

    title= "RC CONTROLLER" if t["mode"] == "MANUALE" else "PID CONTROLLER"
    kx = POWER_PANEL.x + 175
    ky = POWER_PANEL.y + 40
    text(surf, title, (kx, ky), F_TITLE, C_ACCENT)
    ky += 22
    if t["mode"] == "MANUALE":
        rows = [("GAS",   f"{t['rc_gas']:.2f}"),
                ("PITCH", f"{t['rc_pitch']:.1f}"),
                ("ROLL",  f"{t['rc_roll']:.1f}"),
                ("YAW",   "—")]
    else:
        rows = [("GAS",   f"{t['pid_gas']:.2f}"),
                ("PITCH", f"{t['pid_pitch']:.1f}"),
                ("ROLL",  f"{t['pid_roll']:.1f}"),
                ("YAW",   "—")]
    for label, val in rows:
        draw_kv(surf, label, val, kx, ky)
        ky += 38


def draw_temp_panel(surf, t):
    draw_panel(surf, TEMP_PANEL, "TEMPERATURE")
    kx = TEMP_PANEL.x + 20
    ky = TEMP_PANEL.y + 42
    draw_kv(surf, "TEENSY", f"{t['t_teensy']:.1f} °C", kx, ky, col_v=C_WHITE if t["t_teensy"] < 60 else C_YELLOW if t["t_teensy"] < 65 else C_RED)
    draw_kv(surf, "MOTORE", f"{t['t_motor']:.1f} °C",  kx, ky + 34, col_v=C_WHITE if t["t_motor"] < 80 else C_YELLOW if t["t_motor"] < 85 else C_RED)


def draw_speed_panel(surf, t):
    draw_panel(surf, SPEED_PANEL, "SPEED")
    entries = [("PITOT", t["spd_pitot"], "km/h"),("GPS",   t["spd_gps"],   "km/h"),("EST",   t["spd_ms"],    "m/s")]
    bw, bh, gap = 88, 120, 10
    bx = SPEED_PANEL.x + 18
    by = SPEED_PANEL.y + 50
    for label, val, unit in entries:
        r = pygame.Rect(bx, by, bw, bh)
        draw_vbar(surf, r, val, 0, 100, C_GREEN)
        text(surf, label, (r.centerx, r.top - 14), F_SMALL, C_DIM,  anchor="center")
        text(surf, unit,  (r.centerx, r.bottom + 6), F_SMALL, C_DIM, anchor="center")
        bx += bw + gap

 
def draw_servi_panel(surf, t):
    draw_panel(surf, SERVI_PANEL, "SERVI")
    servos = [(t["deg_isx"], "Int SX", t["v_isx"]),(t["deg_idx"], "Int DX", t["v_idx"]),(t["deg_esx"], "Est SX", t["v_esx"]),(t["deg_edx"], "Est DX", t["v_edx"])]

    def servo_ok(v): return 4.5 < v < 6.0

    for i, (deg, lbl, v) in enumerate(servos):
        draw_servo_leds(surf, SERVI_X + i * SERVI_GAP, SERVI_Y,deg, lbl, v, error=not servo_ok(v))

def draw_vserv_panel(surf, t):
    draw_panel(surf, VSERV_PANEL, "TENSIONE SERVI")
    labels = ["Int SX", "Int DX", "Est SX", "Est DX"]
    vals   = [t["v_isx"], t["v_idx"], t["v_esx"], t["v_edx"]]
    bw, bh, gap = 72, 120, 16
    bx = VSERV_PANEL.x + 20
    by = VSERV_PANEL.y + 55
    for label, val in zip(labels, vals):
        ok  = 4.5 < val < 6.0
        col = C_GREEN if ok else C_RED
        r   = pygame.Rect(bx, by, bw, bh)
        draw_vbar(surf, r, val, 4.0, 6.5, col)
        text(surf, label, (r.centerx, r.top - 14), F_SMALL, C_DIM, anchor="center")
        bx += bw + gap

def draw_pfd_center(surf, t):
    draw_horizon(surf, PFD_CX, PFD_CY, PFD_R, t["pitch"], t["roll"])
    draw_hud_box(surf, SPD_BOX, t["spd_ms"],   "SPEED", "m/s")
    draw_hud_box(surf, ALT_BOX, t["altitude"], "ALT",   "m")
    text(surf, f"TARGET  {t['dist_target']:.0f} m",
         (PFD_CX, PFD_CY + PFD_R + 26), F_LABEL, C_DIM, anchor="midtop")
    
def navigation_info_pannel(surf,t):
    draw_panel(surf, NAV_PANEL, "NAV INFO")
    kx = NAV_PANEL.x + 20
    ky = NAV_PANEL.y + 40
    draw_kv(surf, "DIST TGT", f"{t['dist_target']:.1f} m", kx, ky, col_v=C_WHITE if not t["dist_target"] < 150 else C_YELLOW)
    draw_kv(surf, "HDG TGT",  f"{t['hdg_target']:.1f}°", kx, ky + 34)
    draw_kv(surf, "ROLL TGT", f"{t['roll_target']:.1f}°", kx, ky + 68)
    draw_kv(surf, "SATELLITES", f"{t['satellites']:.1f}", kx+215 , ky, col_v=C_WHITE if t["satellites"] >= 5 else C_RED)
    draw_kv(surf, "LAT", f"{t['lat']:.6f}", kx+215, ky + 34)
    draw_kv(surf, "LON", f"{t['lon']:.6f}", kx+215, ky + 68)

def battery_pannel(surf, t):
    draw_panel(surf, BATTERY_PANNEL, "BATTERY")
    
    C_VUOTO = (18, 26, 42)
    kx = BATTERY_PANNEL.x + 40
    ky = BATTERY_PANNEL.y + 40
    bar_w, bar_h = 60, 100
    gap = 180  

    # 1. BATTERIA TEENSY (Sinistra)
    V_t = pygame.Rect(kx, ky, bar_w, bar_h)
    draw_vbar(surf, V_t, t["v_teensy"], 4.0, 6.5, C_GREEN if not t["alarm_batt_teensy"] else C_RED)
    text(surf, "V SYS", (V_t.centerx, V_t.bottom + 14), F_SMALL, C_DIM, anchor="center")

    # 2. BATTERIA MOTORE (Destra)
    V_m = pygame.Rect(kx + bar_w + gap, ky, bar_w, bar_h)
    draw_vbar(surf, V_m, t["v_motor"], 12.0, 16.8, C_GREEN if not t["alarm_batt_motor"] else C_RED)
    text(surf, "V MOT", (V_m.centerx, V_m.bottom + 14), F_SMALL, C_DIM, anchor="center")

    # 3. TUBO E VALVOLA RELÈ
    rele_on = t["relay"]
    pipe_h = 14
    pipe_y = ky + bar_h // 2 - pipe_h // 2
    pipe_x1 = V_t.right     
    
    # Disegno sfondo del tubo
    pygame.draw.rect(surf, C_BORDER, (pipe_x1, pipe_y, gap, pipe_h)) 
    pygame.draw.rect(surf, C_VUOTO, (pipe_x1, pipe_y+2, gap, pipe_h-4)) 
    if rele_on:
        pygame.draw.rect(surf, C_GREEN, (pipe_x1, pipe_y+2, gap, pipe_h-4))

    # Dimensioni e posizione corpo valvola 
    vw, vh = 32, 32
    vx = pipe_x1 + gap // 2 - vw // 2
    vy = ky + bar_h // 2 - vh // 2
    
    # Colore della valvola in base allo stato
    valve_color = C_GREEN if rele_on else C_RED
    
    # Corpo valvola 
    pygame.draw.rect(surf, (14, 20, 32), (vx, vy, vw, vh), border_radius=4)
    pygame.draw.rect(surf, valve_color, (vx, vy, vw, vh), 2, border_radius=4)

    # Disegno dell'otturatore interno
    if rele_on:
        pygame.draw.rect(surf, C_GREEN, (vx + 4, vy + vh // 2 - 4, vw - 8, 8))
        lbl_txt = "FAILOVER ON"
    else:
        pygame.draw.rect(surf, C_RED, (vx + vw // 2 - 4, vy + 4, 8, vh - 8))
        lbl_txt = "ISOLATI"

    # Etichetta sopra la valvola
    text(surf, lbl_txt, (vx + vw // 2, vy - 12), F_SMALL, valve_color, anchor="center")

def posizione(surf,t):
    draw_panel(surf, ORIENTATION_PANEL, "ORIENTAZIONE")
    kx = ORIENTATION_PANEL.x + 20
    ky = ORIENTATION_PANEL.y + 40
    draw_kv(surf, "ROLL", f"{t['roll']:.1f}°", kx, ky , col_v=C_WHITE if abs(t["roll"]) < 30 else C_YELLOW if abs(t["roll"]) < 35 else C_RED)
    draw_kv(surf, "PITCH",  f"{t['pitch']:.1f}°", kx, ky + 34, col_v=C_WHITE if abs(t["pitch"]) < 15 else C_YELLOW if abs(t["pitch"]) < 20 else C_RED)
    draw_kv(surf, "YAW", f"{t['yaw']:.1f}°", kx+215, ky )
    draw_kv(surf, "SPEED km/h", f"{t['spd_fused']:.1f} km/h", kx+215, ky + 34, col_v=C_WHITE if t["spd_fused"] < 70 else C_YELLOW if t["spd_fused"] < 90 else C_RED)
 
def main():

    thread_seriale = threading.Thread(target=read_from_serial, daemon=True)
    thread_seriale.start()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        screen.fill(C_BG)
        draw_header(screen, T)
        draw_power_panel(screen, T)
        draw_temp_panel(screen, T)
        draw_speed_panel(screen, T)
        draw_servi_panel(screen, T)
        draw_vserv_panel(screen, T)
        draw_pfd_center(screen, T)
        navigation_info_pannel(screen, T)
        posizione(screen, T)
        battery_pannel(screen, T)

        pygame.display.flip()
        clock.tick(30)

if __name__ == "__main__":
    main()