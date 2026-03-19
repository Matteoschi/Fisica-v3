import pygame
import sys
import math
import serial

# ==========================================
# 1. INIZIALIZZAZIONE E COSTANTI
# ==========================================
pygame.init()

LARGHEZZA = 1280
ALTEZZA   = 720
schermo   = pygame.display.set_mode((LARGHEZZA, ALTEZZA))
pygame.display.set_caption("GCS - PFD: Test Sensori Completo")

NERO         = (20,  20,  20)
BIANCO       = (255, 255, 255)
VERDE        = (46,  204, 113)
ROSSO        = (231, 76,  60)
GIALLO_DEBUG = (241, 196, 15)
BLU_CIELO    = (52,  152, 219)
GRIGIO_SCURO = (40,  40,  40)

font_testo   = pygame.font.SysFont("consolas", 20, bold=True)
font_piccolo = pygame.font.SysFont("consolas", 16)
font_debug   = pygame.font.SysFont("consolas", 14, bold=True)

# ==========================================
# 2. VARIABILI GLOBALI DI VOLO
# ==========================================
satelliti        = 0
voltaggio_motore = 0.0
modalità         = "ATTESA DATI"

gass_motore_pid   = 0.0
comando_pitch_pid = 0.0
comando_roll_pid  = 0.0
comando_yaw_pid   = 0.0

comando_gass_rc  = 0.0
comando_pitch_rc = 0.0
comando_roll_rc  = 0.0
comando_yaw_rc   = 0.0

velocità_pitot = 0.0
velocità_gps   = 0.0
velocita_ms    = 0.0
quota_m        = 0.0

pitch_simulato = 0.0
roll_simulato  = 0.0
yaw_simulato   = 0.0   

distanza_target = 0.0
rotta_target    = 0.0
target_roll     = 0.0

gradi_intSX = 90.0
gradi_intDX = 90.0
gradi_estSX = 90.0
gradi_estDX = 90.0

volt_S_int_sx = 0.0
volt_S_int_dx = 0.0
volt_S_est_sx = 0.0
volt_S_est_dx = 0.0

Temp_motore = 0.0
Temp_teensy = 0.0

clock = pygame.time.Clock()

def costringi(valore, minimo, massimo):
    return max(minimo, min(valore, massimo))
PORTA_COM = 'COM3'  
BAUD_RATE = 9600    

try:
    ricevente_lora = serial.Serial(PORTA_COM, BAUD_RATE, timeout=0.1)
    print(f"Connesso con successo al modulo LoRa sulla porta {PORTA_COM}")
    
except Exception as e:
    print(f"ATTENZIONE: Impossibile aprire la porta {PORTA_COM}.")
    ricevente_lora = None
# ==========================================
# 3. PARSER CSV
# ==========================================
def elabora_telemetria(riga_csv):
    """
    Formato CSV (32 campi, indice 0 = '$'):
      0:$  1:modo  2:spare  3:Vmot
      4:VsIntSX  5:VsIntDX  6:VsEstSX  7:VsEstDX
      8:spare  9:pitch  10:roll  11:yaw   <-- yaw ora letto!
      12:quota  13:vPitot  14:vGPS  15:vStimata_kmh
      16:distTarget  17:rottaTarget  18:targetRoll
      19:rcPitch  20:rcRoll  21:rcGas
      22:pidPitch  23:pidRoll  24:pidGas_raw(25-130)
      25:grIntSX  26:grIntDX  27:grEstSX  28:grEstDX
      29:Tmotore  30:Tteensy  31:satelliti
    """
    global modalità, satelliti, voltaggio_motore
    global gass_motore_pid, comando_pitch_pid, comando_roll_pid, comando_yaw_pid
    global comando_gass_rc, comando_pitch_rc, comando_roll_rc, comando_yaw_rc
    global velocità_pitot, velocità_gps, velocita_ms, quota_m
    global pitch_simulato, roll_simulato, yaw_simulato
    global distanza_target, rotta_target, target_roll
    global gradi_intSX, gradi_intDX, gradi_estSX, gradi_estDX
    global volt_S_int_sx, volt_S_int_dx, volt_S_est_sx, volt_S_est_dx
    global Temp_motore, Temp_teensy

    try:
        dati = riga_csv.strip().split(',')
        if len(dati) < 32 or dati[0] != '$':
            print(f"[WARN] Pacchetto non valido (len={len(dati)})")
            return

        mod_int  = int(dati[1])
        modalità = {1: "MANUALE", 2: "AUTO PID", 3: "FAILSAFE!"}.get(mod_int, "SCONOSCIUTA")

        voltaggio_motore = float(dati[3])
        volt_S_int_sx    = float(dati[4])
        volt_S_int_dx    = float(dati[5])
        volt_S_est_sx    = float(dati[6])
        volt_S_est_dx    = float(dati[7])

        pitch_simulato = float(dati[9])
        roll_simulato  = float(dati[10])
        yaw_simulato   = float(dati[11])  

        quota_m        = float(dati[12])
        velocità_pitot = float(dati[13])
        velocità_gps   = float(dati[14])
        velocita_ms    = float(dati[15]) / 3.6

        distanza_target = float(dati[16])
        rotta_target    = float(dati[17])
        target_roll     = float(dati[18])

        comando_pitch_rc = float(dati[19])
        comando_roll_rc  = float(dati[20])
        comando_gass_rc  = float(dati[21])

        comando_pitch_pid = float(dati[22])
        comando_roll_pid  = float(dati[23])
        gass_motore_pid   = costringi((float(dati[24]) - 25.0) / 105.0, 0.0, 1.0)

        gradi_intSX = float(dati[25])
        gradi_intDX = float(dati[26])
        gradi_estSX = float(dati[27])
        gradi_estDX = float(dati[28])

        Temp_motore = float(dati[29])
        Temp_teensy = float(dati[30])
        satelliti   = int(dati[31])

    except Exception as e:
        print(f"[ERR] Pacchetto corrotto saltato: {e}")

# ==========================================
# 4. FUNZIONI DI DISEGNO
# ==========================================
def disegna_batteria(superficie, x, y, larghezza, altezza,valore_attuale, valore_max, valore_min,etichetta="", colore=VERDE):

    if valore_max == valore_min:
        return
    
    percentuale  = costringi((valore_attuale - valore_min) / (valore_max - valore_min), 0.0, 1.0)
    colore_barra = ROSSO if percentuale < 0.2 else colore
    spessore     = 3

    pygame.draw.rect(superficie, BIANCO, (x, y, larghezza, altezza), spessore)
    margine   = spessore + 1
    h_interna = int((altezza - margine * 2) * percentuale)
    y_interna = y + altezza - margine - h_interna

    if h_interna > 0:
        pygame.draw.rect(superficie, colore_barra,
                         (x + margine, y_interna, larghezza - margine * 2, h_interna))

    surf = font_piccolo.render(f"{valore_attuale:.2f}", True, BIANCO)
    superficie.blit(surf, surf.get_rect(center=(x + larghezza // 2, y + altezza // 2)))

    if etichetta:
        surf_e = font_piccolo.render(etichetta, True, BIANCO)
        superficie.blit(surf_e, surf_e.get_rect(center=(x + larghezza // 2, y + altezza + 15)))


def disegna_testo(superficie, testo, valore, x, y, font, colore=BIANCO):
    t = font.render(testo,       True, colore)
    v = font.render(str(valore), True, colore)
    superficie.blit(t, (x, y))
    superficie.blit(v, (x + t.get_width(), y))


def disegna_pid(superficie, x, y, larghezza, altezza, gass, pitch, roll, yaw):
    pygame.draw.rect(superficie, BIANCO, (x, y, larghezza, altezza), 2)

    titolo = "RC Controller" if modalità == "MANUALE" else "PID Controller"
    surf_t = font_piccolo.render(titolo, True, BIANCO)
    superficie.blit(surf_t, surf_t.get_rect(center=(x + larghezza // 2, y + 15)))

    disegna_testo(superficie, "Cmd GAS:  ", f"{gass:.2f}",  x + 10, y + 40,  font_piccolo)
    disegna_testo(superficie, "Cmd pitch:", f"{pitch:.1f}", x + 10, y + 65,  font_piccolo)
    disegna_testo(superficie, "Cmd roll: ", f"{roll:.1f}",  x + 10, y + 90,  font_piccolo)
    disegna_testo(superficie, "Cmd yaw:  ", f"{yaw:.1f}",   x + 10, y + 115, font_piccolo)


def disegna_barra_servi(superficie, x, y, valore_gradi, etichetta,larghezza, altezza_led, spazio_led, num_led,stato_errore, voltaggio_servo):
    delta  = valore_gradi - 90
    leds   = [False] * 5
    leds[2] = True
    if delta >  10: leds[3] = True
    if delta >  25: leds[4] = True
    if delta < -10: leds[1] = True
    if delta < -25: leds[0] = True

    surf_e = font_piccolo.render(etichetta, True, BIANCO)
    superficie.blit(surf_e, surf_e.get_rect(center=(x + larghezza // 2, y - 20)))

    for i in range(num_led):
        y_rect = y + (num_led - 1 - i) * (altezza_led + spazio_led)
        if stato_errore:
            colore_led = ROSSO
        elif leds[i]:
            colore_led = VERDE if i == 2 else BLU_CIELO
        else:
            colore_led = GRIGIO_SCURO
        pygame.draw.rect(superficie, colore_led, (x, y_rect, larghezza, altezza_led))
        pygame.draw.rect(superficie, NERO,(x, y_rect, larghezza, altezza_led), 1)

    y_info   = y + num_led * (altezza_led + spazio_led) + 5
    surf_val = font_piccolo.render("ERR!" if stato_errore else f"{int(valore_gradi)}°",True, ROSSO if stato_errore else BIANCO)
    superficie.blit(surf_val, surf_val.get_rect(center=(x + larghezza // 2, y_info)))
    surf_v = font_piccolo.render(f"{voltaggio_servo:.1f}V", True, BIANCO)
    superficie.blit(surf_v, surf_v.get_rect(center=(x + larghezza // 2, y_info + 20)))


def disegna_orizzonte(superficie, cx, cy, raggio, pitch_deg, roll_deg):
    diam   = raggio * 2
    grande = int(raggio * 4.2)
    px_deg = raggio / 25.0
    split  = grande // 2 - int(pitch_deg * px_deg)

    bg = pygame.Surface((grande, grande))
    bg.fill(BLU_CIELO)
    if split < grande:
        pygame.draw.rect(bg, (160, 110, 40), (0, max(0, split), grande, grande))
    if split <= 0:
        bg.fill((160, 110, 40))
    if 0 <= split <= grande:
        pygame.draw.line(bg, BIANCO, (0, split), (grande, split), 3)

    for p in [-20, -10, 10, 20]:
        py = split - int(p * px_deg)
        if 4 < py < grande - 4:
            lw  = raggio // 4
            pygame.draw.line(bg, BIANCO, (grande // 2 - lw, py), (grande // 2 + lw, py), 1)
            lbl = font_piccolo.render(f"{abs(p)}", True, BIANCO)
            bg.blit(lbl, (grande // 2 + lw + 4,                  py - lbl.get_height() // 2))
            bg.blit(lbl, (grande // 2 - lw - lbl.get_width() - 4, py - lbl.get_height() // 2))

    rotated = pygame.transform.rotate(bg, -roll_deg)
    rx, ry  = rotated.get_size()
    content = pygame.Surface((diam, diam))
    content.blit(rotated, (0, 0), (rx // 2 - raggio, ry // 2 - raggio, diam, diam))

    mask = pygame.Surface((diam, diam), pygame.SRCALPHA)
    pygame.draw.circle(mask, (255, 255, 255, 255), (raggio, raggio), raggio)
    ca = content.convert_alpha()
    ca.blit(mask, (0, 0), special_flags=pygame.BLEND_RGBA_MULT)
    superficie.blit(ca, (cx - raggio, cy - raggio))

    pygame.draw.circle(superficie, (200, 200, 200), (cx, cy), raggio, 3)
    hw = raggio // 2
    pygame.draw.line(superficie, GIALLO_DEBUG, (cx - hw,          cy), (cx - raggio // 6, cy), 4)
    pygame.draw.line(superficie, GIALLO_DEBUG, (cx + raggio // 6, cy), (cx + hw,          cy), 4)
    pygame.draw.circle(superficie, GIALLO_DEBUG, (cx, cy), 4)


def disegna_valori_hud(superficie, x, y, larghezza, altezza, valore, etichetta="", unita=""):
    pygame.draw.rect(superficie, (45, 45, 45),(x, y, larghezza, altezza))
    pygame.draw.rect(superficie, (255, 220,  0), (x, y, larghezza, altezza), 2)

    surf_val = font_piccolo.render(f"{valore:.1f}", True, (255, 220, 0))
    superficie.blit(surf_val, surf_val.get_rect(center=(x + larghezza // 2, y + altezza // 2)))

    if etichetta:
        surf_lbl = font_piccolo.render(etichetta, True, BIANCO)
        superficie.blit(surf_lbl, surf_lbl.get_rect(center=(x + larghezza // 2, y - 12)))

    if unita:
        surf_unit = font_piccolo.render(unita, True, BIANCO)
        superficie.blit(surf_unit, surf_unit.get_rect(center=(x + larghezza // 2, y + altezza + 12)))


# ==========================================
# 5. LAYOUT COSTANTI
# ==========================================
PFD_CX    = LARGHEZZA // 2
PFD_CY    = 241
RAGGIO    = 140
BOX_W     = 60
BOX_H     = 30
BOX_Y     = PFD_CY - BOX_H // 2
GAP       = 15
SPD_X     = PFD_CX - RAGGIO - GAP - BOX_W
ALT_X     = PFD_CX + RAGGIO + GAP
SERVI_X   = 870
SERVI_Y   = 150
SERVI_GAP = 85

# ==========================================
# 6. CARICA TELEMETRIAA
# ==========================================
if ricevente_lora is not None and ricevente_lora.in_waiting > 0:
        try:
            #  USB
            dati_grezzi = ricevente_lora.readline()
            
            # La decodifica in testo normale (utf-8) 
            riga_seriale = dati_grezzi.decode('utf-8').strip()
            
            if riga_seriale.startswith('$'):
                elabora_telemetria(riga_seriale)
                
        except Exception as e:
            pass

# ==========================================
# 7. CICLO PRINCIPALE
# ==========================================
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    schermo.fill(NERO)

    # ─── HEADER ───────────────────────────────────────────────────────────
    col_sat = ROSSO if satelliti < 5 else BIANCO
    disegna_testo(schermo, "Satelliti: ", satelliti,35,10, font_testo, col_sat)
    disegna_testo(schermo, "Modalità:  ", modalità,LARGHEZZA // 2 - 94, 20, font_testo, BIANCO)
    disegna_testo(schermo, "YAW: ", f"{yaw_simulato:.1f}°",
                  LARGHEZZA // 2 + 160, 20, font_testo, BIANCO)  

    # ─── PANNELLO SINISTRO ─────────────────────────────────────────────────
    disegna_batteria(schermo, x=35,  y=150, larghezza=75, altezza=200,
                     valore_attuale=voltaggio_motore,
                     valore_max=16.8, valore_min=13.2,
                     etichetta="MAIN 4S", colore=VERDE)

    disegna_batteria(schermo, x=360, y=150, larghezza=50, altezza=200,
                     valore_attuale=gass_motore_pid,
                     valore_max=1.0, valore_min=0.0,
                     etichetta="Throttle", colore=GIALLO_DEBUG)

    if modalità != "MANUALE":
        disegna_pid(schermo, x=141, y=150, larghezza=200, altezza=170,
                    gass=gass_motore_pid,
                    pitch=comando_pitch_pid,
                    roll=comando_roll_pid,
                    yaw=comando_yaw_pid)
    else:
        disegna_pid(schermo, x=141, y=150, larghezza=200, altezza=170,
                    gass=comando_gass_rc,
                    pitch=comando_pitch_rc,
                    roll=comando_roll_rc,
                    yaw=comando_yaw_rc)

    disegna_testo(schermo, "Temp Motore: ", f"{Temp_motore:.1f} °C",
                  141, 340, font_piccolo, ROSSO if Temp_motore > 80 else BIANCO)
    disegna_testo(schermo, "Temp Teensy: ", f"{Temp_teensy:.1f} °C",
                  141, 365, font_piccolo, ROSSO if Temp_teensy > 60 else BIANCO)

    # ─── PANNELLO DESTRO (servi) ───────────────────────────────────────────
    def servo_ok(v): 
        return 4.5 < v < 6.0

    disegna_barra_servi(schermo, SERVI_X,SERVI_Y,gradi_intSX, "IntSX", 40, 20, 5, 5,not servo_ok(volt_S_int_sx), volt_S_int_sx)
    disegna_barra_servi(schermo, SERVI_X + SERVI_GAP,    SERVI_Y,gradi_intDX, "IntDX", 40, 20, 5, 5,not servo_ok(volt_S_int_dx), volt_S_int_dx)
    disegna_barra_servi(schermo, SERVI_X + SERVI_GAP * 2, SERVI_Y,gradi_estSX, "EstSX", 40, 20, 5, 5,not servo_ok(volt_S_est_sx), volt_S_est_sx)
    disegna_barra_servi(schermo, SERVI_X + SERVI_GAP * 3, SERVI_Y,gradi_estDX, "EstDX", 40, 20, 5, 5,not servo_ok(volt_S_est_dx), volt_S_est_dx)

    disegna_testo(schermo, "Velocità Pitot: ", f"{velocità_pitot:.1f} km/h", SERVI_X, 370, font_piccolo, BIANCO)
    disegna_testo(schermo, "Velocità GPS  : ", f"{velocità_gps:.1f} km/h", SERVI_X, 390, font_piccolo, BIANCO)
    disegna_testo(schermo, "Rotta Target  : ", f"{rotta_target:.1f}°",SERVI_X, 330, font_piccolo, BIANCO)
    disegna_testo(schermo, "Target Roll   : ", f"{target_roll:.1f}°",SERVI_X, 350, font_piccolo, BIANCO)

    # ─── PANNELLO CENTRALE (PFD) ───────────────────────────────────────────
    disegna_valori_hud(schermo, SPD_X, BOX_Y, BOX_W, BOX_H,velocita_ms, etichetta="SPEED", unita="m/s")
    disegna_orizzonte(schermo, PFD_CX, PFD_CY, RAGGIO, pitch_simulato, roll_simulato)
    disegna_valori_hud(schermo, ALT_X, BOX_Y, BOX_W, BOX_H,quota_m, etichetta="ALT", unita="m")

    disegna_testo(schermo, "Distanza Target: ", f"{distanza_target:.0f} m",PFD_CX - 110, 400, font_piccolo, BIANCO)


    pygame.display.flip()
    clock.tick(30)