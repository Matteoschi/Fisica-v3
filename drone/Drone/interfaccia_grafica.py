import pygame
import sys
import serial
import threading
import os
from collections import deque

SERIAL_PORT = "COM3"
BAUD_RATE = 9600

pygame.init()
pygame.mixer.init()

W, H = 1280, 900
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("GCS – Primary Flight Display")
clock = pygame.time.Clock()

CARTELLA_AUDIO = r"C:\Users\Utente\Documents\coding\Fisica-v3-main\drone\Drone\audio_gpws"

C_BG     = ( 8,  11,  18)   # sfondo principale
C_PANEL  = (14,  18,  28)   # sfondo pannello
C_BORDER = (36,  50,  76)   # bordi
C_ACCENT = ( 0, 188, 255)   # ciano – titoli e accenti
C_TEXT   = (205, 218, 240)  # testo standard
C_DIM    = ( 82,  98, 126)  # testo secondario / etichette
C_WHITE  = (242, 246, 255)  # testo luminoso / valori
C_GREEN  = ( 40, 210,  88)  # stato OK
C_YELLOW = (255, 200,   0)  # throttle / HUD
C_ORANGE = (255, 140,   0)  # diagnostica: livello WARNING
C_RED    = (218,  48,  48)  # allarme / errore
C_SKY    = ( 28,  96, 178)  # cielo orizzonte
C_GROUND = (124,  80,  30)  # terra orizzonte
C_CROSS  = (255, 210,   0)  # crosshair orizzonte
C_GRAPH_BARO  = (  0, 188, 255)  # linea grafico: baro
C_GRAPH_LIDAR = ( 40, 210,  88)  # linea grafico: lidar
C_GRAPH_FUSA  = (255, 200,   0)  # linea grafico: quota fusa

F_HEAD  = pygame.font.SysFont("consolas", 21, bold=True)  # header principale
F_TITLE = pygame.font.SysFont("consolas", 13, bold=True)  # titoli pannelli
F_VAL   = pygame.font.SysFont("consolas", 15, bold=True)  # valori numerici
F_LABEL = pygame.font.SysFont("consolas", 13)             # etichette
F_SMALL = pygame.font.SysFont("consolas", 12)             # testo piccolo


# ── Cooldown audio (ms) ──────────────────────────────────────
CD_PULL_UP          = 7500
CD_TERRAIN_PULL_UP  = 5500
CD_TERRAIN          = 5500
CD_SINK_RATE        = 6500
CD_DONT_SINK        = 7500
CD_BANK_ANGLE       = 7500
CD_STALL            = 5720
CD_AIRSPEED_LOW     = 9500
CD_FLIGHT_SLOW      = 6500
CD_OVERSPEED        = 11500
CD_ERRORE           = 8500

# ── Roll ──────────────────────────────────────────────────────
ROLL_CRIT = 35     # ROSSO  in HUD + "bank_angle.wav"
ROLL_WARN = ROLL_CRIT - 5  # GIALLO in HUD
# ── Pitch ─────────────────────────────────────────────────────
PITCH_CRIT = 20    # ROSSO  in HUD + "pitch.wav"
PITCH_WARN = PITCH_CRIT - 5  # GIALLO
# ── Velocità (km/h) ───────────────────────────────────────────
SPD_STALL   = 22   # < soglia → "stall.wav" + ROSSO
SPD_AIR_LOW = 35   # < soglia → "air_speed_low.wav" + GIALLO
SPD_SLOW    = 45   # < soglia → "fligh slow.wav"

SPD_WARN    = 70   # >= soglia → GIALLO in HUD
SPD_CRIT    = 90   # >= soglia → ROSSO  in HUD + "overspeed.wav"

# ── Discesa (m/s, negativo = scende) ─────────────────────────
VDISCESA_PULL_UP    = -1.0   # + alt < ALT_PULL_UP    → pull_up
VDISCESA_TERRAIN_PU = -0.5   # + alt < ALT_TERRAIN_PU → terrain pull up
VDISCESA_TERRAIN    = -0.3   # + alt < ALT_TERRAIN    → terrain
VDISCESA_DONT_SINK  = -0.3   # + alt < ALT_DONT_SINK  → dont sink
VDISCESA_SINK_RATE  = -4.0   # qualsiasi quota        → sink rate
VDISCESA_CALLOUT    = -0.2   # attiva callout quota

# ── Quote (m) ─────────────────────────────────────────────────
ALT_PULL_UP       =  5
ALT_TERRAIN_PU    = 10
ALT_TERRAIN       = 20
ALT_DONT_SINK     = 15
ALT_RESET_CALLOUT = 50   # sopra questa quota → reset callout

# ── Finestre callout quota (±2 m) ────────────────────────────
CALLOUT_40_LO, CALLOUT_40_HI = 38, 42
CALLOUT_30_LO, CALLOUT_30_HI = 28, 32
CALLOUT_20_LO, CALLOUT_20_HI = 18, 22
CALLOUT_10_LO, CALLOUT_10_HI =  8, 12

# ── Temperatura (°C) ──────────────────────────────────────────

T_TEENSY_CRIT = 65   # >= soglia → ROSSO + "errore.wav"
T_TEENSY_WARN = T_TEENSY_CRIT - 5  # >= soglia → GIALLO

T_MOTOR_CRIT = 85    # >= soglia → ROSSO + "errore.wav"
T_MOTOR_WARN = T_MOTOR_CRIT - 5  # >= soglia → GIALLO

# ── Tensione servo (V) ────────────────────────────────────────
SERVO_V_MIN = 4.5    # fuori range → servo ERROR
SERVO_V_MAX = 6.0

# ── Scale barre batteria (V) ──────────────────────────────────
VBAR_TEENSY_MIN = 4.0
VBAR_TEENSY_MAX = 6.5

VBAR_MOTOR_MIN = 12.0
VBAR_MOTOR_MAX = 16.8

# ── Distanza target (m) ───────────────────────────────────────
DIST_TGT_WARN = 150   # < soglia → GIALLO

# ── Errore rotta (°) ──────────────────────────────────────────
HDG_ERR_WARN = 30     # >= soglia (assoluto) → GIALLO
HDG_ERR_CRIT = 90     # >= soglia (assoluto) → ROSSO

# ── Satelliti GPS ─────────────────────────────────────────────
SAT_MIN = 5           # < soglia → ROSSO

# ── Throttle ──────────────────────────────────────────────────
THR_WARN = 0.8        # >= soglia → barra GIALLA
SOGLIA_G_SCHIANTO = 50          # m/s² (accelerazione IMU oltre la quale viene rilevato uno schianto)
GAS_MINIMO  = 1000
GAS_MASSIMO = 2000

# ── Orizzonte artificiale ────────────────────────────────────
PFD_CX, PFD_CY, PFD_R = W // 2, 245, 142

# ── Box velocità e quota ai lati dell'orizzonte ──────────────
BOX_W, BOX_H, BOX_GAP = 66, 34, 18
SPD_BOX = pygame.Rect(PFD_CX - PFD_R - BOX_GAP - BOX_W, PFD_CY - BOX_H // 2, BOX_W, BOX_H)
ALT_BOX = pygame.Rect(PFD_CX + PFD_R + BOX_GAP,          PFD_CY - BOX_H // 2, BOX_W, BOX_H)

# ── Pannelli laterali ─────────────────────────────────────────
POWER_PANEL       = pygame.Rect( 24, 108, 378, 272)
TEMP_PANEL        = pygame.Rect( 24, 390, 378, 122)
SPEED_PANEL       = pygame.Rect( 24, 522, 378, 182)
SERVI_PANEL       = pygame.Rect(878, 108, 378, 200)
VSERV_PANEL       = pygame.Rect(878, 320, 378, 193)
NAV_PANEL         = pygame.Rect(428, 522, 424, 140)
ORIENTATION_PANEL = pygame.Rect(428, 391, 424, 100)
BATTERY_PANNEL    = pygame.Rect(878, 522, 378, 180)

# ── Riga in basso: diagnostica sensori/navigazione + grafico quote ──
DIAG_PANEL      = pygame.Rect(24, 712, 1232, 100)


# ── Colonne servi nel pannello SERVI ──────────────────────────
SERVI_X   = SERVI_PANEL.x + 28
SERVI_Y   = SERVI_PANEL.y + 52
SERVI_GAP = 88


_serial_lock  = threading.Lock()
_ser_instance = None


T = {
    "mode":       "ATTESA",
    "serial_ok":  False,
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
    "opt_vx":          0.0,
    "opt_vy":          0.0,
    "sensor_optflow_ok": False,
    "sensor_lidar_ok":   False,
    "sensor_packet_lost": False,
    "heading_error":   0.0,
    "thermal_limit":   GAS_MASSIMO,
    "alt_lidar_raw":   0.0,
    "alt_baro_raw":    0.0,
    # ── Campi v2 firmware ─────────────────────────────────────
    "spd_ground":      0.0,   # Groundspeed fusa [km/h] (usata dal controllore L1)
    "fw_millis":       0,     # millis() firmware al momento dell'invio TEL1
    "tel1_packet_num": 0,     # numero progressivo pacchetto TEL1
    "tel1_lost_count": 0,     # pacchetti TEL1 mancanti stimati (da salti nel contatore)
}

# ── Diagnostica supplementare a bassa frequenza (pacchetti $2,/$3,/$4,) ──
# Non presenti nella schermata principale: consultabili nella schermata
# "TELEMETRIA ESTESA" (tasto E).
T2 = {  # $2, — correnti/potenze batterie, flusso ottico grezzo, gas pre-limite
    "curr_motor": 0.0, "curr_teensy": 0.0,
    "curr_isx": 0.0, "curr_idx": 0.0, "curr_esx": 0.0, "curr_edx": 0.0,
    "pow_motor": 0.0, "pow_teensy": 0.0,
    "batt_low_motor": False, "batt_low_teensy": False,
    "flow_dx": 0, "flow_dy": 0,
    "gas_pre_limit": 0, "thermal_limit_active": False,
}

T3 = {  # $3, — diagnostica PID completa (alt/pitch/roll/vel: errore + P + I + D)
    "alt_err": 0.0, "alt_p": 0.0, "alt_i": 0.0, "alt_d": 0.0,
    "target_pitch_auto": 0.0,
    "pitch_err": 0.0, "pitch_p": 0.0, "pitch_i": 0.0, "pitch_d": 0.0,
    "roll_err": 0.0, "roll_p": 0.0, "roll_i": 0.0, "roll_d": 0.0,
    "vel_err": 0.0, "vel_p": 0.0, "vel_i": 0.0, "vel_d": 0.0,
    "alt_target": 0.0, "vel_target": 0.0,
}

T4 = {  # $4, — GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC 4-16
    "gps_alt": -1.0, "gps_course": -1.0,
    "gps_speed_valid": False, "gps_course_valid": False, "gps_loc_valid": False,
    "baro_pressure": 0.0, "baro_tare": 0.0, "baro_ready": False,
    "pitot_raw": 0, "pitot_zero": 0.0, "pitot_diff": 0.0, "pitot_valid": False,
    "off_pitch": 0.0, "off_roll": 0.0, "off_yaw": 0.0,
    "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0, "accel_tot": 0.0,
    "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
    "imu_cal_sys": 0, "imu_cal_gyro": 0, "imu_cal_accel": 0, "imu_cal_mag": 0,
    "rc_ext": [0] * 13,  # canali radio 4-16 (canali 1-3 già in T)
}




# ================================================================
#  SISTEMA DIAGNOSTICO CENTRALIZZATO
#  (analisi soglie/failsafe derivata da main.ino — vedi commenti
#   nelle singole sezioni per la mappatura condizione → alert)
# ================================================================
from datetime import datetime as _dt

# Soglie duplicate da main.ino, usate SOLO per la spiegazione al pilota
# (il firmware resta l'unica fonte di verità per le decisioni di volo)
FW_VALORE_BATT_MOTORE_BASSA   = 13.5   # V   (VALORE_BATT_MOTORE_BASSA_V)
FW_VALORE_BATT_TEENSY_BASSA   = 4.9    # V   (VALORE_BATT_TEENSY_BASSA)
FW_T_MOTORE_THROTTLE_START    = 70.0   # °C  (T_MOTORE_THROTTLE_START)
FW_T_MOTORE_THROTTLE_END      = 90.0   # °C  (T_MOTORE_THROTTLE_END)
FW_GAS_MASSIMO                = 2000   # µs  (GAS_MASSIMO)
FW_GAS_MINIMO                 = 1200   # µs  (GAS_MINIMO)
FW_ALTEZZA_MAX_LIDAR          = 6.0    # m   (ALTEZZA_MAX_LIDAR)
FW_ALTEZZA_MAX_SENSORE_OTTICO = 4.0    # m   (ALTEZZA_MAX_SENSORE_OTTICO)
FW_SAT_MIN_FIX                = 5      # satelliti minimi per un fix affidabile (soglia GUI)
FW_PITOT_DIFF_ANOMALIA_KMH    = 25.0   # soglia derivata GUI: divergenza pitot/GPS sospetta
FW_PITOT_DURATA_ANOMALIA_S    = 3.0    # secondi di persistenza richiesti prima di segnalare
FW_MAX_AIRSPEED_X8_KMH        = 45.0   # km/h (MAX_AIRSPEED_X8_km)
FW_DISTANZA_FRENATA           = 150.0  # m   (DISTANZA_FRENATA_m)
PITCH_UP_FORZATO = 12.0   # °   (PITCH_UP_FORZATO)
PITCH_DOWN_FORZATO = -8.0  # °   (PITCH_DOWN_FORZATO)

LIVELLI = {
    "CRITICAL": {"icona": "🔴", "colore": C_RED,    "prio": 0, "tag": "CRITICAL"},
    "WARNING":  {"icona": "🟠", "colore": C_ORANGE, "prio": 1, "tag": "WARNING"},
    "INFO":     {"icona": "🟡", "colore": C_YELLOW, "prio": 2, "tag": "INFO"},
    "STATUS":   {"icona": "🔵", "colore": C_ACCENT, "prio": 3, "tag": "STATUS"},
    "OK":       {"icona": "🟢", "colore": C_GREEN,  "prio": 4, "tag": "OK"},
}

_alert_lock  = threading.Lock()
alert_attivi = {}                 # codice -> dict con i dettagli dell'alert attivo
event_log    = deque(maxlen=300)  # (ora_str, livello, testo) — mostrato in GUI e conservato per lo scroll


def _ora():
    return _dt.now().strftime("%H:%M:%S")


def add_alert(codice, livello, titolo, descrizione="", causa="", valore="", soglia="",
              azione="", durata=None, codice_errore=None, dettaglio_terminale=None):
    
    global alert_attivi
    with _alert_lock:
        nuovo = codice not in alert_attivi
        alert_attivi[codice] = {
            "livello": livello, "titolo": titolo, "descrizione": descrizione,
            "causa": causa, "valore": valore, "soglia": soglia, "azione": azione,
            "durata": durata, "codice_errore": codice_errore, "ts": _ora(),
        }
    if nuovo:
        icona = LIVELLI[livello]["icona"]
        event_log.append((_ora(), livello, f"{icona} {titolo}"))
        print(f"[{_ora()}][{LIVELLI[livello]['tag']}][{codice}] {titolo}")
        if descrizione:  print(f"    {descrizione}")
        if causa:        print(f"    Causa: {causa}")
        if valore:       print(f"    Valore attuale: {valore}")
        if soglia:       print(f"    Soglia: {soglia}")
        if azione:       print(f"    Azione consigliata: {azione}")
        if codice_errore: print(f"    Codice errore: {codice_errore}")
        if dettaglio_terminale:
            print(dettaglio_terminale)


def clear_alert(codice, messaggio_ok=None):
    """Disattiva un alert. Se era attivo, registra un evento di chiusura (🟢 risolto)."""
    global alert_attivi
    with _alert_lock:
        info = alert_attivi.pop(codice, None)
    if info is not None:
        titolo = messaggio_ok or info["titolo"]
        event_log.append((_ora(), "OK", f"🟢 {titolo} — RISOLTO"))
        print(f"[{_ora()}][OK][{codice}] {titolo} — risolto")


def alert_principale():
    """Ritorna l'alert a priorità più alta attualmente attivo, o None se nessun allarme."""
    if not alert_attivi:
        return None
    return max(alert_attivi.items(), key=lambda kv: -LIVELLI[kv[1]["livello"]]["prio"])[1]


def alert_ordinati():
    """Lista (codice, alert) ordinata per priorità (CRITICAL prima)."""
    return sorted(alert_attivi.items(), key=lambda kv: LIVELLI[kv[1]["livello"]]["prio"])


def motivi_blocco_manuale(t):
    """
    Sistema 'PERCHÉ?': condizioni che main.ino verifica prima di accettare
    il comando MODO (elaboraComando -> 'if (!failsafe && val>=1 && val<=2)'),
    più lo stato di blocco da schianto gestito separatamente in loop().
    """
    motivi = []
    if t["alarm_failsafe"]:
        motivi.append(("FAILSAFE ATTIVO", "Segnale radio assente o pacchetto SBUS non valido"))
    if t["alarm_crash"]:
        motivi.append(("SCHIANTO RILEVATO", "Gas forzato al minimo — sblocco possibile solo da radiocomando (canale 5 in basso)"))
    return motivi


# ── Stato overlay/interazione GUI (popup, pannelli scrollabili) ──────────
_rect_banner         = pygame.Rect(0, 0, 0, 0)
_overlay_diagnostica = False
_overlay_log         = False
_overlay_perche      = False
_overlay_extended    = False   # schermata "TELEMETRIA ESTESA" (TEL2/TEL3/TEL4)
_scroll_diag         = 0
_scroll_log          = 0
_scroll_ext          = 0

# ── Stato interno per diagnostiche derivate (non presenti direttamente
#    nella telemetria, calcolate qui a partire dai dati ricevuti) ────────
_pitot_anomalia_dal = None      # timestamp (ms) di inizio divergenza pitot/GPS persistente
_relay_prec         = False
_last_tel1_packet   = None      # ultimo numero progressivo pacchetto TEL1 ricevuto
_modalita_prec      = None      # modalità di volo del frame precedente (per rilevare i cambi)


def evaluate_diagnostics(t):
    global _pitot_anomalia_dal, _relay_prec, _modalita_prec

    if t["alarm_failsafe"]:
        add_alert("failsafe", "CRITICAL", "FAILSAFE ATTIVO",
                   causa="Perdita del segnale radio (pacchetto SBUS non valido)",
                   valore="Modalità forzata: FAILSAFE (controllo automatico verso il target impostato)",
                   azione="Verificare trasmettitore/ricevente e ripristinare il collegamento radio.")
    else:
        clear_alert("failsafe", "FAILSAFE DISATTIVATO")

    if "crash_safety_enabled" in t and not t["crash_safety_enabled"]:
        add_alert("crash_safety_off", "WARNING", "RILEVAMENTO SCHIANTO DISATTIVATO",
                   causa="Sicurezza schianto disabilitata da comando terra (SICUREZZA_SCHIANTO_OFF)",
                   valore="Nessun controllo del IMU sull'impatto in corso",
                   azione="Riattivare la sicurezza schianto se non intenzionale dal terra") 
    else:
        clear_alert("crash_safety_off", "RILEVAMENTO SCHIANTO ATTIVO")

    if t["alarm_crash"]:
        rc5_info = f"  |  Canale RC5: {t['rc_ch5']}" if "rc_ch5" in t else ""
        add_alert("crash", "CRITICAL", "SCHIANTO RILEVATO",
                   causa=f"Accelerazione IMU oltre la soglia di impatto: {SOGLIA_G_SCHIANTO} per 3 campioni consecutivi",
                   valore="Gas: NEUTRO forzato — servi interni ED esterni staccati" + rc5_info,
                   azione="Sblocco possibile SOLO da radiocomando: portare il canale 5 sotto 992 "
                          "(riarma e ricentra automaticamente tutti i servi).")
    else:
        clear_alert("crash", "SBLOCCO EMERGENZA ESEGUITO (servi riarmati e ricentrati)")

    thermal_enabled = t.get("thermal_safety_enabled", True)
    if not thermal_enabled:
        if t["t_motor"] >= FW_T_MOTORE_THROTTLE_START:
            add_alert("motore_termico", "CRITICAL", "PROTEZIONE TERMICA DISATTIVATA — MOTORE SENZA LIMITE",
                       causa="Temperatura oltre soglia di intervento MA sistema_sicurezza_temp è disattivato",
                       valore=f"Temperatura: {t['t_motor']:.1f} °C  |  Gas erogato: {t['pid_gas']*100:.0f} % (non limitato) da firmware",
                       soglia=f"Soglia normalmente attiva da {FW_T_MOTORE_THROTTLE_START:.0f} °C",
                       azione="Riattivare la protezione termica (SICUREZZA_TEMP_ON) o ridurre manualmente il gas.")
        else:
            clear_alert("motore_termico", "PROTEZIONE TERMICA DISATTIVATA (temperatura comunque nella norma)")
            
    elif t.get("thermal_limiting_active", t["thermal_limit"] < FW_GAS_MASSIMO):
        pct_limite = round((t["thermal_limit"] - FW_GAS_MINIMO) / (FW_GAS_MASSIMO - FW_GAS_MINIMO) * 100)
        add_alert("motore_termico", "WARNING", "POTENZA MOTORE LIMITATA",
                   causa="PROTEZIONE TERMICA ATTIVA",
                   valore=f"Temperatura: {t['t_motor']:.1f} °C  |  Gas erogato: {t['pid_gas']*100:.0f} % (limitato) da firmware",
                   soglia=f"Inizio limitazione: {FW_T_MOTORE_THROTTLE_START:.0f} °C — Limite massimo: {FW_T_MOTORE_THROTTLE_END:.0f} °C",
                   azione="Ridurre il regime motore o attendere il raffreddamento. "
                          f"Gas massimo consentito ora: {t['thermal_limit']} µs (~{pct_limite}%).")
    else:
        clear_alert("motore_termico", "POTENZA MOTORE RIPRISTINATA")

    power_enabled = t.get("power_safety_enabled", True)
    if not power_enabled:
        add_alert("alimentazione_safety_off", "WARNING", "DIAGNOSTICA ALIMENTAZIONE DISATTIVATA",
                   causa="Sicurezza alimentazione disabilitata da comando (SICUREZZA_ALIMENTAZIONE_OFF)",
                   valore="Nessun controllo su batteria motore/Teensy né failover relè",
                   azione="Riattivare (SICUREZZA_ALIMENTAZIONE_ON) se non intenzionale.")
    else:
        clear_alert("alimentazione_safety_off", "DIAGNOSTICA ALIMENTAZIONE ATTIVA")

    if power_enabled and t["alarm_batt_motor"]:
        add_alert("batt_motore", "WARNING", "BATTERIA MOTORE BASSA",
                   causa="Tensione sotto la soglia minima operativa",
                   valore=f"Tensione: {t['v_motor']:.2f} V",
                   soglia=f"{FW_VALORE_BATT_MOTORE_BASSA:.2f} V",
                   azione="Atterrare e sostituire/ricaricare la batteria di potenza e diminuire il regime motore.")
    else:
        clear_alert("batt_motore", "BATTERIA MOTORE OK")

    if power_enabled and t["alarm_batt_teensy"]:
        add_alert("batt_teensy", "WARNING", "BATTERIA AVIONICA (TEENSY) BASSA",
                   causa="Tensione sotto la soglia minima operativa",
                   valore=f"Tensione: {t['v_teensy']:.2f} V  |  Failover relè: {'ATTIVO' if t['relay'] else 'in attesa'}",
                   soglia=f"{FW_VALORE_BATT_TEENSY_BASSA:.2f} V",
                   azione="Verificare BEC/regolatore avionica; il relè commuta automaticamente sulla batteria motore. "
                          "NOTA: i servi interni vengono staccati automaticamente per risparmiare corrente.")
    else:
        clear_alert("batt_teensy", "BATTERIA AVIONICA OK")

    if t["relay"] and not _relay_prec:
        add_alert("relay_on", "INFO", "FAILOVER ALIMENTAZIONE ATTIVATO",
                   causa="Batteria avionica (Teensy) sotto soglia",
                   valore="Relè commutato: l'avionica è ora alimentata dalla batteria motore",
                   azione="Pianificare l'atterraggio per sostituire la batteria avionica.")
    elif not t["relay"] and _relay_prec:
        clear_alert("relay_on", "FAILOVER ALIMENTAZIONE DISATTIVATO")
    _relay_prec = t["relay"]

    servo_enabled = t.get("servo_safety_enabled", True)
    if not servo_enabled:
        add_alert("servo_safety_off", "WARNING", "DIAGNOSTICA CORRENTE SERVI DISATTIVATA",
                   causa="Sicurezza servi disabilitata da comando (SICUREZZA_SERVI_OFF)",
                   valore="Tutti i servi sono considerati forzatamente OK, nessuna anomalia rilevabile",
                   azione="Riattivare (SICUREZZA_SERVI_ON) se non intenzionale.")
        clear_alert("servi_anomalia", "DIAGNOSTICA SERVI DISATTIVATA")
        clear_alert("servi_risparmio_batteria", "DIAGNOSTICA SERVI DISATTIVATA")
    else:
        clear_alert("servo_safety_off", "DIAGNOSTICA CORRENTE SERVI ATTIVA")

        def _is_anomalia_reale(nome_corrente):
            if nome_corrente in t:
                return t[nome_corrente] < 0.5 or t[nome_corrente] > 2500.0
            return None  # sconosciuto

        servi_ko_reali = []
        servi_off_batteria = []
        candidati = (("Int SX", t["ok_isx"], "i_int_sx"), ("Int DX", t["ok_idx"], "i_int_dx"),
                     ("Est SX", t["ok_esx"], "i_est_sx"), ("Est DX", t["ok_edx"], "i_est_dx"))
        for nome, ok, campo_corrente in candidati:
            if ok:
                continue
            anomalia_reale = _is_anomalia_reale(campo_corrente)
            e_interno = nome.startswith("Int")
            if anomalia_reale is False and e_interno and t.get("alarm_batt_teensy", False):
                servi_off_batteria.append(nome)
            elif anomalia_reale is None and e_interno and t.get("alarm_batt_teensy", False):
                servi_off_batteria.append(nome)
            else:
                servi_ko_reali.append(nome)

        if servi_ko_reali:
            add_alert("servi_anomalia", "WARNING" if len(servi_ko_reali) < 4 else "CRITICAL",
                       "ANOMALIA CORRENTE SERVO",
                       causa="Corrente fuori range (0.5–2500 mA) per oltre 5 letture consecutive",
                       valore="Servi in anomalia: " + ", ".join(servi_ko_reali),
                       azione="Il mixer si è riconfigurato automaticamente sui servi rimanenti; "
                              "verificare cablaggio/meccanica dei servi indicati appena possibile.")
        else:
            clear_alert("servi_anomalia", "SERVI RIPRISTINATI")

        if servi_off_batteria:
            add_alert("servi_risparmio_batteria", "INFO", "SERVI INTERNI STACCATI (RISPARMIO BATTERIA)",
                       causa="Batteria Teensy bassa: il mixer disattiva i servi interni per risparmiare corrente",
                       valore="Servi disattivati: " + ", ".join(servi_off_batteria),
                       azione="Non è un guasto meccanico/elettrico: verrà ripristinato al rientro della tensione Teensy.")
        else:
            clear_alert("servi_risparmio_batteria", "SERVI INTERNI RIATTIVATI")

    if t["satellites"] == 0:
        add_alert("gps_fix", "WARNING", "GPS NON DISPONIBILE",
                   causa="Nessun fix GPS valido",
                   valore="Satelliti: 0",
                   azione="Attendere l'acquisizione satellitare; la navigazione automatica non è affidabile spera che non perdi il drone.")
    elif t["satellites"] < FW_SAT_MIN_FIX:
        add_alert("gps_fix", "INFO", "GPS SEGNALE DEBOLE",
                   causa="Numero di satelliti sotto il margine di sicurezza consigliato",
                   valore=f"Satelliti: {t['satellites']}", soglia=f"{FW_SAT_MIN_FIX}",
                   azione="Nessuna azione immediata richiesta; monitorare la qualità del fix.")
    else:
        clear_alert("gps_fix", "GPS FIX ACQUISITO")

    if "gps_speed_valid" in t and not t["gps_speed_valid"]:
        add_alert("gps_speed_invalid", "INFO", "VELOCITÀ GPS NON VALIDA",
                   causa="gps.speed.isValid() è falso: il ricevitore non fornisce una velocità attendibile",
                   valore="Velocità al suolo calcolata come 0 (o da flusso ottico se disponibile)",
                   azione="Nessuna azione immediata; il sistema usa il fallback disponibile.")
    else:
        clear_alert("gps_speed_invalid", "VELOCITÀ GPS VALIDA")

    if not t["sensor_lidar_ok"]:
        add_alert("lidar", "WARNING", "LIDAR NON DISPONIBILE",
                   causa="Inizializzazione del sensore non riuscita in fase di avvio",
                   azione="Il sistema prosegue con il solo barometro per la quota; verificare il TF-Luna al prossimo atterraggio.")
    elif t["alt_baro_raw"] > FW_ALTEZZA_MAX_LIDAR:
        add_alert("lidar", "STATUS", "LIDAR NON UTILIZZATO",
                   causa="Quota fuori dall'intervallo operativo del sensore (normale in crociera)",
                   valore=f"Quota: {t['alt_baro_raw']:.1f} m", soglia=f"{FW_ALTEZZA_MAX_LIDAR:.0f} m",
                   azione="Nessuna azione: condizione attesa, NON è un errore.")
    else:
        clear_alert("lidar")

    if not t["sensor_optflow_ok"]:
        add_alert("optflow", "WARNING", "FLUSSO OTTICO NON DISPONIBILE",
                   causa="Inizializzazione del sensore PMW3901 non riuscita in fase di avvio",
                   azione="Il sistema prosegue senza questa sorgente di velocità; verificare i cavi SPI al prossimo atterraggio.")
    elif t["altitude"] > FW_ALTEZZA_MAX_SENSORE_OTTICO:
        add_alert("optflow", "STATUS", "FLUSSO OTTICO NON UTILIZZATO",
                   causa="Quota fuori dall'intervallo operativo del sensore (normale sopra i 4 m)",
                   valore=f"Quota: {t['altitude']:.1f} m", soglia=f"{FW_ALTEZZA_MAX_SENSORE_OTTICO:.0f} m",
                   azione="Nessuna azione: condizione attesa, NON è un errore.")
    else:
        clear_alert("optflow")

    if t["tel1_lost_count"] > 0:
        add_alert("tel1_lost", "INFO", "PACCHETTI TELEMETRIA PERSI",
                   causa="Salti rilevati nel numero progressivo del pacchetto TEL1 (radio LoRa)",
                   valore=f"Pacchetti persi stimati: {t['tel1_lost_count']}",
                   azione="Verificare qualità del collegamento LoRa/antenne se il numero cresce rapidamente.")
    else:
        clear_alert("tel1_lost")

    if "distance_to_target" in t:
        if t["distance_to_target"] > FW_DISTANZA_FRENATA:
            add_alert("fase_volo", "STATUS", "FASE: CROCIERA",
                       causa=f"Distanza dal target ({t['distance_to_target']:.0f} m) oltre la distanza di frenata",
                       valore="Velocità target: crociera  |  Gas di base: crociera",
                       soglia=f"{FW_DISTANZA_FRENATA:.0f} m",
                       azione="Nessuna azione: fase operativa normale.")
        else:
            add_alert("fase_volo", "STATUS", "FASE: AVVICINAMENTO",
                       causa=f"Distanza dal target ({t['distance_to_target']:.0f} m) sotto la distanza di frenata",
                       valore="Velocità target: avvicinamento  |  Gas di base: avvicinamento",
                       soglia=f"{FW_DISTANZA_FRENATA:.0f} m",
                       azione="Nessuna azione: fase operativa normale.")

    if "flight_mode" in t:
        stato_attuale = 3 if t["alarm_failsafe"] else t["flight_mode"]
        if _modalita_prec is not None and stato_attuale != _modalita_prec:
            nomi = {1: "MANUALE", 2: "AUTO", 3: "FAILSAFE"}
            add_alert("pid_reset", "INFO", "PID RESETTATO (cambio modalità di volo)",
                       causa=f"Transizione modalità: {nomi.get(_modalita_prec, _modalita_prec)} → {nomi.get(stato_attuale, stato_attuale)}",
                       valore="Integrali e derivate di quota/pitch/roll/velocità azzerati",
                       azione="Nessuna azione: comportamento atteso ad ogni cambio modalità.")
        _modalita_prec = stato_attuale


    if "pid_target_pitch_auto" in t and "pid_alt_error" in t:
        tp = t["pid_target_pitch_auto"]
        ae = t["pid_alt_error"]
        alt_info = f"  |  Quota attuale: {t['altitude']:.1f} m" if "altitude" in t else ""
        if abs(tp - PITCH_DOWN_FORZATO) < 0.05 and abs(ae) < 0.001:
            add_alert("alt_override", "CRITICAL", "QUOTA MASSIMA SUPERATA — PICCHIATA FORZATA",
                       causa="G_altitudine_m > ALTEZZA_MAX_m: il PID quota è bypassato dal firmware",
                       valore=f"Pitch forzato: {PITCH_DOWN_FORZATO}° (picchiata)  |  Gas forzato: GAS_MINIMO{alt_info}",
                       azione="Comportamento di sicurezza del firmware, non un guasto: il drone sta scendendo "
                              "forzatamente per rientrare sotto il limite di quota massima.")
        elif abs(tp - PITCH_UP_FORZATO) < 0.05 and abs(ae) < 0.001:
            add_alert("alt_override", "CRITICAL", "QUOTA MINIMA VIOLATA — CABRATA FORZATA",
                       causa="G_altitudine_m < ALTEZZA_MIN_m: il PID quota è bypassato dal firmware",
                       valore=f"Pitch forzato: {PITCH_UP_FORZATO}° (cabrata)  |  Gas forzato: quasi GAS_MASSIMO{alt_info}",
                       azione="Comportamento di sicurezza del firmware, non un guasto: il drone sta salendo "
                              "forzatamente per rientrare sopra il limite di quota minima. Attenzione a stallo/ostacoli.")
        else:
            clear_alert("alt_override", "QUOTA NEL RANGE NORMALE (PID attivo)")

    esterni_attivi = t["ok_esx"] and t["ok_edx"]
    interni_attivi = t["ok_isx"] and t["ok_idx"]
    if esterni_attivi and interni_attivi:
        clear_alert("mixer_degradato", "MIXER NORMALE (interni=pitch, esterni=roll)")
    elif esterni_attivi and not interni_attivi:
        add_alert("mixer_degradato", "WARNING", "MIXER DEGRADATO — SOLO SERVI ESTERNI",
                   causa="Servi interni non disponibili (guasto o staccati)",
                   valore="I servi esterni ora comandano pitch+roll combinati",
                   azione="Autorità di controllo ridotta. Ripristinare i servi interni appena possibile.")
    elif interni_attivi and not esterni_attivi:
        add_alert("mixer_degradato", "WARNING", "MIXER DEGRADATO — SOLO SERVI INTERNI",
                   causa="Servi esterni non disponibili (guasto o staccati)",
                   valore="I servi interni ora comandano pitch+roll combinati",
                   azione="Autorità di controllo ridotta. Ripristinare i servi esterni appena possibile.")
    else:
        add_alert("mixer_degradato", "CRITICAL", "PERDITA TOTALE CONTROLLO ASSETTO",
                   causa="Sia i servi interni che quelli esterni sono in anomalia/staccati contemporaneamente",
                   valore="Il mixer non invia più alcun comando a pitch/roll (return anticipato nel firmware)",
                   azione="EMERGENZA: nessun controllo di assetto disponibile. Atterraggio/recupero immediato.")

    if "takeoff_speed_ok" in t and "takeoff_alt_ok" in t and not t["in_flight"]:
        if t["takeoff_speed_ok"] and t["takeoff_alt_ok"]:
            add_alert("decollo_conferma", "INFO", "DECOLLO IN CORSO DI CONFERMA",
                       causa="Velocità e quota di decollo raggiunte, in attesa del debounce (1.5 s) prima di droneInVolo=true",
                       valore="Condizioni di decollo soddisfatte",
                       azione="Nessuna azione: comportamento atteso, la conferma è imminente.")
        else:
            clear_alert("decollo_conferma", "CONDIZIONI DI DECOLLO NON ANCORA SODDISFATTE")
    else:
        clear_alert("decollo_conferma")


def send_command(campo: str, valore) -> bool:
    global _ser_instance
    if _ser_instance is None or not _ser_instance.is_open:
        print("[CMD] Seriale non disponibile")
        return False
    msg = f"CMD:{campo}:{valore}\n".encode('utf-8')
    with _serial_lock:
        try:
            _ser_instance.write(msg)
            print(f"[CMD] ▶ {msg.strip()}")
            return True
        except serial.SerialException as e:
            print(f"[CMD] Errore invio: {e}")
            return False


def read_from_serial():
    global _ser_instance
    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            _ser_instance = ser
            T["serial_ok"] = True
            print(f"[SERIAL] Connesso a {SERIAL_PORT} @ {BAUD_RATE} baud.")
            while True:
                with _serial_lock:
                    raw = ser.readline()
                if raw:
                    parse_telemetry(raw.decode('utf-8', errors='ignore').strip())
        except serial.SerialException as e:
            _ser_instance = None
            T["serial_ok"] = False
            print(f"[SERIAL] Porta non disponibile: {e} — nuovo tentativo in 3s")
            pygame.time.wait(3000)


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


def parse_telemetry(line):
    """
    Smista la riga ricevuta dal firmware in base al prefisso:
      "$,"  -> TEL1 (stato/assetto/nav/batterie/servi — ~2 Hz, sempre presente)
      "$2," -> TEL2 (correnti/potenze, flusso ottico grezzo, gas pre-limite)
      "$3," -> TEL3 (diagnostica PID completa)
      "$4," -> TEL4 (GPS/baro/pitot estesi, IMU estesa, RC 4-16)
    TEL2/3/4 arrivano in round-robin ogni ~2s (vedi inviaTelemetria() in main.ino)
    e alimentano la schermata "TELEMETRIA ESTESA" (tasto E), non la PFD principale.
    """
    line = line.strip()
    if not line:
        return
    try:
        if line.startswith("$2,"):
            parse_tel2(line)
        elif line.startswith("$3,"):
            parse_tel3(line)
        elif line.startswith("$4,"):
            parse_tel4(line)
        elif line.startswith("$,"):
            parse_tel1(line)
        else:
            print(f"[WARN] Pacchetto sconosciuto: {line[:24]}")
    except Exception as e:
        print(f"[ERR] Pacchetto corrotto saltato: {e}")


def parse_tel1(line):
    """
    Pacchetto principale ($,) — 46 campi (indice 0 = '$'), sincronizzato con
    inviaTelemetria() v2 in main.ino:
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
      16 : Airspeed fusa        [km/h]  (usata dal PID/autothrottle)
      17 : Groundspeed fusa     [km/h]  (usata dal controllore L1)
      18 : Distanza target      [m]
      19 : Rotta verso target   [°]
      20 : Target roll (L1)     [°]
      21 : RC Pitch             [µs 172-1811]
      22 : RC Roll              [µs]
      23 : RC Gas               [µs]
      24 : PID out Pitch        [µs]
      25 : PID out Roll         [µs]
      26 : PID out Gas          [µs ~1000-2000]
      27 : Pos Servo Int SX     [°]
      28 : Pos Servo Int DX     [°]
      29 : Pos Servo Est SX     [°]
      30 : Pos Servo Est DX     [°]
      31 : Temperatura motore   [°C]
      32 : Temperatura avionica [°C]
      33 : Satelliti GPS
      34 : Latitudine           [°]
      35 : Longitudine          [°]
      36 : Relè attivato        (0/1)
      37 : Vel X flusso ottico  [m/s]
      38 : Vel Y flusso ottico  [m/s]
      39 : Stato sensori        bitmask (b0=flusso ottico OK, b1=LIDAR OK, b2=pacchetto SBUS perso)
      40 : Errore rotta         [°]
      41 : Limite gas termico   [µs]
      42 : Altitudine LIDAR grezza [m]
      43 : Altitudine Baro grezza   [m]
      44 : timestamp firmware (millis())
      45 : numero progressivo pacchetto
    """
    global T, _last_tel1_packet
    f = line.split(',')
    if len(f) < 46 or f[0] != '$':
        print(f"[WARN] Pacchetto TEL1 non valido (len={len(f)})")
        return

    T["mode"] = {1: "MANUALE", 2: "AUTO PID", 3: "FAILSAFE!"}.get(int(f[1]), "SCONOSCIUTA")

    alarm = int(f[2])
    T["alarm_failsafe"]    = bool(alarm & 1)
    T["alarm_batt_motor"]  = bool(alarm & 2)
    T["alarm_relay"]       = bool(alarm & 4)
    T["alarm_batt_teensy"] = bool(alarm & 8)
    T["alarm_crash"]       = bool(alarm & 16)
    T["in_flight"]         = bool(alarm & 32)

    T["v_motor"]  = float(f[3])
    T["v_teensy"] = float(f[4])
    T["v_isx"] = float(f[5])
    T["v_idx"] = float(f[6])
    T["v_esx"] = float(f[7])
    T["v_edx"] = float(f[8])

    health = f[9].strip()
    T["ok_isx"] = len(health) > 0 and health[0] == '1'
    T["ok_idx"] = len(health) > 1 and health[1] == '1'
    T["ok_esx"] = len(health) > 2 and health[2] == '1'
    T["ok_edx"] = len(health) > 3 and health[3] == '1'

    T["pitch"]    = float(f[10])
    T["roll"]     = float(f[11])
    T["yaw"]      = float(f[12])
    T["altitude"] = float(f[13])

    T["spd_pitot"]  = float(f[14])
    T["spd_gps"]    = float(f[15])
    T["spd_fused"]  = float(f[16])   # Airspeed fusa (usata dal PID/autothrottle)
    T["spd_ground"] = float(f[17])   # Groundspeed fusa (usata dal controllore L1)
    T["spd_ms"]     = T["spd_fused"] / 3.6   # km/h → m/s per l'HUD (airspeed)

    T["dist_target"] = float(f[18])
    T["hdg_target"]  = float(f[19])
    T["roll_target"] = float(f[20])

    T["rc_pitch"] = float(f[21])
    T["rc_roll"]  = float(f[22])
    T["rc_gas"]   = float(f[23])

    T["pid_pitch"] = float(f[24])
    T["pid_roll"]  = float(f[25])
    T["pid_gas"]   = clamp((float(f[26]) - GAS_MINIMO) / (GAS_MASSIMO - GAS_MINIMO), 0.0, 1.0)
    T["throttle"]  = T["pid_gas"]

    T["deg_isx"] = float(f[27])
    T["deg_idx"] = float(f[28])
    T["deg_esx"] = float(f[29])
    T["deg_edx"] = float(f[30])

    T["t_motor"]  = float(f[31])
    T["t_teensy"] = float(f[32])

    T["satellites"] = int(f[33])
    T["lat"] = float(f[34])
    T["lon"] = float(f[35])

    T["relay"] = f[36].strip() == '1'

    T["opt_vx"] = float(f[37])
    T["opt_vy"] = float(f[38])

    stato_sensori = int(f[39])
    T["sensor_optflow_ok"]  = bool(stato_sensori & 1)
    T["sensor_lidar_ok"]    = bool(stato_sensori & 2)
    T["sensor_packet_lost"] = bool(stato_sensori & 4)

    T["heading_error"] = float(f[40])
    T["thermal_limit"] = int(f[41])
    T["alt_lidar_raw"] = float(f[42])
    T["alt_baro_raw"]  = float(f[43])

    # ── timestamp firmware + contatore pacchetto (rilevamento perdite) ──
    T["fw_millis"] = int(f[44])
    pacchetto_num  = int(f[45])
    if _last_tel1_packet is not None and pacchetto_num > _last_tel1_packet:
        atteso = _last_tel1_packet + 1
        if pacchetto_num != atteso:
            T["tel1_lost_count"] += (pacchetto_num - atteso)
    _last_tel1_packet = pacchetto_num
    T["tel1_packet_num"] = pacchetto_num



def parse_tel2(line):
    """
    TEL2 ($2,) — correnti/potenze batterie, flusso ottico grezzo, gas pre-limite:
      1 corrente motore [mA], 2 corrente Teensy [mA],
      3-6 correnti servi Int SX/Int DX/Est SX/Est DX [mA],
      7 potenza motore [mW], 8 potenza Teensy [mW],
      9 batteria motore bassa (0/1), 10 batteria Teensy bassa (0/1),
      11-12 flusso ottico dx/dy grezzo, 13 gas pre-limite termico [µs],
      14 limitazione termica attiva (0/1)
    """
    f = line.split(',')
    if len(f) < 15 or f[0] != '$2':
        print(f"[WARN] Pacchetto TEL2 non valido (len={len(f)})")
        return
    T2["curr_motor"]  = float(f[1])
    T2["curr_teensy"] = float(f[2])
    T2["curr_isx"]    = float(f[3])
    T2["curr_idx"]    = float(f[4])
    T2["curr_esx"]    = float(f[5])
    T2["curr_edx"]    = float(f[6])
    T2["pow_motor"]   = float(f[7])
    T2["pow_teensy"]  = float(f[8])
    T2["batt_low_motor"]  = f[9].strip() == '1'
    T2["batt_low_teensy"] = f[10].strip() == '1'
    T2["flow_dx"] = int(float(f[11]))
    T2["flow_dy"] = int(float(f[12]))
    T2["gas_pre_limit"] = int(float(f[13]))
    T2["thermal_limit_active"] = f[14].strip() == '1'


def parse_tel3(line):
    """
    TEL3 ($3,) — diagnostica PID completa:
      1-4   alt: errore/P/I/D
      5     target pitch (uscita del PID quota)
      6-9   pitch: errore/P/I/D
      10-13 roll: errore/P/I/D
      14-17 vel (autothrottle): errore/P/I/D
      18    target altitudine [m]
      19    target velocità attuale [km/h]
    """
    f = line.split(',')
    if len(f) < 20 or f[0] != '$3':
        print(f"[WARN] Pacchetto TEL3 non valido (len={len(f)})")
        return
    T3["alt_err"] = float(f[1]); T3["alt_p"] = float(f[2]); T3["alt_i"] = float(f[3]); T3["alt_d"] = float(f[4])
    T3["target_pitch_auto"] = float(f[5])
    T3["pitch_err"] = float(f[6]); T3["pitch_p"] = float(f[7]); T3["pitch_i"] = float(f[8]); T3["pitch_d"] = float(f[9])
    T3["roll_err"] = float(f[10]); T3["roll_p"] = float(f[11]); T3["roll_i"] = float(f[12]); T3["roll_d"] = float(f[13])
    T3["vel_err"] = float(f[14]); T3["vel_p"] = float(f[15]); T3["vel_i"] = float(f[16]); T3["vel_d"] = float(f[17])
    T3["alt_target"] = float(f[18])
    T3["vel_target"] = float(f[19])


def parse_tel4(line):
    """
    TEL4 ($4,) — GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC 4-16:
      1  altitudine GPS [m] (-1 se non valida)
      2  rotta GPS (course) [°] (-1 se non valida)
      3  velocità GPS valida (0/1)
      4  rotta GPS valida (0/1)
      5  fix posizione GPS valido (0/1)
      6  pressione barometrica [Pa]
      7  tara altitudine ASL [m]
      8  barometro pronto (0/1)
      9  pitot grezzo (ADC)
      10 pitot zero (taratura)
      11 pitot differenza
      12 pitot valido (0/1)
      13-15 offset pitch/roll/yaw (tara IMU)
      16-18 accelerazione lineare X/Y/Z [m/s²]
      19 accelerazione totale [m/s²]
      20-22 giroscopio X/Y/Z [°/s]
      23-26 calibrazione IMU sys/gyro/accel/mag (0-3)
      27-39 canali RC 4-16 [µs] (13 valori)
    """
    f = line.split(',')
    if len(f) < 40 or f[0] != '$4':
        print(f"[WARN] Pacchetto TEL4 non valido (len={len(f)})")
        return
    T4["gps_alt"] = float(f[1])
    T4["gps_course"] = float(f[2])
    T4["gps_speed_valid"]  = f[3].strip() == '1'
    T4["gps_course_valid"] = f[4].strip() == '1'
    T4["gps_loc_valid"]    = f[5].strip() == '1'
    T4["baro_pressure"] = float(f[6])
    T4["baro_tare"]     = float(f[7])
    T4["baro_ready"]    = f[8].strip() == '1'
    T4["pitot_raw"]  = int(float(f[9]))
    T4["pitot_zero"] = float(f[10])
    T4["pitot_diff"] = float(f[11])
    T4["pitot_valid"] = f[12].strip() == '1'
    T4["off_pitch"] = float(f[13]); T4["off_roll"] = float(f[14]); T4["off_yaw"] = float(f[15])
    T4["accel_x"] = float(f[16]); T4["accel_y"] = float(f[17]); T4["accel_z"] = float(f[18])
    T4["accel_tot"] = float(f[19])
    T4["gyro_x"] = float(f[20]); T4["gyro_y"] = float(f[21]); T4["gyro_z"] = float(f[22])
    T4["imu_cal_sys"]   = int(float(f[23]))
    T4["imu_cal_gyro"]  = int(float(f[24]))
    T4["imu_cal_accel"] = int(float(f[25]))
    T4["imu_cal_mag"]   = int(float(f[26]))
    rc = f[27:40]
    T4["rc_ext"] = [int(float(v)) for v in rc if v.strip() != ""]


def text(surf, s, pos, font, color, anchor="topleft"):
    img = font.render(s, True, color)
    surf.blit(img, img.get_rect(**{anchor: pos}))


def draw_panel(surf, rect, title=""):
    BAR_H = 26
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=8)
    pygame.draw.rect(surf, C_BORDER, rect, 2, border_radius=8)
    if title:
        bar = pygame.Rect(rect.x, rect.y, rect.w, BAR_H)
        pygame.draw.rect(surf, C_BORDER, bar, 2, border_top_left_radius=8, border_top_right_radius=8)
        text(surf, title, bar.center, F_TITLE, C_ACCENT, anchor="center")


def draw_kv(surf, label, value, x, y, col_v=C_WHITE, col_k=C_DIM):
    text(surf, label, (x, y), F_LABEL, col_k)
    text(surf, value, (x + 110, y), F_VAL, col_v)


def draw_vbar(surf, rect, value, vmin, vmax, color=C_GREEN):
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=5)
    pygame.draw.rect(surf, C_BORDER, rect, 2, border_radius=5)
    if vmax > vmin:
        pct = clamp((value - vmin) / (vmax - vmin), 0.0, 1.0)
        fill_h = int((rect.height - 6) * pct)
        if fill_h > 0:
            bar_col = C_RED if pct < 0.2 else color
            fill = pygame.Rect(rect.x + 3, rect.bottom - 3 - fill_h, rect.width - 6, fill_h)
            pygame.draw.rect(surf, bar_col, fill, border_radius=3)
    text(surf, f"{value:.2f}", rect.center, F_SMALL, C_WHITE, anchor="center")


def draw_servo_leds(surf, x, y, degrees, label, v_servo, error=False):
    LED_W, LED_H, LED_GAP, N = 40, 20, 5, 5
    delta = degrees - 90
    leds = [False] * N
    leds[2] = True
    if delta > 10: leds[3] = True
    if delta > 25: leds[4] = True
    if delta < -10: leds[1] = True
    if delta < -25: leds[0] = True

    text(surf, label, (x + LED_W // 2, y - 18), F_SMALL, C_TEXT, anchor="center")
    for i in range(N):
        y_led = y + (N - 1 - i) * (LED_H + LED_GAP)
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
         (x + LED_W // 2, y_info), F_SMALL, C_RED if error else C_WHITE, anchor="center")


def draw_hud_box(surf, rect, value, label="", unit=""):
    pygame.draw.rect(surf, (22, 28, 44), rect, border_radius=5)
    pygame.draw.rect(surf, C_YELLOW, rect, 2, border_radius=5)
    if label:
        text(surf, label, (rect.centerx, rect.top - 13), F_SMALL, C_DIM, anchor="center")
    text(surf, f"{value:.1f}", rect.center, F_VAL, C_YELLOW, anchor="center")
    if unit:
        text(surf, unit, (rect.centerx, rect.bottom + 6), F_SMALL, C_DIM, anchor="center")


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


def draw_header(surf, t):
    """Barra superiore: satelliti, modalità, yaw."""
    text(surf, t["mode"], (W // 2, 12), F_HEAD, C_WHITE, anchor="midtop")
    if t["in_flight"]:
        text(surf, "IN FLIGHT", (W - 28, 12), F_HEAD, C_ACCENT, anchor="topright")
    elif t["alarm_failsafe"]:
        text(surf, "FAILSAFE!", (W - 28, 12), F_HEAD, C_RED, anchor="topright")
    elif t["alarm_crash"]:
        text(surf, "CRASH!", (W - 28, 12), F_HEAD, C_RED, anchor="topright")
    else:
        text(surf, "ON GROUND", (W - 28, 12), F_HEAD, C_DIM, anchor="topright")

    # Indicatore connessione seriale
    serial_col = C_GREEN if t["serial_ok"] else C_RED
    serial_lbl = f"SERIAL {SERIAL_PORT}" if t["serial_ok"] else f"NO SERIAL ({SERIAL_PORT})"
    text(surf, serial_lbl, (28, 12), F_TITLE, serial_col)


def draw_power_panel(surf, t):
    draw_panel(surf, POWER_PANEL, "POWER")

    thr = pygame.Rect(POWER_PANEL.x + 20, POWER_PANEL.y + 40, 100, 196)
    draw_vbar(surf, thr, t["throttle"], 0.0, 1.0,
              C_GREEN if t["throttle"] < THR_WARN else C_YELLOW)
    text(surf, "THR", (thr.centerx, thr.bottom + 14), F_SMALL, C_DIM, anchor="center")

    title = "RC CONTROLLER" if t["mode"] == "MANUALE" else "PID CONTROLLER"
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
    draw_kv(surf, "TEENSY", f"{t['t_teensy']:.1f} °C", kx, ky,
            col_v=(C_WHITE if t["t_teensy"] < T_TEENSY_WARN
                   else C_YELLOW if t["t_teensy"] < T_TEENSY_CRIT
                   else C_RED))
    draw_kv(surf, "MOTORE", f"{t['t_motor']:.1f} °C", kx, ky + 34,
            col_v=(C_WHITE if t["t_motor"] < T_MOTOR_WARN
                   else C_YELLOW if t["t_motor"] < T_MOTOR_CRIT
                   else C_RED))


def draw_speed_panel(surf, t):
    draw_panel(surf, SPEED_PANEL, "SPEED")
    entries = [("PITOT", t["spd_pitot"],  "km/h"),
               ("GPS",   t["spd_gps"],    "km/h"),]
    bw, bh, gap = 78, 120, 8
    bx = SPEED_PANEL.x + 14
    by = SPEED_PANEL.y + 50
    for label, val, unit in entries:
        r = pygame.Rect(bx, by, bw, bh)
        draw_vbar(surf, r, val, 0, 100, C_GREEN)
        text(surf, label, (r.centerx, r.top - 14), F_SMALL, C_DIM, anchor="center")
        text(surf, unit, (r.centerx, r.bottom + 6), F_SMALL, C_DIM, anchor="center")
        bx += bw + gap


def servo_ok(v):
    return SERVO_V_MIN < v < SERVO_V_MAX


def draw_servi_panel(surf, t):
    draw_panel(surf, SERVI_PANEL, "SERVI")
    servos = [(t["deg_isx"], "Int SX", t["v_isx"]), (t["deg_idx"], "Int DX", t["v_idx"]),
              (t["deg_esx"], "Est SX", t["v_esx"]), (t["deg_edx"], "Est DX", t["v_edx"])]
    for i, (deg, lbl, v) in enumerate(servos):
        draw_servo_leds(surf, SERVI_X + i * SERVI_GAP, SERVI_Y, deg, lbl, v, error=not servo_ok(v))


def draw_vserv_panel(surf, t):
    draw_panel(surf, VSERV_PANEL, "TENSIONE SERVI")
    labels = ["Int SX", "Int DX", "Est SX", "Est DX"]
    vals = [t["v_isx"], t["v_idx"], t["v_esx"], t["v_edx"]]
    bw, bh, gap = 72, 120, 16
    bx = VSERV_PANEL.x + 20
    by = VSERV_PANEL.y + 55
    for label, val in zip(labels, vals):
        ok = servo_ok(val)
        col = C_GREEN if ok else C_RED
        r = pygame.Rect(bx, by, bw, bh)
        draw_vbar(surf, r, val, SERVO_V_MIN - 0.5, SERVO_V_MAX + 0.5, col)
        text(surf, label, (r.centerx, r.top - 14), F_SMALL, C_DIM, anchor="center")
        bx += bw + gap


def draw_pfd_center(surf, t):
    draw_horizon(surf, PFD_CX, PFD_CY, PFD_R, t["pitch"], t["roll"])
    draw_hud_box(surf, SPD_BOX, t["spd_ms"], "SPEED", "m/s")
    draw_hud_box(surf, ALT_BOX, t["altitude"], "ALT", "m")
    text(surf, f"TARGET  {t['dist_target']:.0f} m",
         (PFD_CX, PFD_CY + PFD_R + 26), F_LABEL, C_DIM, anchor="midtop")


def draw_nav_panel(surf, t):
    draw_panel(surf, NAV_PANEL, "NAV INFO")
    kx = NAV_PANEL.x + 20
    ky = NAV_PANEL.y + 40
    draw_kv(surf, "DIST TGT", f"{t['dist_target']:.1f} m", kx, ky,
            col_v=C_WHITE if t["dist_target"] >= DIST_TGT_WARN else C_YELLOW)
    draw_kv(surf, "HDG TGT", f"{t['hdg_target']:.1f}°", kx, ky + 34)
    draw_kv(surf, "ROLL TGT", f"{t['roll_target']:.1f}°", kx, ky + 68)
    draw_kv(surf, "SATELLITES", f"{t['satellites']}", kx + 215, ky,
            col_v=C_WHITE if t["satellites"] >= SAT_MIN else C_RED)
    draw_kv(surf, "LAT", f"{t['lat']:.6f}", kx + 215, ky + 34)
    draw_kv(surf, "LON", f"{t['lon']:.6f}", kx + 215, ky + 68)


def draw_battery_panel(surf, t):
    draw_panel(surf, BATTERY_PANNEL, "BATTERY")
    C_VUOTO = (18, 26, 42)
    kx = BATTERY_PANNEL.x + 40
    ky = BATTERY_PANNEL.y + 40
    bar_w, bar_h = 60, 100
    gap = 180

    V_t = pygame.Rect(kx, ky, bar_w, bar_h)
    draw_vbar(surf, V_t, t["v_teensy"], VBAR_TEENSY_MIN, VBAR_TEENSY_MAX,
              C_GREEN if not t["alarm_batt_teensy"] else C_RED)
    text(surf, "V SYS", (V_t.centerx, V_t.bottom + 14), F_SMALL, C_DIM, anchor="center")

    V_m = pygame.Rect(kx + bar_w + gap, ky, bar_w, bar_h)
    draw_vbar(surf, V_m, t["v_motor"], VBAR_MOTOR_MIN, VBAR_MOTOR_MAX,
              C_GREEN if not t["alarm_batt_motor"] else C_RED)
    text(surf, "V MOT", (V_m.centerx, V_m.bottom + 14), F_SMALL, C_DIM, anchor="center")

    rele_on = t["relay"]
    pipe_h = 14
    pipe_y = ky + bar_h // 2 - pipe_h // 2
    pipe_x1 = V_t.right
    pygame.draw.rect(surf, C_BORDER, (pipe_x1, pipe_y, gap, pipe_h))
    pygame.draw.rect(surf, C_VUOTO, (pipe_x1, pipe_y + 2, gap, pipe_h - 4))
    if rele_on:
        pygame.draw.rect(surf, C_GREEN, (pipe_x1, pipe_y + 2, gap, pipe_h - 4))

    vw, vh = 32, 32
    vx = pipe_x1 + gap // 2 - vw // 2
    vy = ky + bar_h // 2 - vh // 2
    valve_color = C_GREEN if rele_on else C_RED
    pygame.draw.rect(surf, (14, 20, 32), (vx, vy, vw, vh), border_radius=4)
    pygame.draw.rect(surf, valve_color, (vx, vy, vw, vh), 2, border_radius=4)
    if rele_on:
        pygame.draw.rect(surf, C_GREEN, (vx + 4, vy + vh // 2 - 4, vw - 8, 8))
        lbl_txt = "FAILOVER ON"
    else:
        pygame.draw.rect(surf, C_RED, (vx + vw // 2 - 4, vy + 4, 8, vh - 8))
        lbl_txt = "ISOLATI"
    text(surf, lbl_txt, (vx + vw // 2, vy - 12), F_SMALL, valve_color, anchor="center")


def draw_orientation_panel(surf, t):
    draw_panel(surf, ORIENTATION_PANEL, "ORIENTAZIONE")
    kx = ORIENTATION_PANEL.x + 20
    ky = ORIENTATION_PANEL.y + 40
    draw_kv(surf, "ROLL", f"{t['roll']:.1f}°", kx, ky,
            col_v=(C_WHITE if abs(t["roll"]) < ROLL_WARN
                   else C_YELLOW if abs(t["roll"]) < ROLL_CRIT
                   else C_RED))
    draw_kv(surf, "PITCH", f"{t['pitch']:.1f}°", kx, ky + 34,
            col_v=(C_WHITE if abs(t["pitch"]) < PITCH_WARN
                   else C_YELLOW if abs(t["pitch"]) < PITCH_CRIT
                   else C_RED))
    draw_kv(surf, "YAW", f"{t['yaw']:.1f}°", kx + 215, ky)
    draw_kv(surf, "AIRSPD", f"{t['spd_fused']:.1f} km/h", kx + 215, ky + 34,
            col_v=(C_WHITE if t["spd_fused"] < SPD_WARN
                   else C_YELLOW if t["spd_fused"] < SPD_CRIT
                   else C_RED))


def draw_led_indicator(surf, x, y, label, ok):
    """LED quadrato + etichetta, per stato booleano di un sensore/canale."""
    col = C_GREEN if ok else C_RED
    pygame.draw.rect(surf, col, (x, y, 18, 18), border_radius=3)
    pygame.draw.rect(surf, C_BORDER, (x, y, 18, 18), 1, border_radius=3)
    text(surf, label, (x + 26, y + 9), F_LABEL, C_TEXT, anchor="midleft")
    text(surf, "OK" if ok else "FAIL", (x + 26 + 130, y + 9), F_VAL, col, anchor="midleft")


def draw_diag_panel(surf, t):
    """Pannello diagnostica: stato sensori extra, errore rotta, limite termico, flusso ottico."""
    draw_panel(surf, DIAG_PANEL, "DIAGNOSTICA SENSORI & NAV")
    kx = DIAG_PANEL.x + 24
    ky = DIAG_PANEL.y + 42

    draw_led_indicator(surf, kx, ky, "FLUSSO OTTICO", t["sensor_optflow_ok"])
    draw_led_indicator(surf, kx + 225, ky, "LIDAR", t["sensor_lidar_ok"])
    draw_led_indicator(surf, kx + 450, ky, "PACCHETTO RC", not t["sensor_packet_lost"])

    err = abs(t["heading_error"])
    col_err = (C_WHITE if err < HDG_ERR_WARN
               else C_YELLOW if err < HDG_ERR_CRIT
               else C_RED)
    draw_kv(surf, "ERR ROTTA", f"{t['heading_error']:.1f}°", kx, ky + 30, col_v=col_err)

    limitato = t["thermal_limit"] < GAS_MASSIMO
    draw_kv(surf, "LIM TERMICO", f"{t['thermal_limit']} µs", kx + 225, ky + 30,
            col_v=C_YELLOW if limitato else C_WHITE)

    draw_kv(surf, "OPT VX", f"{t['opt_vx']:.2f} m/s", kx + 450, ky + 30)
    draw_kv(surf, "OPT VY", f"{t['opt_vy']:.2f} m/s", kx + 675, ky + 30)




def draw_alert_banner(surf, t):
    """
    Zona dedicata all'alert più importante attualmente attivo (priorità:
    CRITICAL > WARNING > INFO > STATUS). Occupa lo spazio libero tra l'header
    e la prima riga di pannelli, senza spostare né coprire nulla di esistente.
    Cliccabile / attivabile con INVIO per aprire il dettaglio ("PERCHÉ?").
    """
    global _rect_banner
    rect = pygame.Rect(24, 36, 1232, 62)
    _rect_banner = rect

    principale = alert_principale()
    pygame.draw.rect(surf, C_PANEL, rect, border_radius=8)

    if principale is None:
        pygame.draw.rect(surf, C_GREEN, rect, 2, border_radius=8)
        text(surf, "🟢  TUTTI I SISTEMI NOMINALI", rect.center, F_VAL, C_GREEN, anchor="center")
    else:
        col = LIVELLI[principale["livello"]]["colore"]
        icona = LIVELLI[principale["livello"]]["icona"]
        pygame.draw.rect(surf, col, rect, 3, border_radius=8)
        text(surf, f"{icona} {principale['titolo']}", (rect.x + 16, rect.y + 9), F_HEAD, col)
        sotto = principale["causa"] or principale["descrizione"]
        if sotto:
            text(surf, sotto, (rect.x + 16, rect.y + 36), F_LABEL, C_TEXT)

    hint = "[INVIO] dettagli   [D] diagnostica   [L] log   [M] perché MANUALE   [E] telemetria estesa"
    text(surf, hint, (rect.right - 14, rect.y + 8), F_SMALL, C_DIM, anchor="topright")

    conteggi = {}
    for a in alert_attivi.values():
        conteggi[a["livello"]] = conteggi.get(a["livello"], 0) + 1
    if conteggi:
        riepilogo = "   ".join(f"{LIVELLI[l]['icona']} x{n}" for l, n in conteggi.items())
        text(surf, riepilogo, (rect.right - 14, rect.bottom - 10), F_SMALL, C_TEXT, anchor="bottomright")


def _draw_overlay_backdrop(surf, titolo):
    dim = pygame.Surface((W, H), pygame.SRCALPHA)
    dim.fill((5, 7, 12, 235))
    surf.blit(dim, (0, 0))
    rect = pygame.Rect(60, 60, W - 120, H - 120)
    draw_panel(surf, rect, titolo)
    return rect


def draw_diagnostics_overlay(surf):
    """Pannello scrollabile con TUTTI gli alert attivi e i dettagli completi."""
    rect = _draw_overlay_backdrop(surf, "PANNELLO DIAGNOSTICA COMPLETO   —   [D] chiudi   [↑/↓] scorri")
    clip_prec = surf.get_clip()
    contenuto = pygame.Rect(rect.x + 10, rect.y + 36, rect.width - 20, rect.height - 46)
    surf.set_clip(contenuto)

    y = contenuto.y + 6 - _scroll_diag
    items = alert_ordinati()
    if not items:
        text(surf, "🟢 Nessun allarme attivo — tutti i sistemi nominali.", (contenuto.x + 14, y), F_VAL, C_GREEN)
    for codice, a in items:
        col = LIVELLI[a["livello"]]["colore"]
        icona = LIVELLI[a["livello"]]["icona"]
        text(surf, f"{icona} [{a['ts']}] {a['titolo']}", (contenuto.x + 14, y), F_VAL, col)
        riga = []
        if a["causa"]:  riga.append(f"Causa: {a['causa']}")
        if a["soglia"]: riga.append(f"Soglia: {a['soglia']}")
        if riga:
            text(surf, "    " + "   |   ".join(riga), (contenuto.x + 14, y + 20), F_LABEL, C_TEXT)
        if a["valore"]:
            text(surf, f"    Valore attuale: {a['valore']}", (contenuto.x + 14, y + 38), F_LABEL, C_TEXT)
        if a["azione"]:
            text(surf, f"    ➜ Azione consigliata: {a['azione']}", (contenuto.x + 14, y + 56), F_SMALL, C_ACCENT)
        pygame.draw.line(surf, C_BORDER, (contenuto.x, y + 74), (contenuto.right, y + 74), 1)
        y += 84
    surf.set_clip(clip_prec)


def draw_log_overlay(surf):
    """Log eventi scrollabile (i più recenti in cima). Registrato integralmente anche su terminale."""
    rect = _draw_overlay_backdrop(surf, "LOG EVENTI   —   [L] chiudi   [↑/↓] scorri")
    clip_prec = surf.get_clip()
    contenuto = pygame.Rect(rect.x + 10, rect.y + 36, rect.width - 20, rect.height - 46)
    surf.set_clip(contenuto)

    y = contenuto.y + 6 - _scroll_log
    voci = list(event_log)[::-1]
    if not voci:
        text(surf, "Nessun evento registrato.", (contenuto.x + 14, y), F_LABEL, C_DIM)
    for ora, livello, msg in voci:
        col = LIVELLI.get(livello, LIVELLI["INFO"])["colore"]
        text(surf, f"{ora}   {msg}", (contenuto.x + 14, y), F_LABEL, col)
        y += 24
    surf.set_clip(clip_prec)


def draw_perche_manuale_overlay(surf, t):
    """Popup 'PERCHÉ?' dedicato al blocco della modalità MANUALE (comando MODO in main.ino)."""
    rect = _draw_overlay_backdrop(surf, "PERCHÉ?  —  MODALITÀ MANUALE   —   [M] o [ESC] chiudi")
    x = rect.x + 24
    y = rect.y + 50
    motivi = motivi_blocco_manuale(t)
    if not motivi:
        text(surf, "🟢 MANUALE DISPONIBILE", (x, y), F_HEAD, C_GREEN)
        text(surf, "Nessuna condizione di blocco attiva: il comando MODO:1 verrà accettato.", (x, y + 34), F_LABEL, C_TEXT)
    else:
        text(surf, "🔴 MODALITÀ MANUALE BLOCCATA", (x, y), F_HEAD, C_RED)
        y += 40
        text(surf, "Motivi:", (x, y), F_TITLE, C_DIM); y += 26
        for nome, spiegazione in motivi:
            text(surf, f"✖ {nome}", (x + 20, y), F_VAL, C_RED); y += 22
            text(surf, spiegazione, (x + 40, y), F_SMALL, C_TEXT); y += 30
        y += 10
        text(surf, "Condizioni necessarie per sbloccare:", (x, y), F_TITLE, C_DIM); y += 26
        text(surf, "✓ Failsafe OFF (segnale radio ripristinato)", (x + 20, y), F_LABEL, C_TEXT); y += 22
        text(surf, "✓ Schianto OFF (sblocco da radiocomando, canale 5 in basso)", (x + 20, y), F_LABEL, C_TEXT)


def draw_extended_overlay(surf):
    """
    Schermata "TELEMETRIA ESTESA": mostra tutti i dati diagnostici dei
    pacchetti TEL2/TEL3/TEL4 che non compaiono nella PFD principale
    (correnti/potenze, diagnostica PID completa, GPS/baro/pitot/IMU estesi,
    RC 4-16, contatori pacchetto). Aggiornati in round-robin ogni ~2s.
    """
    rect = _draw_overlay_backdrop(surf, "TELEMETRIA ESTESA — TEL2 / TEL3 / TEL4   —   [E] chiudi   [↑/↓] scorri")
    clip_prec = surf.get_clip()
    contenuto = pygame.Rect(rect.x + 10, rect.y + 36, rect.width - 20, rect.height - 46)
    surf.set_clip(contenuto)

    col_w = contenuto.width // 2
    x1 = contenuto.x + 14
    x2 = contenuto.x + col_w + 14
    val_off = 232

    def riga_kv(x, y, lbl, val, col=C_WHITE):
        text(surf, lbl, (x, y), F_LABEL, C_DIM)
        text(surf, val, (x + val_off, y), F_VAL, col)
        return y + 22

    # ── Colonna sinistra: TEL2 (potenza/correnti) + TEL4 (IMU/pitot/baro) ──
    y = contenuto.y + 6 - _scroll_ext
    text(surf, "── CORRENTI / POTENZE (TEL2) ──", (x1, y), F_TITLE, C_ACCENT); y += 26
    for lbl, val in [
        ("Corrente motore",   f"{T2['curr_motor']:.1f} mA"),
        ("Corrente Teensy",   f"{T2['curr_teensy']:.1f} mA"),
        ("Corrente Int SX",   f"{T2['curr_isx']:.1f} mA"),
        ("Corrente Int DX",   f"{T2['curr_idx']:.1f} mA"),
        ("Corrente Est SX",   f"{T2['curr_esx']:.1f} mA"),
        ("Corrente Est DX",   f"{T2['curr_edx']:.1f} mA"),
        ("Potenza motore",    f"{T2['pow_motor']:.1f} mW"),
        ("Potenza Teensy",    f"{T2['pow_teensy']:.1f} mW"),
        ("Gas pre-limite termico", f"{T2['gas_pre_limit']} µs"),
        ("Limitazione termica attiva", "ATTIVA" if T2['thermal_limit_active'] else "no"),
        ("Flusso ottico dx (grezzo)",  f"{T2['flow_dx']}"),
        ("Flusso ottico dy (grezzo)",  f"{T2['flow_dy']}"),
    ]:
        y = riga_kv(x1, y, lbl, val)

    y += 12
    text(surf, "── IMU ESTESA / PITOT / BARO (TEL4) ──", (x1, y), F_TITLE, C_ACCENT); y += 26
    for lbl, val in [
        ("Accel X/Y/Z", f"{T4['accel_x']:.2f}/{T4['accel_y']:.2f}/{T4['accel_z']:.2f} m/s²"),
        ("Accel totale", f"{T4['accel_tot']:.2f} m/s²"),
        ("Giroscopio X/Y/Z", f"{T4['gyro_x']:.2f}/{T4['gyro_y']:.2f}/{T4['gyro_z']:.2f} °/s"),
        ("Cal. IMU (sys/gyro/acc/mag)", f"{T4['imu_cal_sys']}/{T4['imu_cal_gyro']}/{T4['imu_cal_accel']}/{T4['imu_cal_mag']}"),
        ("Offset pitch/roll/yaw", f"{T4['off_pitch']:.2f}/{T4['off_roll']:.2f}/{T4['off_yaw']:.2f}"),
        ("Pitot grezzo (ADC)", f"{T4['pitot_raw']}"),
        ("Pitot zero (taratura)", f"{T4['pitot_zero']:.1f}"),
        ("Pitot differenza", f"{T4['pitot_diff']:.1f}"),
        ("Pitot valido", "sì" if T4['pitot_valid'] else "no"),
        ("Pressione barometrica", f"{T4['baro_pressure']:.1f} Pa"),
        ("Tara altitudine ASL", f"{T4['baro_tare']:.1f} m"),
        ("Barometro pronto", "sì" if T4['baro_ready'] else "no"),
    ]:
        y = riga_kv(x1, y, lbl, val)

    # ── Colonna destra: TEL3 (PID) + GPS esteso + pacchetti/RC ──
    y = contenuto.y + 6 - _scroll_ext
    text(surf, "── DIAGNOSTICA PID COMPLETA (TEL3) ──", (x2, y), F_TITLE, C_ACCENT); y += 26
    for lbl, val in [
        ("Target altitudine", f"{T3['alt_target']:.1f} m"),
        ("Target velocità attuale", f"{T3['vel_target']:.1f} km/h"),
        ("Alt: err/P/I/D", f"{T3['alt_err']:.2f}/{T3['alt_p']:.2f}/{T3['alt_i']:.2f}/{T3['alt_d']:.2f}"),
        ("Target pitch auto", f"{T3['target_pitch_auto']:.2f}°"),
        ("Pitch: err/P/I/D", f"{T3['pitch_err']:.2f}/{T3['pitch_p']:.2f}/{T3['pitch_i']:.2f}/{T3['pitch_d']:.2f}"),
        ("Roll: err/P/I/D", f"{T3['roll_err']:.2f}/{T3['roll_p']:.2f}/{T3['roll_i']:.2f}/{T3['roll_d']:.2f}"),
        ("Vel: err/P/I/D", f"{T3['vel_err']:.2f}/{T3['vel_p']:.2f}/{T3['vel_i']:.2f}/{T3['vel_d']:.2f}"),
    ]:
        y = riga_kv(x2, y, lbl, val)

    y += 12
    text(surf, "── GPS ESTESO (TEL4) ──", (x2, y), F_TITLE, C_ACCENT); y += 26
    for lbl, val in [
        ("Altitudine GPS", f"{T4['gps_alt']:.1f} m" if T4['gps_alt'] >= 0 else "n/d"),
        ("Rotta GPS (course)", f"{T4['gps_course']:.1f}°" if T4['gps_course'] >= 0 else "n/d"),
        ("Fix posizione valido", "sì" if T4['gps_loc_valid'] else "no"),
        ("Velocità GPS valida", "sì" if T4['gps_speed_valid'] else "no"),
        ("Rotta GPS valida", "sì" if T4['gps_course_valid'] else "no"),
    ]:
        y = riga_kv(x2, y, lbl, val)

    y += 12
    text(surf, "── PACCHETTI TEL1 / RC ESTESO ──", (x2, y), F_TITLE, C_ACCENT); y += 26
    y = riga_kv(x2, y, "Timestamp firmware", f"{T['fw_millis']} ms")
    y = riga_kv(x2, y, "N. pacchetto TEL1", f"{T['tel1_packet_num']}")
    y = riga_kv(x2, y, "Pacchetti TEL1 persi (stimati)", f"{T['tel1_lost_count']}",
                col=C_RED if T['tel1_lost_count'] > 0 else C_GREEN)

    if T4["rc_ext"]:
        text(surf, "RC canali 4-16 (µs):", (x2, y), F_LABEL, C_DIM); y += 20
        riga_txt = ""
        for i, v in enumerate(T4["rc_ext"]):
            pezzo = f"CH{i+4}:{v}  "
            if len(riga_txt) + len(pezzo) > 44:
                text(surf, riga_txt, (x2 + 10, y), F_SMALL, C_WHITE); y += 18
                riga_txt = ""
            riga_txt += pezzo
        if riga_txt:
            text(surf, riga_txt, (x2 + 10, y), F_SMALL, C_WHITE); y += 18

    surf.set_clip(clip_prec)


def riproduci_audio(nome_file: str) -> bool:
    percorso = os.path.join(CARTELLA_AUDIO, nome_file)
    if not os.path.exists(percorso):
        print(f"[AUDIO] Mancante: {percorso}")
        return False
    if pygame.mixer.music.get_busy():
        return False
    try:
        pygame.mixer.music.load(percorso)
        pygame.mixer.music.play()
        print(f"[AUDIO] ▶ {nome_file}")
        return True
    except Exception as e:
        print(f"[AUDIO] Errore {nome_file}: {e}")
        return False


_w_timers: dict = {}


def warning(nome: str, cooldown_ms: int = 3000) -> bool:
    ora = pygame.time.get_ticks()
    if ora - _w_timers.get(nome, 0) >= cooldown_ms:
        if riproduci_audio(nome):
            _w_timers[nome] = ora
            return True
    return False


# ── Stato interno warning ─────────────────────────────────────
_w_alt_prec: float = 0.0
_w_t_prec_ms: int = 0
_w_callouts: set = set()
_w_mode_prec: object = None


def aggiorna_warning(t: dict) -> None:
    global _w_alt_prec, _w_t_prec_ms, _w_callouts, _w_mode_prec

    # ── Delta-tempo reale ──
    ora_ms = pygame.time.get_ticks()
    dt_ms = max(ora_ms - _w_t_prec_ms, 1) if _w_t_prec_ms else 33
    _w_t_prec_ms = ora_ms
    dt_s = dt_ms / 1000.0

    alt = t["altitude"]
    spd = t["spd_fused"]
    roll = abs(t["roll"])
    in_volo = t["in_flight"]
    mode = t["mode"]

    # ── Velocità di discesa ──
    if in_volo:
        v_discesa = (alt - _w_alt_prec) / dt_s
        _w_alt_prec = alt
    else:
        v_discesa = 0.0
        _w_alt_prec = alt

    if alt > ALT_RESET_CALLOUT or not in_volo:
        _w_callouts.clear()

    if in_volo:
        if alt < ALT_PULL_UP and v_discesa <= VDISCESA_PULL_UP:
            warning("pull_up.wav", cooldown_ms=CD_PULL_UP)
        elif alt < ALT_TERRAIN_PU and v_discesa <= VDISCESA_TERRAIN_PU:
            warning("tarrain_terrain_pull_up.wav", cooldown_ms=CD_TERRAIN_PULL_UP)
        elif alt < ALT_TERRAIN and v_discesa <= VDISCESA_TERRAIN:
            warning("terrain.wav", cooldown_ms=CD_TERRAIN)

        if v_discesa <= VDISCESA_SINK_RATE:
            warning("sink_rate.wav", cooldown_ms=CD_SINK_RATE)

        if alt < ALT_DONT_SINK and v_discesa <= VDISCESA_DONT_SINK:
            warning("dont sink.wav", cooldown_ms=CD_DONT_SINK)

        if roll >= ROLL_CRIT:
            warning("bank_angle.wav", cooldown_ms=CD_BANK_ANGLE)

        if spd < SPD_STALL:
            warning("stall.wav", cooldown_ms=CD_STALL)
        elif spd < SPD_AIR_LOW:
            warning("air_speed_low.wav", cooldown_ms=CD_AIRSPEED_LOW)
        elif spd < SPD_SLOW:
            warning("fligh slow.wav", cooldown_ms=CD_FLIGHT_SLOW)

        if spd >= SPD_CRIT:
            warning("overspeed.wav", cooldown_ms=CD_OVERSPEED)

        # Callout quota
        if v_discesa <= VDISCESA_CALLOUT:
            if "40" not in _w_callouts and CALLOUT_40_LO < alt < CALLOUT_40_HI:
                _w_callouts.add("40"); riproduci_audio("40.wav")
            elif "30" not in _w_callouts and CALLOUT_30_LO < alt < CALLOUT_30_HI:
                _w_callouts.add("30"); riproduci_audio("30.wav")
            elif "20" not in _w_callouts and CALLOUT_20_LO < alt < CALLOUT_20_HI:
                _w_callouts.add("20"); riproduci_audio("20.wav")
            elif "10" not in _w_callouts and CALLOUT_10_LO < alt < CALLOUT_10_HI:
                _w_callouts.add("10"); riproduci_audio("10.wav")

    if _w_mode_prec is None:
        _w_mode_prec = mode
    elif mode != _w_mode_prec:
        if mode in ("MANUALE", "AUTO PID"):
            riproduci_audio("autopilot.wav")
        _w_mode_prec = mode

    if t["alarm_failsafe"]:
        warning("errore.wav", cooldown_ms=CD_ERRORE)

    if t["alarm_crash"]:
        warning("errore.wav", cooldown_ms=CD_ERRORE)

    if t["alarm_batt_motor"]:
        warning("errore.wav", cooldown_ms=CD_ERRORE)

    if t["alarm_batt_teensy"]:
        warning("errore.wav", cooldown_ms=CD_ERRORE)

    if t["alarm_relay"]:
        warning("errore.wav", cooldown_ms=CD_ERRORE)

    if t["t_teensy"] >= T_TEENSY_CRIT:
        warning("errore.wav", cooldown_ms=CD_ERRORE)

    if t["t_motor"] >= T_MOTOR_CRIT:
        warning("errore.wav", cooldown_ms=CD_ERRORE)

    if not (t["ok_isx"] and t["ok_idx"] and t["ok_esx"] and t["ok_edx"]):
        warning("errore.wav", cooldown_ms=CD_ERRORE)


def terminal_cli_thread():
    help_text = """
    =========================================================
    COMANDI DISPONIBILI (Scrivi nel formato CMD:CAMPO:VALORE)
    =========================================================
    -- SERVI (Valori 45-135) --
    SERVO_ISX / SERVO_IDX / SERVO_ESX / SERVO_EDX : <angolo>

    -- STATO SERVI (Nessun valore necessario) --
    SERVO_ISX_ATTACH / SERVO_ISX_DETACH (ecc. per gli altri)

    -- CONTROLLI --
    RELE_ON / RELE_OFF
    MODO : <1 (Manuale) o 2 (Auto)>
    GAS  : <1000 - 2000>

    -- SICUREZZE --
    SICUREZZA_SCHIANTO_ON / OFF
    SICUREZZA_ALIMENTAZIONE_ON / OFF
    SICUREZZA_SERVI_ON / OFF
    SICUREZZA_TEMP_ON / OFF

    -- NAVIGAZIONE --
    SET_LATITUDE  : <float>
    SET_LONGITUDE : <float>
    SET_ALTITUDE  : <float>
    =========================================================
    Digita 'HELP' per rivedere questa lista.
    """
    print(help_text)

    while True:
        try:
            raw_input = input()
            if not raw_input.strip():
                continue

            if raw_input.strip().upper() == 'HELP':
                print(help_text)
                continue

            # Pulizia e normalizzazione dell'input:
            # "cMd   :   SET_ALTITUDE  : 200 " -> ["CMD", "SET_ALTITUDE", "200"]
            parts = [p.strip().upper() for p in raw_input.split(':')]

            if parts[0] == "CMD":
                campo = parts[1] if len(parts) > 1 else ""

                # Se l'utente non inserisce un valore (es. "CMD:RELE_ON"),
                # passiamo "0" di default per non far fallire il parsing sul Teensy
                valore = parts[2] if len(parts) > 2 and parts[2] != "" else "0"

                if campo:
                    send_command(campo, valore)
            else:
                print("[CLI] Formato errato. Usa CMD:CAMPO:VALORE (es. CMD:SET_ALTITUDE:200)")

        except Exception as e:
            print(f"[CLI] Errore di input: {e}")


def main():
    global _overlay_diagnostica, _overlay_log, _overlay_perche, _overlay_extended
    global _scroll_diag, _scroll_log, _scroll_ext

    thread_seriale = threading.Thread(target=read_from_serial, daemon=True)
    thread_seriale.start()

    # Thread per l'invio comandi da terminale
    thread_cli = threading.Thread(target=terminal_cli_thread, daemon=True)
    thread_cli.start()

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_d:
                    _overlay_diagnostica = not _overlay_diagnostica
                    _overlay_log = _overlay_perche = _overlay_extended = False
                elif event.key == pygame.K_l:
                    _overlay_log = not _overlay_log
                    _overlay_diagnostica = _overlay_perche = _overlay_extended = False
                elif event.key == pygame.K_m:
                    _overlay_perche = not _overlay_perche
                    _overlay_diagnostica = _overlay_log = _overlay_extended = False
                elif event.key == pygame.K_e:
                    _overlay_extended = not _overlay_extended
                    _overlay_diagnostica = _overlay_log = _overlay_perche = False
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_SPACE):
                    if not (_overlay_diagnostica or _overlay_log or _overlay_perche or _overlay_extended):
                        _overlay_diagnostica = True
                elif event.key == pygame.K_ESCAPE:
                    _overlay_diagnostica = _overlay_log = _overlay_perche = _overlay_extended = False
                elif event.key == pygame.K_UP:
                    _scroll_diag = max(0, _scroll_diag - 84)
                    _scroll_log = max(0, _scroll_log - 48)
                    _scroll_ext = max(0, _scroll_ext - 84)
                elif event.key == pygame.K_DOWN:
                    _scroll_diag += 84
                    _scroll_log += 48
                    _scroll_ext += 84

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if _rect_banner.collidepoint(event.pos):
                    _overlay_diagnostica = True
                    _overlay_log = _overlay_perche = _overlay_extended = False

        # ── Valutazione diagnostica (analizza la telemetria ricevuta da main.ino) ──
        evaluate_diagnostics(T)
        aggiorna_warning(T)

        screen.fill(C_BG)
        draw_header(screen, T)
        draw_alert_banner(screen, T)
        draw_power_panel(screen, T)
        draw_temp_panel(screen, T)
        draw_speed_panel(screen, T)
        draw_servi_panel(screen, T)
        draw_vserv_panel(screen, T)
        draw_pfd_center(screen, T)
        draw_nav_panel(screen, T)
        draw_orientation_panel(screen, T)
        draw_battery_panel(screen, T)
        draw_diag_panel(screen, T)

        # ── Overlay (finestre di dettaglio): disegnati per ultimi, sopra a tutto ──
        if _overlay_diagnostica:
            draw_diagnostics_overlay(screen)
        elif _overlay_log:
            draw_log_overlay(screen)
        elif _overlay_perche:
            draw_perche_manuale_overlay(screen, T)
        elif _overlay_extended:
            draw_extended_overlay(screen)

        pygame.display.flip()
        clock.tick(30)


if __name__ == "__main__":
    main()