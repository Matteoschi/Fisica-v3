
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

W, H   = 1280, 900
screen = pygame.display.set_mode((W, H))
pygame.display.set_caption("GCS – Primary Flight Display")
clock  = pygame.time.Clock()

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
C_ORANGE = (255, 140,   0)  # diagnostica: livello WARNING (🟠)
C_RED    = (218,  48,  48)  # allarme / errore
C_SKY    = ( 28,  96, 178)  # cielo orizzonte
C_GROUND = (124,  80,  30)  # terra orizzonte
C_CROSS  = (255, 210,   0)  # crosshair orizzonte
C_GRAPH_BARO   = (  0, 188, 255)  # linea grafico: baro
C_GRAPH_LIDAR  = ( 40, 210,  88)  # linea grafico: lidar
C_GRAPH_FUSA   = (255, 200,   0)  # linea grafico: quota fusa
F_HEAD  = pygame.font.SysFont("consolas", 21, bold=True)  # header principale
F_TITLE = pygame.font.SysFont("consolas", 13, bold=True)  # titoli pannelli
F_VAL   = pygame.font.SysFont("consolas", 15, bold=True)  # valori numerici
F_LABEL = pygame.font.SysFont("consolas", 13)             # etichette
F_SMALL = pygame.font.SysFont("consolas", 12)             # testo piccolo



# Cooldown audio (ms) 
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

# ── Roll 
ROLL_WARN   = 30     # GIALLO in HUD
ROLL_CRIT   = 35     # ROSSO  in HUD + "bank_angle.wav"

# ── Pitch 
PITCH_WARN  = 15     # GIALLO
PITCH_CRIT  = 20     # ROSSO  in HUD + "pitch.wav"

# ── Velocità (km/h) 
SPD_STALL   = 22     # < soglia → "stall.wav" + ROSSO
SPD_AIR_LOW = 35     # < soglia → "air_speed_low.wav"+ GIALLO
SPD_SLOW    = 45     # < soglia → "fligh slow.wav"
SPD_WARN    = 70     # >= soglia → GIALLO in HUD
SPD_CRIT    = 90     # >= soglia → ROSSO  in HUD + "overspeed.wav"

# ── Discesa (m/s, negativo = scende) 
VDISCESA_PULL_UP    = -1.0   # + alt < ALT_PULL_UP   → pull_up
VDISCESA_TERRAIN_PU = -0.5   # + alt < ALT_TERRAIN_PU → terrain pull up
VDISCESA_TERRAIN    = -0.3   # + alt < ALT_TERRAIN    → terrain
VDISCESA_DONT_SINK  = -0.3   # + alt < ALT_DONT_SINK  → dont sink
VDISCESA_SINK_RATE  = -4.0   # qualsiasi quota        → sink rate
VDISCESA_CALLOUT    = -0.2   # attiva callout quota

# ── Quote (m) 
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

# ── Temperatura (°C)
T_TEENSY_WARN  = 60   # >= soglia → GIALLO
T_TEENSY_CRIT  = 65   # >= soglia → ROSSO + "errore.wav"

T_MOTOR_WARN   = 80   # >= soglia → GIALLO
T_MOTOR_CRIT   = 85   # >= soglia → ROSSO + "errore.wav"

# ── Tensione servo (V) 
SERVO_V_MIN = 4.5    # fuori range → servo ERROR
SERVO_V_MAX = 6.0

# ── Scale barre batteria (V) 
VBAR_TEENSY_MIN = 4.0
VBAR_TEENSY_MAX = 6.5

VBAR_MOTOR_MIN  = 12.0
VBAR_MOTOR_MAX  = 16.8

# ── Distanza target (m)
DIST_TGT_WARN = 150   # < soglia → GIALLO

# ── Errore rotta (°)
HDG_ERR_WARN = 30     # >= soglia (assoluto) → GIALLO
HDG_ERR_CRIT = 90     # >= soglia (assoluto) → ROSSO

# ── Storico grafici (numero campioni telemetria conservati)
HIST_LEN = 120

# ── Satelliti GPS 
SAT_MIN = 5           # < soglia → ROSSO

# ── Throttle 
THR_WARN = 0.8        # >= soglia → barra GIALLA

GAS_MINIMO  = 1000
GAS_MASSIMO = 2000   

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

# ── Nuova riga in basso: diagnostica sensori/navigazione avanzata + grafico quote
DIAG_PANEL      = pygame.Rect(24, 712, 1232, 100)
ALT_GRAPH_PANEL = pygame.Rect(644, 712, 612, 172)

# ── Colonne servi nel pannello SERVI
SERVI_X   = SERVI_PANEL.x + 28
SERVI_Y   = SERVI_PANEL.y + 52
SERVI_GAP = 88


_serial_lock    = threading.Lock()
_ser_instance   = None


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
}

# Storici per i grafici (aggiornati ad ogni pacchetto telemetria valido)
HIST_BARO  = deque(maxlen=HIST_LEN)
HIST_LIDAR = deque(maxlen=HIST_LEN)
HIST_FUSA  = deque(maxlen=HIST_LEN)

# ================================================================
#  SISTEMA DIAGNOSTICO CENTRALIZZATO
#  (analisi soglie/failsafe derivata da main.ino — vedi riepilogo
#   in fondo al file per la mappatura completa condizione → alert)
# ================================================================
import time as _time_mod
from datetime import datetime as _dt

# Soglie duplicate da main.ino, usate SOLO per la spiegazione al pilota
# (il firmware resta l'unica fonte di verità per le decisioni di volo)
FW_VALORE_BATT_MOTORE_BASSA   = 11.8   # V   (VALORE_BATT_MOTORE_BASSA)
FW_VALORE_BATT_TEENSY_BASSA   = 4.9    # V   (VALORE_BATT_TEENSY_BASSA)
FW_T_MOTORE_THROTTLE_START    = 70.0   # °C  (T_MOTORE_THROTTLE_START)
FW_T_MOTORE_THROTTLE_END      = 90.0   # °C  (T_MOTORE_THROTTLE_END)
FW_GAS_MASSIMO                = 2000   # µs  (GAS_MASSIMO)
FW_GAS_MINIMO                 = 1200   # µs  (GAS_MINIMO)
FW_ALTEZZA_MAX_LIDAR          = 6.0    # m   (ALTEZZA_MAX_LIDAR)
FW_ALTEZZA_MAX_SENSORE_OTTICO = 4.0    # m   (ALTEZZA_MAX_SENSORE_OTTICO)
FW_SAT_MIN_FIX                = 5      # satelliti minimi per un fix affidabile (soglia GUI, SAT_MIN)
FW_PITOT_DIFF_ANOMALIA_KMH    = 25.0   # soglia derivata GUI: divergenza pitot/GPS sospetta
FW_PITOT_DURATA_ANOMALIA_S    = 3.0    # secondi di persistenza richiesti prima di segnalare

LIVELLI = {
    "CRITICAL": {"icona": "🔴", "colore": C_RED,    "prio": 0, "tag": "CRITICAL"},
    "WARNING":  {"icona": "🟠", "colore": C_ORANGE, "prio": 1, "tag": "WARNING"},
    "INFO":     {"icona": "🟡", "colore": C_YELLOW, "prio": 2, "tag": "INFO"},
    "STATUS":   {"icona": "🔵", "colore": C_ACCENT, "prio": 3, "tag": "STATUS"},
    "OK":       {"icona": "🟢", "colore": C_GREEN,  "prio": 4, "tag": "OK"},
}

_alert_lock  = threading.Lock()
alert_attivi = {}                 # codice -> dict con i dettagli dell'alert attivo
event_log    = deque(maxlen=300)  # (ora_str, livello, testo) — mostrato in GUI (ultimi N) e conservato per lo scroll

def _ora():
    return _dt.now().strftime("%H:%M:%S")

def add_alert(codice, livello, titolo, descrizione="", causa="", valore="", soglia="",
              azione="", durata=None, codice_errore=None, dettaglio_terminale=None):
    """
    Sistema centralizzato di notifiche diagnostiche.
    Attiva/aggiorna un alert identificato da 'codice'. La GENERAZIONE di un evento
    nel log e sul terminale avviene SOLO alla transizione inattivo -> attivo, per
    evitare di duplicare lo stesso messaggio ad ogni frame; i valori numerici
    dell'alert vengono comunque aggiornati in tempo reale per la GUI.
    """
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
        if descrizione:     print(f"    {descrizione}")
        if causa:            print(f"    Causa: {causa}")
        if valore:           print(f"    Valore attuale: {valore}")
        if soglia:           print(f"    Soglia: {soglia}")
        if azione:           print(f"    Azione consigliata: {azione}")
        if codice_errore:    print(f"    Codice errore: {codice_errore}")
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
_rect_banner        = pygame.Rect(0, 0, 0, 0)
_overlay_diagnostica = False
_overlay_log         = False
_overlay_perche      = False
_scroll_diag         = 0
_scroll_log          = 0

# ── Stato interno per diagnostiche derivate (non presenti direttamente
#    nella telemetria, calcolate qui a partire dai dati ricevuti) ────────
_pitot_anomalia_dal = None      # timestamp (ms) di inizio divergenza pitot/GPS persistente
_relay_prec         = False


def evaluate_diagnostics(t):
    """
    Valuta ad ogni frame lo stato ricevuto da main.ino e aggiorna gli alert attivi.
    Ogni condizione qui sotto è ricavata da una soglia/decisione REALMENTE presente
    nel firmware (vedi commenti); le diagnostiche "derivate" (es. Pitot) sono
    esplicitamente etichettate come tali e non dichiarano un guasto hardware
    certo se il firmware non è in grado di rilevarlo direttamente.
    """
    global _pitot_anomalia_dal, _relay_prec

    # ── 1. FAILSAFE (bit0 codiceAllarme, main.ino: ricevente.read(...,&failsafe,...)) ──
    if t["alarm_failsafe"]:
        add_alert("failsafe", "CRITICAL", "FAILSAFE ATTIVO",
                   causa="Perdita del segnale radio (pacchetto SBUS non valido)",
                   valore="Modalità forzata: FAILSAFE (controllo automatico verso il target)",
                   azione="Verificare trasmettitore/ricevente e ripristinare il collegamento radio.")
    else:
        clear_alert("failsafe", "FAILSAFE DISATTIVATO")

    # ── 2. SCHIANTO (bit4, gestisciSchianto(): accelerazione IMU > SOGLIA_G_SCHIANTO) ──
    if t["alarm_crash"]:
        add_alert("crash", "CRITICAL", "SCHIANTO RILEVATO",
                   causa="Accelerazione IMU oltre la soglia di impatto per 3 campioni consecutivi",
                   valore="Gas: NEUTRO forzato — servi interni staccati",
                   azione="Sblocco possibile SOLO da radiocomando (canale 5 riportato in basso).")
    else:
        clear_alert("crash", "SBLOCCO EMERGENZA ESEGUITO")

    # ── 3. PROTEZIONE TERMICA MOTORE (gasMaxTermico(), soglie 70→90°C) ──
    if t["thermal_limit"] < FW_GAS_MASSIMO:
        pct_limite = round((t["thermal_limit"] - FW_GAS_MINIMO) / (FW_GAS_MASSIMO - FW_GAS_MINIMO) * 100)
        add_alert("motore_termico", "WARNING", "POTENZA MOTORE LIMITATA",
                   causa="PROTEZIONE TERMICA ATTIVA",
                   valore=f"Temperatura: {t['t_motor']:.1f} °C  |  Gas erogato: {t['pid_gas']*100:.0f} %",
                   soglia=f"Inizio limitazione: {FW_T_MOTORE_THROTTLE_START:.0f} °C — Limite massimo: {FW_T_MOTORE_THROTTLE_END:.0f} °C",
                   azione="Ridurre il regime motore o attendere il raffreddamento. "
                          f"Gas massimo consentito ora: {t['thermal_limit']} µs (~{pct_limite}%).")
    else:
        clear_alert("motore_termico", "POTENZA MOTORE RIPRISTINATA")

    # ── 4. BATTERIA MOTORE (bit1, VALORE_BATT_MOTORE_BASSA = 11.8 V) ──
    if t["alarm_batt_motor"]:
        add_alert("batt_motore", "WARNING", "BATTERIA MOTORE BASSA",
                   causa="Tensione sotto la soglia minima operativa",
                   valore=f"Tensione: {t['v_motor']:.2f} V",
                   soglia=f"{FW_VALORE_BATT_MOTORE_BASSA:.2f} V",
                   azione="Atterrare e sostituire/ricaricare la batteria di potenza.")
    else:
        clear_alert("batt_motore", "BATTERIA MOTORE OK")

    # ── 5. BATTERIA TEENSY / AVIONICA (bit3, VALORE_BATT_TEENSY_BASSA = 4.9 V) ──
    if t["alarm_batt_teensy"]:
        add_alert("batt_teensy", "WARNING", "BATTERIA AVIONICA (TEENSY) BASSA",
                   causa="Tensione sotto la soglia minima operativa",
                   valore=f"Tensione: {t['v_teensy']:.2f} V  |  Failover relè: {'ATTIVO' if t['relay'] else 'in attesa'}",
                   soglia=f"{FW_VALORE_BATT_TEENSY_BASSA:.2f} V",
                   azione="Verificare BEC/regolatore avionica; il relè commuta automaticamente sulla batteria motore.")
    else:
        clear_alert("batt_teensy", "BATTERIA AVIONICA OK")

    # ── 6. FAILOVER RELÈ (transizione OFF->ON di 'relay', bit2/campo 35) ──
    if t["relay"] and not _relay_prec:
        add_alert("relay_on", "INFO", "FAILOVER ALIMENTAZIONE ATTIVATO",
                   causa="Batteria avionica (Teensy) sotto soglia",
                   valore="Relè commutato: l'avionica è ora alimentata dalla batteria motore",
                   azione="Pianificare l'atterraggio per sostituire la batteria avionica.")
    elif not t["relay"] and _relay_prec:
        clear_alert("relay_on", "FAILOVER ALIMENTAZIONE DISATTIVATO")
    _relay_prec = t["relay"]

    # ── 7. SERVI IN ANOMALIA DI CORRENTE (diagnosticaServi(), soglie 0.5–2500 mA) ──
    servi_ko = [n for n, ok in (("Int SX", t["ok_isx"]), ("Int DX", t["ok_idx"]),
                                 ("Est SX", t["ok_esx"]), ("Est DX", t["ok_edx"])) if not ok]
    if servi_ko:
        add_alert("servi_anomalia", "WARNING" if len(servi_ko) < 4 else "CRITICAL",
                   "ANOMALIA CORRENTE SERVO",
                   causa="Corrente fuori range (0.5–2500 mA) per oltre 5 letture consecutive",
                   valore="Servi in anomalia: " + ", ".join(servi_ko),
                   azione="Il mixer si è riconfigurato automaticamente sui servi rimanenti; "
                          "verificare cablaggio/meccanica dei servi indicati appena possibile.")
    else:
        clear_alert("servi_anomalia", "SERVI RIPRISTINATI")

    # ── 8. GPS (satelliti/fix — campo 32 satellites, 0 se fix non valido) ──
    if t["satellites"] == 0:
        add_alert("gps_fix", "WARNING", "GPS NON DISPONIBILE",
                   causa="Nessun fix GPS valido",
                   valore="Satelliti: 0",
                   azione="Attendere l'acquisizione satellitare; la navigazione automatica non è affidabile.")
    elif t["satellites"] < FW_SAT_MIN_FIX:
        add_alert("gps_fix", "INFO", "GPS SEGNALE DEBOLE",
                   causa="Numero di satelliti sotto il margine di sicurezza consigliato",
                   valore=f"Satelliti: {t['satellites']}", soglia=f"{FW_SAT_MIN_FIX}",
                   azione="Nessuna azione immediata richiesta; monitorare la qualità del fix.")
    else:
        clear_alert("gps_fix", "GPS FIX ACQUISITO")

    # ── 9. LIDAR — distinzione "non usato per range" vs "guasto" (aggiornaLidar()) ──
    if not t["sensor_lidar_ok"]:
        add_alert("lidar", "WARNING", "LIDAR NON DISPONIBILE",
                   causa="Inizializzazione del sensore non riuscita in fase di avvio",
                   azione="Il sistema prosegue con il solo barometro per la quota; verificare il TF-Luna al prossimo atterraggio.")
    elif t["alt_baro_raw"] > FW_ALTEZZA_MAX_LIDAR:
        add_alert("lidar", "STATUS", "LIDAR NON UTILIZZATO",
                   causa="Quota fuori dall'intervallo operativo del sensore (normale in crociera)",
                   valore=f"Quota: {t['alt_baro_raw']:.1f} m", soglia=f"{FW_ALTEZZA_MAX_LIDAR:.0f} m",
                   azione="Nessuna azione: condizione attesa, non è un errore.")
    else:
        clear_alert("lidar")

    # ── 10. FLUSSO OTTICO — stessa distinzione (velocità_flusso_ottico()) ──
    if not t["sensor_optflow_ok"]:
        add_alert("optflow", "WARNING", "FLUSSO OTTICO NON DISPONIBILE",
                   causa="Inizializzazione del sensore PMW3901 non riuscita in fase di avvio",
                   azione="Il sistema prosegue senza questa sorgente di velocità; verificare i cavi SPI al prossimo atterraggio.")
    elif t["opt_vx"] < 0.0 and t["opt_vy"] < 0.0 and t["altitude"] > FW_ALTEZZA_MAX_SENSORE_OTTICO:
        add_alert("optflow", "STATUS", "FLUSSO OTTICO NON UTILIZZATO",
                   causa="Quota fuori dall'intervallo operativo del sensore (normale sopra i 4 m)",
                   valore=f"Quota: {t['altitude']:.1f} m", soglia=f"{FW_ALTEZZA_MAX_SENSORE_OTTICO:.0f} m",
                   azione="Nessuna azione: condizione attesa, non è un errore.")
    else:
        clear_alert("optflow")

    # ── 11. ANOMALIA PITOT (DIAGNOSTICA DERIVATA — main.ino non ha un check esplicito) ──
    diff = abs(t["spd_pitot"] - t["spd_gps"])
    if t["in_flight"] and diff >= FW_PITOT_DIFF_ANOMALIA_KMH:
        if _pitot_anomalia_dal is None:
            _pitot_anomalia_dal = pygame.time.get_ticks()
        durata_s = (pygame.time.get_ticks() - _pitot_anomalia_dal) / 1000.0
        if durata_s >= FW_PITOT_DURATA_ANOMALIA_S:
            add_alert("pitot_anomalia", "WARNING", "POSSIBILE ANOMALIA PITOT",
                       causa="Divergenza persistente tra velocità Pitot e velocità GPS (diagnostica derivata, non un check firmware)",
                       valore=f"Pitot: {t['spd_pitot']:.1f} km/h  |  GPS: {t['spd_gps']:.1f} km/h  |  Differenza: {diff:.1f} km/h",
                       soglia=f"{FW_PITOT_DIFF_ANOMALIA_KMH:.0f} km/h per oltre {FW_PITOT_DURATA_ANOMALIA_S:.0f} s",
                       azione="Possibili cause: Pitot ostruito, perdita di pressione, errore del sensore. Verificare a terra.",
                       dettaglio_terminale=f"[DIAGNOSTIC][PITOT]\nPitot: {t['spd_pitot']:.1f} km/h\nGPS: {t['spd_gps']:.1f} km/h\n"
                                            f"Differenza: {diff:.1f} km/h\nDurata anomalia: {durata_s:.1f} s")
    else:
        _pitot_anomalia_dal = None
        clear_alert("pitot_anomalia", "VELOCITÀ PITOT/GPS COERENTI")


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
      36 : Vel X flusso ottico  [m/s]
      37 : Vel Y flusso ottico  [m/s]
      38 : Stato sensori        bitmask (b0=flusso ottico OK, b1=LIDAR OK, b2=pacchetto SBUS perso)
      39 : Errore rotta         [°]
      40 : Limite gas termico   [µs]
      41 : Altitudine LIDAR grezza [m]
      42 : Altitudine Baro grezza   [m]
    """
    global T
    try:
        f = line.strip().split(',')
        if len(f) < 43 or f[0] != '$':
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
        T["pid_gas"] = clamp((float(f[25]) - GAS_MINIMO) / (GAS_MASSIMO - GAS_MINIMO), 0.0, 1.0)
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

        # ── Flusso ottico (velocità stimata) ──────────────────────────────
        T["opt_vx"] = float(f[36])
        T["opt_vy"] = float(f[37])

        # ── Stato sensori (bitmask) ────────────────────────────────────────
        stato_sensori = int(f[38])
        T["sensor_optflow_ok"]  = bool(stato_sensori & 1)
        T["sensor_lidar_ok"]    = bool(stato_sensori & 2)
        T["sensor_packet_lost"] = bool(stato_sensori & 4)

        # ── Navigazione avanzata ───────────────────────────────────────────
        T["heading_error"] = float(f[39])

        # ── Protezione termica motore ───────────────────────────────────────
        T["thermal_limit"] = int(f[40])

        # ── Altitudini grezze (diagnostica sensori) ─────────────────────────
        T["alt_lidar_raw"] = float(f[41])
        T["alt_baro_raw"]  = float(f[42])

        HIST_BARO.append(T["alt_baro_raw"])
        HIST_LIDAR.append(T["alt_lidar_raw"])
        HIST_FUSA.append(T["altitude"])

    except Exception as e:
        print(f"[ERR] Pacchetto corrotto saltato: {e}")

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
    kx    = POWER_PANEL.x + 175
    ky    = POWER_PANEL.y + 40
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
            col_v=(C_WHITE  if t["t_teensy"] < T_TEENSY_WARN
                   else C_YELLOW if t["t_teensy"] < T_TEENSY_CRIT
                   else C_RED))
    draw_kv(surf, "MOTORE", f"{t['t_motor']:.1f} °C", kx, ky + 34,
            col_v=(C_WHITE  if t["t_motor"] < T_MOTOR_WARN
                   else C_YELLOW if t["t_motor"] < T_MOTOR_CRIT
                   else C_RED))


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

def servo_ok(v):
    return SERVO_V_MIN < v < SERVO_V_MAX


def draw_servi_panel(surf, t):
    draw_panel(surf, SERVI_PANEL, "SERVI")
    servos = [(t["deg_isx"], "Int SX", t["v_isx"]),(t["deg_idx"], "Int DX", t["v_idx"]),(t["deg_esx"], "Est SX", t["v_esx"]),(t["deg_edx"], "Est DX", t["v_edx"])]

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
        ok  = servo_ok(val)
        col = C_GREEN if ok else C_RED
        r   = pygame.Rect(bx, by, bw, bh)
        draw_vbar(surf, r, val, SERVO_V_MIN - 0.5, SERVO_V_MAX + 0.5, col)
        text(surf, label, (r.centerx, r.top - 14), F_SMALL, C_DIM, anchor="center")
        bx += bw + gap

def draw_pfd_center(surf, t):
    draw_horizon(surf, PFD_CX, PFD_CY, PFD_R, t["pitch"], t["roll"])
    draw_hud_box(surf, SPD_BOX, t["spd_ms"],   "SPEED", "m/s")
    draw_hud_box(surf, ALT_BOX, t["altitude"], "ALT",   "m")
    text(surf, f"TARGET  {t['dist_target']:.0f} m",
         (PFD_CX, PFD_CY + PFD_R + 26), F_LABEL, C_DIM, anchor="midtop")
    
def navigation_info_pannel(surf, t):
    draw_panel(surf, NAV_PANEL, "NAV INFO")
    kx = NAV_PANEL.x + 20
    ky = NAV_PANEL.y + 40
    draw_kv(surf, "DIST TGT",   f"{t['dist_target']:.1f} m", kx, ky,
            col_v=C_WHITE if t["dist_target"] >= DIST_TGT_WARN else C_YELLOW)
    draw_kv(surf, "HDG TGT",    f"{t['hdg_target']:.1f}°",   kx, ky + 34)
    draw_kv(surf, "ROLL TGT",   f"{t['roll_target']:.1f}°",  kx, ky + 68)
    draw_kv(surf, "SATELLITES", f"{t['satellites']}",         kx + 215, ky,
            col_v=C_WHITE if t["satellites"] >= SAT_MIN else C_RED)
    draw_kv(surf, "LAT",        f"{t['lat']:.6f}",            kx + 215, ky + 34)
    draw_kv(surf, "LON",        f"{t['lon']:.6f}",            kx + 215, ky + 68)

def battery_pannel(surf, t):
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
    pipe_h  = 14
    pipe_y  = ky + bar_h // 2 - pipe_h // 2
    pipe_x1 = V_t.right
    pygame.draw.rect(surf, C_BORDER, (pipe_x1, pipe_y,   gap, pipe_h))
    pygame.draw.rect(surf, C_VUOTO,  (pipe_x1, pipe_y+2, gap, pipe_h-4))
    if rele_on:
        pygame.draw.rect(surf, C_GREEN, (pipe_x1, pipe_y+2, gap, pipe_h-4))

    vw, vh = 32, 32
    vx = pipe_x1 + gap // 2 - vw // 2
    vy = ky + bar_h // 2 - vh // 2
    valve_color = C_GREEN if rele_on else C_RED
    pygame.draw.rect(surf, (14, 20, 32), (vx, vy, vw, vh), border_radius=4)
    pygame.draw.rect(surf, valve_color,  (vx, vy, vw, vh), 2, border_radius=4)
    if rele_on:
        pygame.draw.rect(surf, C_GREEN, (vx + 4, vy + vh // 2 - 4, vw - 8, 8))
        lbl_txt = "FAILOVER ON"
    else:
        pygame.draw.rect(surf, C_RED, (vx + vw // 2 - 4, vy + 4, 8, vh - 8))
        lbl_txt = "ISOLATI"
    text(surf, lbl_txt, (vx + vw // 2, vy - 12), F_SMALL, valve_color, anchor="center")

def posizione(surf, t):
    draw_panel(surf, ORIENTATION_PANEL, "ORIENTAZIONE")
    kx = ORIENTATION_PANEL.x + 20
    ky = ORIENTATION_PANEL.y + 40
    draw_kv(surf, "ROLL",      f"{t['roll']:.1f}°",      kx, ky,
            col_v=(C_WHITE  if abs(t["roll"])  < ROLL_WARN
                   else C_YELLOW if abs(t["roll"])  < ROLL_CRIT
                   else C_RED))
    draw_kv(surf, "PITCH",     f"{t['pitch']:.1f}°",     kx, ky + 34,
            col_v=(C_WHITE  if abs(t["pitch"]) < PITCH_WARN
                   else C_YELLOW if abs(t["pitch"]) < PITCH_CRIT
                   else C_RED))
    draw_kv(surf, "YAW",       f"{t['yaw']:.1f}°",       kx + 215, ky)
    draw_kv(surf, "SPEED km/h",f"{t['spd_fused']:.1f} km/h", kx + 215, ky + 34,
            col_v=(C_WHITE  if t["spd_fused"] < SPD_WARN
                   else C_YELLOW if t["spd_fused"] < SPD_CRIT
                   else C_RED))


def draw_led_indicator(surf, x, y, label, ok):
    """LED quadrato + etichetta, per stato booleano di un sensore/canale."""
    col = C_GREEN if ok else C_RED
    pygame.draw.rect(surf, col, (x, y, 18, 18), border_radius=3)
    pygame.draw.rect(surf, C_BORDER, (x, y, 18, 18), 1, border_radius=3)
    text(surf, label, (x + 26, y + 9), F_LABEL, C_TEXT, anchor="midleft")
    text(surf, "OK" if ok else "FAIL", (x + 26 + 130, y + 9), F_VAL,
         col, anchor="midleft")


def draw_diag_panel(surf, t):
    """Pannello diagnostica: stato sensori extra, errore rotta, limite termico, flusso ottico."""
    draw_panel(surf, DIAG_PANEL, "DIAGNOSTICA SENSORI & NAV")
    kx = DIAG_PANEL.x + 24
    ky = DIAG_PANEL.y + 42

    draw_led_indicator(surf, kx, ky,             "FLUSSO OTTICO", t["sensor_optflow_ok"])
    draw_led_indicator(surf, kx+225, ky ,        "LIDAR",         t["sensor_lidar_ok"])
    draw_led_indicator(surf, kx+450, ky ,        "PACCHETTO RC",  not t["sensor_packet_lost"])

    kx2 = kx
    err = abs(t["heading_error"])
    col_err = (C_WHITE if err < HDG_ERR_WARN
               else C_YELLOW if err < HDG_ERR_CRIT
               else C_RED)
    draw_kv(surf, "ERR ROTTA", f"{t['heading_error']:.1f}°", kx2, ky+30, col_v=col_err)

    limitato = t["thermal_limit"] < GAS_MASSIMO
    draw_kv(surf, "LIM TERMICO", f"{t['thermal_limit']} µs", kx2+225, ky + 30,
            col_v=C_YELLOW if limitato else C_WHITE)

    draw_kv(surf, "OPT VX", f"{t['opt_vx']:.2f} m/s", kx2+450, ky +30)
    draw_kv(surf, "OPT VY", f"{t['opt_vy']:.2f} m/s", kx2+675, ky +30)


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
        col   = LIVELLI[principale["livello"]]["colore"]
        icona = LIVELLI[principale["livello"]]["icona"]
        pygame.draw.rect(surf, col, rect, 3, border_radius=8)
        text(surf, f"{icona} {principale['titolo']}", (rect.x + 16, rect.y + 9), F_HEAD, col)
        sotto = principale["causa"] or principale["descrizione"]
        if sotto:
            text(surf, sotto, (rect.x + 16, rect.y + 36), F_LABEL, C_TEXT)

    hint = "[INVIO] dettagli   [D] diagnostica   [L] log   [M] perché MANUALE"
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
        col   = LIVELLI[a["livello"]]["colore"]
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

# ── Stato interno warning 
_w_alt_prec:  float  = 0.0
_w_t_prec_ms: int    = 0
_w_callouts:  set    = set()
_w_mode_prec: object = None

def aggiorna_warning(t: dict) -> None:
    global _w_alt_prec, _w_t_prec_ms, _w_callouts, _w_mode_prec

    # ── Delta-tempo reale 
    ora_ms       = pygame.time.get_ticks()
    dt_ms        = max(ora_ms - _w_t_prec_ms, 1) if _w_t_prec_ms else 33
    _w_t_prec_ms = ora_ms
    dt_s         = dt_ms / 1000.0

    alt     = t["altitude"]
    spd     = t["spd_fused"]
    roll    = abs(t["roll"])
    in_volo = t["in_flight"]
    mode    = t["mode"]

    # ── Velocità di discesa
    if in_volo:
        v_discesa   = (alt - _w_alt_prec) / dt_s   
        _w_alt_prec = alt
    else:
        v_discesa   = 0.0
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

        #Callout quota 
        if v_discesa <= VDISCESA_CALLOUT:
            if "40" not in _w_callouts and CALLOUT_40_LO < alt < CALLOUT_40_HI:
                _w_callouts.add("40");  riproduci_audio("40.wav")
            elif "30" not in _w_callouts and CALLOUT_30_LO < alt < CALLOUT_30_HI:
                _w_callouts.add("30");  riproduci_audio("30.wav")
            elif "20" not in _w_callouts and CALLOUT_20_LO < alt < CALLOUT_20_HI:
                _w_callouts.add("20");  riproduci_audio("20.wav")
            elif "10" not in _w_callouts and CALLOUT_10_LO < alt < CALLOUT_10_HI:
                _w_callouts.add("10");  riproduci_audio("10.wav")

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
            # Rimane in attesa dell'input dell'utente sul terminale
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
                # passiamo "0" di default per non far fallire l'indexOf(':', 4) sul Teensy
                valore = parts[2] if len(parts) > 2 and parts[2] != "" else "0"
                
                if campo:
                    send_command(campo, valore)
            else:
                print("[CLI] Formato errato. Usa CMD:CAMPO:VALORE (es. CMD:SET_ALTITUDE:200)")
                
        except Exception as e:
            print(f"[CLI] Errore di input: {e}")

def main():
    global _overlay_diagnostica, _overlay_log, _overlay_perche, _scroll_diag, _scroll_log

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
                    _overlay_log = _overlay_perche = False
                elif event.key == pygame.K_l:
                    _overlay_log = not _overlay_log
                    _overlay_diagnostica = _overlay_perche = False
                elif event.key == pygame.K_m:
                    _overlay_perche = not _overlay_perche
                    _overlay_diagnostica = _overlay_log = False
                elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_SPACE):
                    if not (_overlay_diagnostica or _overlay_log or _overlay_perche):
                        _overlay_diagnostica = True
                elif event.key == pygame.K_ESCAPE:
                    _overlay_diagnostica = _overlay_log = _overlay_perche = False
                elif event.key == pygame.K_UP:
                    _scroll_diag = max(0, _scroll_diag - 84)
                    _scroll_log  = max(0, _scroll_log - 48)
                elif event.key == pygame.K_DOWN:
                    _scroll_diag += 84
                    _scroll_log  += 48

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if _rect_banner.collidepoint(event.pos):
                    _overlay_diagnostica = True
                    _overlay_log = _overlay_perche = False

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
        navigation_info_pannel(screen, T)
        posizione(screen, T)
        battery_pannel(screen, T)
        draw_diag_panel(screen, T)


        # ── Overlay (finestre di dettaglio): disegnati per ultimi, sopra a tutto ──
        if _overlay_diagnostica:
            draw_diagnostics_overlay(screen)
        elif _overlay_log:
            draw_log_overlay(screen)
        elif _overlay_perche:
            draw_perche_manuale_overlay(screen, T)

        pygame.display.flip()
        clock.tick(30)

if __name__ == "__main__":
    main()