import ctypes
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import sys
import csv   
import math  
import pandas as pd
try:
    import config
    print(f"[PYTHON] File configurazione 'config.py' importato correttamente.")
except ImportError:
    print("[WARN] 'config.py' non trovato. ")

# ==============================================================================
# 2.MOTORE C
# ==============================================================================
cartella_script = os.path.dirname(os.path.abspath(__file__))
nome_dll = "missile_guidance.dll" if os.name == 'nt' else "missile_guidance.so"
percorso_completo_dll = os.path.join(cartella_script, nome_dll)

try:
    libreria_motore_fisico = ctypes.CDLL(percorso_completo_dll)
    print(f"[PYTHON] Libreria caricata con successo: {percorso_completo_dll}")
except OSError as e:
    print(f"[ERRORE] Impossibile caricare la DLL")
    print(f"Dettaglio errore: {e}")
    sys.exit(1)

# ==============================================================================
# 3. INTERFACCIA CTYPES
# ==============================================================================
puntatore_array_double = ctypes.POINTER(ctypes.c_double)

libreria_motore_fisico.aggiorna_missile.argtypes = [
    puntatore_array_double, # 1. Posizione Missile
    puntatore_array_double, # 2. Velocità Missile
    puntatore_array_double, # 3. Posizione Target
    puntatore_array_double, # 4. Velocità Target
    ctypes.c_double,        # 5. dt
    ctypes.c_double,        # 6. Tempo Corrente
    ctypes.c_double,        # 7. Massa Totale Launch
    ctypes.c_double,        # 8. Massa Carburante
    ctypes.c_double,        # 9. Durata Motore
    ctypes.c_double,        # 10. Massa a Vuoto
    ctypes.POINTER(ctypes.c_double),  # 11. Puntatore Massa Fuel
    ctypes.c_double,        # 12. Consumo kg/s
    ctypes.c_double,        # 13. Spinta Nominale
    ctypes.c_double,        # 14. Coeff Drag
    puntatore_array_double  # 15. Array Output
]
dati_telemetria = (ctypes.c_double * 14)(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)  # Array di 14 elementi per tutta la telemetria
libreria_motore_fisico.aggiorna_missile.restype = ctypes.c_double 

# ==============================================================================
# 4. PREPARAZIONE DATI
# ==============================================================================
posizione_missile = (ctypes.c_double * 3)(*config.MISSILE_POS)
velocita_missile = (ctypes.c_double * 3)(*config.MISSILE_VEL)
posizione_bersaglio = (ctypes.c_double * 3)(*config.TARGET_POS)
velocita_bersaglio = (ctypes.c_double * 3)(*config.TARGET_VEL)

tracciato_m_x, tracciato_m_y, tracciato_m_z = [], [], []
tracciato_t_x, tracciato_t_y, tracciato_t_z = [], [], []

passo_temporale = config.DT
numero_massimo_passi = config.MAX_STEPS

# Parametri aggiuntivi per il motore
massa_a_vuoto = config.MASSA_TOTALE - config.MASSA_PROPELLENTE  # Massa secca
massa_fuel_attuale = ctypes.c_double(config.MASSA_PROPELLENTE)   # Fuel rimasto (puntatore)
consumo_kg_s = config.MASSA_PROPELLENTE / config.TEMPO_COMBUSTIONE  # Rateo di consumo

# ==============================================================================
# SETUP LOGGING CSV 
# ==============================================================================
percorso_file_csv = r"missile guidato\scatola_nera.csv"

# Creazione cartella se non esiste
dir_path = os.path.dirname(percorso_file_csv)
if not os.path.exists(dir_path):
    try:
        os.makedirs(dir_path)
    except OSError:
        pass # Se fallisce userà la cartella corrente o darà errore dopo

try:
    file_csv = open(percorso_file_csv, mode='w', newline='')
    writer = csv.writer(file_csv, delimiter=';') 
    writer.writerow([
        "Tempo", "Distanza", 
        "Missile_X", "Missile_Y", "Missile_Z", "Vel_Totale",
        "Target_X", "Target_Y", "Target_Z", 
        "guide_force m/s^2", "Densità_Aria_kg_m3", "Temperatura_Kelvin", "Temp_Ristagno_K", "Mach", "Pressione_Dinamica_Pa", "Pressione_Locale_Pa", "Spinta_Motore_N", "Spinta_Base_N", "Cd_Totale", "Cl_Attuale", "Fuel_kg","stato sicurezza"
    ])
    print(f"[LOG] File CSV aperto: {percorso_file_csv}")
except Exception as e:
    print(f"[ERRORE] Impossibile aprire file CSV: {e}")
    sys.exit(1)

# ==============================================================================
# 5. CICLO DI SIMULAZIONE
# ==============================================================================
print(f"\n[PYTHON] Avvio simulazione...")

bersaglio_colpito = False
tempo_trascorso = 0.0
massa_secca = config.MASSA_TOTALE - config.MASSA_PROPELLENTE

for passo in range(numero_massimo_passi):
    # 1. Salviamo le posizioni attuali per il grafico
    tracciato_m_x.append(posizione_missile[0])
    tracciato_m_y.append(posizione_missile[1])
    tracciato_m_z.append(posizione_missile[2])
    
    tracciato_t_x.append(posizione_bersaglio[0])
    tracciato_t_y.append(posizione_bersaglio[1])
    tracciato_t_z.append(posizione_bersaglio[2])

    # Calcolo velocità totale per il log
    vel_tot = math.sqrt(velocita_missile[0]**2 + velocita_missile[1]**2 + velocita_missile[2]**2)

    # 2. Chiamata al C
    # Ritorna la distanza attuale; i dati dettagliati vanno in dati_telemetria
    distanza_attuale = libreria_motore_fisico.aggiorna_missile(
        posizione_missile, 
        velocita_missile, 
        posizione_bersaglio, 
        velocita_bersaglio, 
        passo_temporale,
        ctypes.c_double(tempo_trascorso),
        ctypes.c_double(config.MASSA_TOTALE),
        ctypes.c_double(config.MASSA_PROPELLENTE),
        ctypes.c_double(config.TEMPO_COMBUSTIONE),
        ctypes.c_double(massa_a_vuoto),
        ctypes.byref(massa_fuel_attuale),
        ctypes.c_double(consumo_kg_s),
        ctypes.c_double(config.SPINTA_MOTORE),
        ctypes.c_double(config.DRAG_COEFF),
        dati_telemetria
    )
    
    # Lettura dati telemetria da C:
    # [0] = distanza (m) - Distanza Target-Missile
    # [1] = guide_force (m/s²) - Accelerazione di guida Pro-Nav
    # [2] = densita_aria (kg/m³) - Densità atmosferica corrente
    # [3] = temperatura_kelvin (K) - Temperatura atmosferica
    # [4] = temperatura_ristagno (K) - Temperatura di ristagno (stagnation)
    # [5] = numero_di_mach - Numero di Mach attuale
    # [6] = pressione_dinamica (Pa) - Pressione dinamica (bernoulli)
    # [7] = pressione_locale (Pa) - Pressione atmosferica locale
    # [8] = spinta_motore_totale (N) - Spinta motore con bonus altitudine
    # [9] = spinta_attuale (N) - Spinta motore nominale
    # [10] = cd_totale - Coefficiente drag totale (parassite + indotto)
    # [11] = cl_attuale - Coefficiente portanza attuale
    # [12] = (riservato)
    # [13] = (riservato)
    
    valore_distanza = dati_telemetria[0]   # Distanza Target-Missile (m)
    valore_g_guida  = dati_telemetria[1]   # G-Force dalla guida Pro-Nav (m/s²)
    valore_rho      = dati_telemetria[2]   # Densità aria (kg/m³)
    valore_temp     = dati_telemetria[3]   # Temperatura (K)
    valore_temp_ristagno = dati_telemetria[4]  # Temperatura di ristagno (K)
    valore_mach     = dati_telemetria[5]   # Numero di Mach
    valore_q_dyn    = dati_telemetria[6]   # Pressione dinamica (Pa)
    valore_p_loc    = dati_telemetria[7]   # Pressione locale (Pa)
    valore_spinta   = dati_telemetria[8]   # Spinta motore reale (N)
    valore_spinta_base = dati_telemetria[9]  # Spinta motore base (N)
    valore_cd       = dati_telemetria[10]  # Cd totale
    valore_cl       = dati_telemetria[11]  # Cl attuale
    codice_stato = dati_telemetria[12]

    testo_stato = "OK"
    if codice_stato == 1.0:
        testo_stato = "LIMIT_STRUCT (G)"    # Limitato dalla struttura
    elif codice_stato == 1.5:
        testo_stato = "LIMIT_AERO (CL)"     # Limitato dalle ali (non abbastanza portanza)
    elif codice_stato == 2.0:
        testo_stato = "LOST_LOCK"           # Perso visiva
    elif codice_stato == 3.0:
        testo_stato = "STALL_SPEED"         # Troppo lento
    elif codice_stato == 4.0:
        testo_stato = "DANGER_STRUCT"       # Overclock: Sta piegando il telaio!
    elif codice_stato == 4.5:
        testo_stato = "DANGER_SPIN"
    # Lettura fuel dal puntatore aggiornato
    fuel = massa_fuel_attuale.value

    # SCRITTURA SU CSV
    # Colonne: Tempo | Distanza | Pos Missile (X,Y,Z) | Vel Totale | Pos Target (X,Y,Z) | 
    #          G_Load | RHO | Temperatura | T_Ristagno | Mach | Q_dinamica | P_locale | Spinta | Spinta_base | Cd | Cl | Fuel
    riga_dati=[
        f"{tempo_trascorso:.3f}", f"{valore_distanza:.2f}",
        f"{posizione_missile[0]:.2f}", f"{posizione_missile[1]:.2f}", f"{posizione_missile[2]:.2f}", f"{vel_tot:.2f}",
        f"{posizione_bersaglio[0]:.2f}", f"{posizione_bersaglio[1]:.2f}", f"{posizione_bersaglio[2]:.2f}",
        f"{valore_g_guida:.2f}", f"{valore_rho:.6f}", f"{valore_temp:.2f}", f"{valore_temp_ristagno:.2f}", f"{valore_mach:.3f}", 
        f"{valore_q_dyn:.0f}", f"{valore_p_loc:.0f}", f"{valore_spinta:.0f}", f"{valore_spinta_base:.0f}", f"{valore_cd:.4f}", f"{valore_cl:.4f}", f"{fuel:.2f}", testo_stato
    ]
    riga_excel_ita = [x.replace('.', ',') for x in riga_dati[:-1]] + [riga_dati[-1]]
    writer.writerow(riga_excel_ita)


    tempo_trascorso += passo_temporale

    if distanza_attuale <= 0.0 or distanza_attuale < 8.0: 
        print(f"[SUCCESSO] TARGET COLPITO al passo {passo} (t={tempo_trascorso:.2f}s)!")
        bersaglio_colpito = True
        break
        
    if posizione_missile[2] < 0:
        print(f"[FALLIMENTO] Schianto al suolo (t={tempo_trascorso:.2f}s).")
        break

# Chiusura file CSV
file_csv.close()
print(f"[LOG] Dati salvati correttamente.")

if not bersaglio_colpito:
    print(f"[INFO] Simulazione terminata SENZA impatto.")

# ==============================================================================
# 6. VISUALIZZAZIONE GRAFICA 3D
# ==============================================================================
print("[PYTHON] Generazione grafico 3D...")

figura = plt.figure(figsize=(10, 8))
assi = figura.add_subplot(111, projection='3d')

min_assi, max_assi = config.LIMITI_ASSI
assi.set_xlim(min_assi, max_assi)
assi.set_ylim(min_assi, max_assi)
assi.set_zlim(0, max_assi)

assi.set_xlabel("X (m)")
assi.set_ylabel("Y (m)")
assi.set_zlabel("Z (Altezza m)")
assi.set_title(f"Simulazione Missile - {'IMPATTO' if bersaglio_colpito else 'MANCATO'}")

linea_missile, = assi.plot([], [], [], 'r-', linewidth=2, label='Missile (AIM-9 Like)')
linea_target, = assi.plot([], [], [], 'b--', linewidth=1, label='Target')

def aggiorna_grafico(frame):
    idx = frame * 5  
    if idx >= len(tracciato_m_x): idx = len(tracciato_m_x) - 1
    
    linea_missile.set_data(tracciato_m_x[:idx], tracciato_m_y[:idx])
    linea_missile.set_3d_properties(tracciato_m_z[:idx])
    
    linea_target.set_data(tracciato_t_x[:idx], tracciato_t_y[:idx])
    linea_target.set_3d_properties(tracciato_t_z[:idx])
    
    return linea_missile, linea_target

totale_frames = len(tracciato_m_x) // 5
animazione = FuncAnimation(figura, aggiorna_grafico, frames=totale_frames, interval=1, blit=False)

plt.legend()
plt.show()
