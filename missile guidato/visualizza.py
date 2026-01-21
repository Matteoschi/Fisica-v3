import ctypes
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import sys

# ==============================================================================
# 1. CARICAMENTO CONFIGURAZIONE
# ==============================================================================
try:
    import config
    print(f"[PYTHON] File configurazione 'config.py' importato correttamente.")
except ImportError:
    print("[WARN] 'config.py' non trovato. ")

# ==============================================================================
# 2. CARICAMENTO LIBRERIA MOTORE C
# ==============================================================================
cartella_script = os.path.dirname(os.path.abspath(__file__))
nome_dll = "missile_guidance.dll" if os.name == 'nt' else "missile_guidance.so" #SO POEW LINUX O MAC
percorso_completo_dll = os.path.join(cartella_script, nome_dll)

try:
    libreria_motore_fisico = ctypes.CDLL(percorso_completo_dll)
    print(f"[PYTHON] Libreria caricata con successo: {percorso_completo_dll}")
except OSError as e:
    print(f"[ERRORE] Impossibile caricare la DLL. Hai compilato il file C?")
    print(f"Dettaglio errore: {e}")
    sys.exit(1)

# ==============================================================================
# 3. INTERFACCIA CTYPES (IL PONTE TRA PYTHON E C)
# ==============================================================================
puntatore_array_double = ctypes.POINTER(ctypes.c_double)

libreria_motore_fisico.aggiorna_missile.argtypes = [
    puntatore_array_double, # 1. Posizione Missile (Array)
    puntatore_array_double, # 2. Velocità Missile (Array)
    puntatore_array_double, # 3. Posizione Target (Array)
    puntatore_array_double, # 4. Velocità Target (Array)
    ctypes.c_double,        # 5. Delta Time (dt)
    ctypes.c_double,        # 6. Tempo Corrente (serve per il motore)
    ctypes.c_double,        # 7. Massa Totale al lancio
    ctypes.c_double,        # 8. Massa del solo Carburante
    ctypes.c_double,        # 9. Durata combustione motore
    ctypes.c_double,        # 10. Spinta in Newton
    ctypes.c_double         # 11. Coefficiente Aerodinamico
]

# La funzione C restituisce un double (la distanza)
libreria_motore_fisico.aggiorna_missile.restype = ctypes.c_double 

# ==============================================================================
# 4. PREPARAZIONE DATI
# ==============================================================================
# Creiamo gli array compatibili con C usando i dati del config
posizione_missile = (ctypes.c_double * 3)(*config.MISSILE_POS)
velocita_missile = (ctypes.c_double * 3)(*config.MISSILE_VEL)
posizione_bersaglio = (ctypes.c_double * 3)(*config.TARGET_POS)
velocita_bersaglio = (ctypes.c_double * 3)(*config.TARGET_VEL)

# Liste per salvare lo storico della traiettoria (per il grafico)
tracciato_m_x, tracciato_m_y, tracciato_m_z = [], [], []
tracciato_t_x, tracciato_t_y, tracciato_t_z = [], [], []

passo_temporale = config.DT
numero_massimo_passi = config.MAX_STEPS

# ==============================================================================
# 5. CICLO DI SIMULAZIONE
# ==============================================================================
print(f"\n[PYTHON] Avvio simulazione (Fisica a Massa Variabile)...")
print(f" - Massa Lancio: {config.MASSA_TOTALE} kg")
print(f" - Carburante: {config.MASSA_PROPELLENTE} kg")
print(f" - Burn Time: {config.TEMPO_COMBUSTIONE} s")

bersaglio_colpito = False
tempo_trascorso = 0.0

for passo in range(numero_massimo_passi):
    # 1. Salviamo le posizioni attuali per il grafico
    tracciato_m_x.append(posizione_missile[0])
    tracciato_m_y.append(posizione_missile[1])
    tracciato_m_z.append(posizione_missile[2])
    
    tracciato_t_x.append(posizione_bersaglio[0])
    tracciato_t_y.append(posizione_bersaglio[1])
    tracciato_t_z.append(posizione_bersaglio[2])

    # 2. Chiamata al C (Passiamo tutti i parametri fisici aggiornati)
    distanza_attuale = libreria_motore_fisico.aggiorna_missile(
        posizione_missile, 
        velocita_missile, 
        posizione_bersaglio, 
        velocita_bersaglio, 
        passo_temporale,
        ctypes.c_double(tempo_trascorso),          # TEMPO CORRENTE
        ctypes.c_double(config.MASSA_TOTALE),      # MASSA START
        ctypes.c_double(config.MASSA_PROPELLENTE), # BENZINA
        ctypes.c_double(config.TEMPO_COMBUSTIONE), # QUANTO DURA IL MOTORE
        ctypes.c_double(config.SPINTA_MOTORE),     # POTENZA
        ctypes.c_double(config.DRAG_COEFF)         # AERODINAMICA
    )
    
    # 3. Aggiorniamo il tempo
    tempo_trascorso += passo_temporale

    # 4. Controlli di fine simulazione
    # Se la distanza è <= 0 o molto piccola (proximity fuse), abbiamo colpito
    if distanza_attuale <= 0.0 or distanza_attuale < 8.0: 
        print(f"[SUCCESSO] TARGET COLPITO al passo {passo} (t={tempo_trascorso:.2f}s)!")
        print(f"           Coordinate impatto: X={posizione_missile[0]:.1f}, Y={posizione_missile[1]:.1f}, Z={posizione_missile[2]:.1f}")
        bersaglio_colpito = True
        break
        
    # Se la Z (altezza) è negativa, ci siamo schiantati a terra
    if posizione_missile[2] < 0:
        print(f"[FALLIMENTO] Il missile si è schiantato al suolo (t={tempo_trascorso:.2f}s).")
        break

if not bersaglio_colpito:
    print(f"[INFO] Simulazione terminata senza impatto (Tempo esaurito o mancato).")

# ==============================================================================
# 6. VISUALIZZAZIONE GRAFICA 3D
# ==============================================================================
print("[PYTHON] Generazione grafico 3D...")

figura = plt.figure(figsize=(10, 8))
assi = figura.add_subplot(111, projection='3d')

# Impostazione limiti assi dal config
min_assi, max_assi = config.LIMITI_ASSI
assi.set_xlim(min_assi, max_assi)
assi.set_ylim(min_assi, max_assi)
assi.set_zlim(0, max_assi)

assi.set_xlabel("X (m)")
assi.set_ylabel("Y (m)")
assi.set_zlabel("Z (Altezza m)")
assi.set_title(f"Simulazione Missile - {'IMPATTO' if bersaglio_colpito else 'MANCATO'}")

# Linee vuote che aggiorneremo
linea_m, = assi.plot([], [], [], 'r-', linewidth=2, label='Missile (AIM-9 Like)')
linea_t, = assi.plot([], [], [], 'b--', linewidth=1, label='Target')

# Funzione di animazione
def aggiorna_grafico(frame):
    # Usiamo 'frame' come indice per far vedere l'evoluzione nel tempo
    # Aumentiamo la velocità saltando qualche step se necessario
    idx = frame * 5  
    if idx >= len(tracciato_m_x): idx = len(tracciato_m_x) - 1
    
    linea_m.set_data(tracciato_m_x[:idx], tracciato_m_y[:idx])
    linea_m.set_3d_properties(tracciato_m_z[:idx])
    
    linea_t.set_data(tracciato_t_x[:idx], tracciato_t_y[:idx])
    linea_t.set_3d_properties(tracciato_t_z[:idx])
    
    return linea_m, linea_t

# Calcoliamo i frame necessari
totale_frames = len(tracciato_m_x) // 5
animazione = FuncAnimation(figura, aggiorna_grafico, frames=totale_frames, interval=1, blit=False)

plt.legend()
plt.show()