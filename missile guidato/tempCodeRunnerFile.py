import ctypes
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
try:
    import config
    print(f"[PYHTON] file configurazione inizializzato correttamente")
except ImportError:
    print("[ERRORE] file di inizializzazione configurazione mancante") 

# ==============================================================================
# 1. INIZIALIZZAZIONE SISTEMA DI LOGGING
# ==============================================================================
nome_file_log = "scatola_nera.csv"

# Se esiste un vecchio log, lo cancelliamo per pulizia
if os.path.exists(nome_file_log):
    os.remove(nome_file_log)

# Creiamo il file e scriviamo l'intestazione delle colonne
with open(nome_file_log, "w") as file_log:
    file_log.write("Missile_X,Missile_Y,Missile_Z,Velocita_M,Target_X,Target_Y,Target_Z,Distanza,tempo (s)\n")

print(f"[PYTHON] File di log '{nome_file_log}' inizializzato.")

# ==============================================================================
# 2. CARICAMENTO LIBRERIA MOTORE C
# ==============================================================================
nome_libreria = './missile_guidance.dll' if os.name == 'nt' else './missile_guidance.so'

try:
    libreria_motore_fisico = ctypes.CDLL(nome_libreria)
    print(f"[PYTHON] Libreria '{nome_libreria}' caricata correttameente.")
except OSError:
    print(f"[ERRORE] Impossibile trovare '{nome_libreria}'")
    exit()

# Definizione dei tipi di dati per comunicare con il C
puntatore_array_double = ctypes.POINTER(ctypes.c_double)

# 4 array + 4 numeri singoli (delta_t, spinta, vmax, smorzamento)
libreria_motore_fisico.aggiorna_stato_simulazione.argtypes = [puntatore_array_double, puntatore_array_double, puntatore_array_double, puntatore_array_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
# La funzione ora restituisce un intero (0 o 1)
libreria_motore_fisico.aggiorna_stato_simulazione.restype = ctypes.c_int

# ==============================================================================
# 3. PREPARAZIONE DATI (DA CONFIG.PY)
# ==============================================================================
passo_temporale = config.DT
numero_massimo_passi = config.MAX_STEPS

# Creazione array Ctypes usando i valori del file config
posizione_missile = (ctypes.c_double * 3)(*config.MISSILE_POS)
velocita_missile = (ctypes.c_double * 3)(*config.MISSILE_VEL)

posizione_bersaglio = (ctypes.c_double * 3)(*config.TARGET_POS)
velocita_bersaglio = (ctypes.c_double * 3)(*config.TARGET_VEL)


tracciato_missile_x, tracciato_missile_y, tracciato_missile_z = [], [], []
tracciato_bersaglio_x, tracciato_bersaglio_y, tracciato_bersaglio_z = [], [], []

# ==============================================================================
# 4. CICLO DI SIMULAZIONE
# ==============================================================================
print(f"[PYTHON] Avvio simulazione... \n Target Vel : {config.TARGET_VEL} || target Pos : {config.TARGET_POS} || Missile vel : {config.MISSILE_VEL} || Missile Pos : {config.MISSILE_POS}")

frame_corrente = 0
bersaglio_colpito = False
secondi_trascorsi = 0.0

for passo in range(numero_massimo_passi):
    tracciato_missile_x.append(posizione_missile[0])
    tracciato_missile_y.append(posizione_missile[1])
    tracciato_missile_z.append(posizione_missile[2])
    
    tracciato_bersaglio_x.append(posizione_bersaglio[0])
    tracciato_bersaglio_y.append(posizione_bersaglio[1])
    tracciato_bersaglio_z.append(posizione_bersaglio[2])

    esito_simulazione = libreria_motore_fisico.aggiorna_stato_simulazione(
        posizione_missile, 
        velocita_missile, 
        posizione_bersaglio, 
        velocita_bersaglio, 
        passo_temporale,
        config.SPINTA_MASSIMA,
        config.VELOCITA_MASSIMA,
        config.SMORZAMENTO,
        secondi_trascorsi
    )
    secondi_trascorsi += passo_temporale
    frame_corrente += 1

    if esito_simulazione == 1:
        print(f"[SUCCESSO] IMPATTO CONFERMATO al frame {passo}!")
        bersaglio_colpito = True
        break

# ==============================================================================
# 5. GENERAZIONE GRAFICO 3D
# ==============================================================================
print("[PYTHON] Generazione animazione 3D in corso...")

figura = plt.figure(figsize=(10, 8))
assi = figura.add_subplot(111, projection='3d')

# Impostazione limiti assi dal config
minimo_assi, massimo_assi = config.LIMITI_ASSI
assi.set_xlim(minimo_assi, massimo_assi)
assi.set_ylim(minimo_assi, massimo_assi)
assi.set_zlim(0, massimo_assi)

assi.set_xlabel('Asse X (Metri)')
assi.set_ylabel('Asse Y (Metri)')
assi.set_zlabel('Quota Z (Metri)')
assi.set_title(f'Simulazione Intercettazione Missile\nStatus: {"COLPITO" if bersaglio_colpito else "MANCATO"}')

# Creazione oggetti grafici vuoti
linea_missile, = assi.plot([], [], [], 'r-', linewidth=2, label='Traiettoria Missile')
linea_bersaglio, = assi.plot([], [], [], 'b--', linewidth=1, label='Traiettoria Bersaglio')
testa_missile, = assi.plot([], [], [], 'ro', markersize=5)
testa_bersaglio, = assi.plot([], [], [], 'bo', markersize=5)

def funzione_aggiornamento_grafico(frame):
    # Aggiorna Missile
    linea_missile.set_data(tracciato_missile_x[:frame], tracciato_missile_y[:frame])
    linea_missile.set_3d_properties(tracciato_missile_z[:frame])
    testa_missile.set_data([tracciato_missile_x[frame]], [tracciato_missile_y[frame]])
    testa_missile.set_3d_properties([tracciato_missile_z[frame]])
    
    # Aggiorna Bersaglio
    linea_bersaglio.set_data(tracciato_bersaglio_x[:frame], tracciato_bersaglio_y[:frame])
    linea_bersaglio.set_3d_properties(tracciato_bersaglio_z[:frame])
    testa_bersaglio.set_data([tracciato_bersaglio_x[frame]], [tracciato_bersaglio_y[frame]])
    testa_bersaglio.set_3d_properties([tracciato_bersaglio_z[frame]])
    
    return linea_missile, linea_bersaglio, testa_missile, testa_bersaglio

animazione = FuncAnimation(
    figura, 
    funzione_aggiornamento_grafico, 
    frames=len(tracciato_missile_x), 
    interval=20, 
    blit=False,
    repeat=False
)

plt.legend()
plt.show()