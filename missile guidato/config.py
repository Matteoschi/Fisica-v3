# ==========================================
#  PANNELLO DI CONTROLLO SIMULAZIONE
# ==========================================

DT = 0.01             
MAX_STEPS = 2000       

# --- PARAMETRI FISICI (AIM-9 Sidewinder Like) ---

# MASSA
MASSA_TOTALE = 85.0       
MASSA_PROPELLENTE = 28.0 

# Il motore brucia tutto in 2.8 secondi 
TEMPO_COMBUSTIONE = 2.8   # Secondi

SPINTA_MOTORE = 25000.0   # Newton 

# AERODINAMICA
DRAG_COEFF = 0.0035       

# --- START POSITIONS ---
# Missile parte da zero
MISSILE_POS = [2500, 0.0, 0.0]       
MISSILE_VEL = [0.0, 0.0, 20.0] 

# Target 
TARGET_POS = [3000.0, 2000.0, 1500.0] 
TARGET_VEL = [-300.0, 60.0, 50.0]    

# Limiti grafici
LIMITI_ASSI = [-500, 3000]