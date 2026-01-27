DT = 0.01              # 10ms (sufficiente per la simulazione, 0.005 era eccessivo)
MAX_STEPS = 6000       # 60 secondi (durata massima della batteria termica del seeker)

# ==========================================
#  PARAMETRI FISICI (AIM-9M Sidewinder Data)
# ==========================================

MASSA_TOTALE = 85.5       
MASSA_PROPELLENTE = 27.5  

TEMPO_COMBUSTIONE = 5.2   
SPINTA_MOTORE = 11770.0   

DRAG_COEFF = 0.40         


MISSILE_POS = [0.0, 0.0, 10000.0]       

MISSILE_VEL = [300.0, 0.0, 20.0] 

TARGET_POS = [5000.0, 1000.0, 5000.0] 

TARGET_VEL = [479.0, 200.0, 0.0] 

LIMITI_ASSI = [-2000, 10000]