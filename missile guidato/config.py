DT = 0.01              # 10ms (sufficiente per la simulazione, 0.005 era eccessivo)
MAX_STEPS = 6000       # 60 secondi (durata massima della batteria termica del seeker)
Distanza_Impatto = 1.0   # Distanza minima per considerare l'impatto (metri)

# ==========================================
#  PARAMETRI FISICI (AIM-9M Sidewinder Data)
# ==========================================

MASSA_TOTALE = 85.5       
MASSA_PROPELLENTE = 27.5  

TEMPO_COMBUSTIONE = 5.2   
SPINTA_MOTORE = 11770.0   

DRAG_COEFF = 0.40         


MISSILE_POS = [-5000, 0.0, 3000.0]       

MISSILE_VEL = [660, 0.0, -5] 

TARGET_POS = [-1800, 50.0, 3000.0] 

TARGET_VEL = [670, 0, 100] 

LIMITI_ASSI = [-5000, 8000]