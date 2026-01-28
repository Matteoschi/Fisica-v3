#ifndef COSTANTI_MISSILE_H
#define COSTANTI_MISSILE_H

/* COSTANTI FISICHE */
#define G_ACCEL 9.80665              // m/s^2
#define GAMMA 1.4                    // -
#define R_GAS 287.05                 // J/(kg·K)
#define PI 3.141592653589793

/* ATMOSFERA (ISA semplificata) */
#define DENSITA_ARIA_LIV_MARE 1.225  // kg/m^3
#define PRESSIONE_SEA_LEVEL 101325.0 // Pa
#define SCALE_HEIGHT 8400.0          // m (valore accettabile; 7500–8400 entrambe usate)

/* REGIME COMPRESSIBILE */
#define FATTORE_CORRETTIVO_MACH 2.3  // suggerito (meno aggressivo di 2.5)
#define K_WAVE 3.5                   // wave-drag moderato
#define B_MAX 2.3                    // Prandtl-Glauert limit

/* GEOMETRIA */
#define AREA_MISSILE 0.0133          // m^2 (tuo valore; ~127 mm diam => 0.01267; 0.0133 è accettabile)
#define AREA_UGELLO 0.0038           // m^2 (consigliato; exit dia ≈ 69.6 mm)

/* AERODINAMICA REALISTICA */
#define LIMITE_STALLO 1.4            // Cl_max realistico per missile con canard
#define INDUCED_DRAG_K 0.22          // 1/(pi*e*AR) realistico per low-AR missile

/* GUIDANCE & STRUTTURA */
#define COSTANTE_NAVIGAZIONE 4.0     // PN
#define LIMIT_G_LOAD 35.0            // g (limite operativo realistico)
#define RAGGIO_PROSSIMITA 9.0        // m
#define ANG_GRADI_CONO_VISIONE 40    //Gradi

/* TERMICO / SICUREZZA */
#define MAX_TEMP_STRUTTURA 750.0     // K
#define MARGINE_SICUREZZA_TEMPERATURA 30.0          
#define EPSILON 1e-6
#define OVERCLOCK 0


// Codici di stato del sistema di guida
// Codici di stato estesi
#define STATUS_OK                 0.0

#define STATUS_LIMIT_STRUCTURAL   1.0  // Limitato dalla struttura (max G)
#define STATUS_LIMIT_AERO         1.5  // Limitato dall'aerodinamica (max lift)
#define STATUS_LIMIT_TEMPERATURE  1.8  // Limitato dalla temperatura strutturale
#define STATUS_LIMIT_SPEED        1.9  
#define STATUS_STALL_SPEED        3.0  // Velocità insufficiente
#define STATUS_LOCK_LOST          2.0  // Target fuori visuale

// in overclock
#define STATUS_OVERCLOCK_TEMPERATURE   2.8  
#define STATUS_OVERCLOCK_SPEED         2.9  
#define STATUS_OVERCLOCK_STRUCT   4.0  // Overclock: Struttura sotto stress estremo
#define STATUS_OVERCLOCK_AERO     4.5  // Overclock: Richiesta aerodinamica impossibile (Stallo profondo)

#endif // COSTANTI_MISSILE_H
