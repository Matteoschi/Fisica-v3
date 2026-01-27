#ifndef COSTANTI_MISSILE_H
#define COSTANTI_MISSILE_H

/* ===============================
   COSTANTI FISICHE GENERALI
   =============================== */

#define G_ACCEL 9.81                 // Accelerazione gravitazionale (m/s^2)
#define GAMMA 1.4                    // Indice adiabatico aria
#define R_GAS 287.05                 // Costante specifica gas aria (J/(kg·K))
#define EPSILON 1e-6                 // Valore numerico di sicurezza

/* ===============================
   ATMOSFERA STANDARD SEMPLIFICATA
   =============================== */

#define DENSITA_ARIA_LIV_MARE 1.225  // kg/m^3
#define PRESSIONE_SEA_LEVEL 101325.0 // Pa
#define SCALE_HEIGHT 8500.0          // m (decadimento esponenziale densità)
#define FATTORE_CORRETTIVO_MACH 2.5
#define K_WAVE 3.75
#define B_MAX 2.4

/* ===============================
   PARAMETRI MISSILE
   =============================== */

#define AREA_MISSILE 0.0127          // m^2 (sezione frontale)
#define AREA_UGELLO 0.0045           // m^2 (ugello motore)

/* ===============================
   AERODINAMICA
   =============================== */

#define LIMITE_STALLO 1.2            // Cl massimo (limite stallo alette)
#define INDUCED_DRAG_K 0.03          // Fattore drag indotto

/* ===============================
   GUIDANCE & STRUTTURA
   =============================== */

#define COSTANTE_NAVIGAZIONE 4.0     // Proportional Navigation (N)
#define LIMIT_G_LOAD 35.0            // Limite strutturale in G
#define RAGGIO_PROSSIMITA 10.0       // m (detonazione prossimità)

/* ===============================
   LIMITI TERMICI
   =============================== */

#define MAX_TEMP_STRUTTURA 750.0     // K (limite termico struttura)
#define MARGINE_SICUREZZA_TEMPERATURA 50.0 // Iniziamo a frenare 50 gradi prima

#define OVERKLOCK 0 // 0 attiva sicurezza , 1 disattiva sicurezza

#endif // COSTANTI_MISSILE_H