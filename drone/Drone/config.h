#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// BATTERIE
// ============================================================

const float VALORE_BATT_MOTORE_BASSA_V = 13.5f;
const float VALORE_BATT_TEENSY_BASSA_V = 4.9f;


// ============================================================
// SCHIANTO E LIMITI DI ASSETTO
// ============================================================

const int SOGLIA_G_SCHIANTO = 50;

const int MAX_ROLL_g = 35;
const int MAX_PITCH_g = 20;


// ============================================================
// MOTORE / GAS
// ============================================================

const int GAS_NEUTRO = 1000;
const int GAS_MASSIMO = 2000;
const int GAS_MINIMO = 1200;


// ============================================================
// VELOCITÀ
// ============================================================

const float MAX_AIRSPEED_X8_km = 45.0f;
const float VEL_STALLO_X8_km = 20.0f;
const float SOGLIA_VELO_DECOLLO_MS = 5.0f;


// ============================================================
// FRENATA
// ============================================================

const float DISTANZA_FRENATA_m = 150.0f;


// ============================================================
// ALTEZZE
// ============================================================

const float ALTEZZA_MAX_LIDAR_m = 6.0f;
const float ALTEZZA_MAX_SENSORE_OTTICO_m = 4.0f;

const float SOGLIA_ALT_DECOLLO_M = 5.0f;

const float ALTEZZA_MAX_m = 120.0f;
const float ALTEZZA_MIN_m = 10.0f;


// ============================================================
// PITCH FORZATO
// ============================================================

const float PITCH_DOWN_FORZATO = -8.0f;
const float PITCH_UP_FORZATO = 12.0f;


// ============================================================
// NAVIGAZIONE
// ============================================================

const float RAGGIO_ACCETTAZIONE_MINIMO_m = 25.0f;


const float T_MOTORE_THROTTLE_START = 70.0f;   
const float T_MOTORE_THROTTLE_END  = 90.0f;        

const float SERVO_mA_MIN = 0.5f;
const float SERVO_mA_MAX = 2500.0f;
const int ERRORI_CONSECUTIVI_SERVO = 20;   // Numero di letture consecutive fuori range prima di segnalare anomalia persistente
#endif