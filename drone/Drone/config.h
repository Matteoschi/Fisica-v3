#ifndef CONFIG_H
#define CONFIG_H

// BATTERIE
// Aggiornato per HOOVO 4S 14.8V (motore) e OVONIC 3S 11.1V (avionica)
const float VALORE_BATT_MOTORE_BASSA_V = 14.0f; // ~3.5V per cella su 4S
const float VALORE_BATT_TEENSY_BASSA_V = 10.5f; // ~3.5V per cella su 3S

// SCHIANTO E LIMITI DI ASSETTO
const float SOGLIA_ACCELERAZIONE_SCHIANTO_ms2 = 50.0f;

const int MAX_ROLL_deg  = 35;
const int MAX_PITCH_deg = 20;

// MOTORE / GAS (valori in microsecondi, us, per i comandi ESC/PWM)
const int GAS_NEUTRO_us     = 1000;
const int GAS_MASSIMO_us    = 2000;
const int GAS_MINIMO_us     = 1200;
const int GAS_AVVICINAMENTO_us = 1250;
const int GAS_CROCIERA_us      = 1500; // ~50% gas per mantenere 60 km/h

// VELOCITÀ (in km/h) - Adeguate al peso di 3.9 kg e crociera di 60 km/h
const float MAX_AIRSPEED_X8_kmh      = 90.0f; 
const float VELOCITA_STALLO_X8_kmh   = 30.0f; // Alzata per via del carico alare maggiore
const float MARGINE_ISTERESI_kmh     = 5.0f;

const float SOGLIA_VELOCITA_DECOLLO_ms = 9.0f;   // m/s, alzata per garantire un lancio sicuro

// FRENATA
const float DISTANZA_FRENATA_m = 150.0f;

// ALTITUDINI 
const float ALTITUDINE_MAX_LIDAR_m  = 6.0f;   // Sopra questa quota il LIDAR non è affidabile
const float ALTITUDINE_MAX_OTTICO_m = 4.0f;   // Sopra questa quota il flusso ottico non è affidabile

const float SOGLIA_ALTITUDINE_DECOLLO_m = 5.0f;

const float ALTITUDINE_MAX_m = 120.0f;
const float ALTITUDINE_MIN_m = 10.0f;

const float MIN_ESC_START_TEMP_C = 80.0f;
const float MIN_ESC_END_TEMP_C   = 120.0f;

// PITCH FORZATO (gradi)
const float PITCH_DOWN_FORZATO_deg = -8.0f;
const float PITCH_UP_FORZATO_deg   = 12.0f;

// NAVIGAZIONE
const float RAGGIO_ACCETTAZIONE_MINIMO_m = 25.0f;

// PROTEZIONE TERMICA MOTORE (gradi Celsius)
const float MIN_THROTTLE_START_TEMP_C = 70.0f;
const float MAX_THROTTLE_END_TEMP_C   = 90.0f;

// ASSORBIMENTI SERVO
const float SERVO_mA_MIN = 0.5f;
const float SERVO_mA_MAX = 3200.0f; // Alzato per gestire i picchi da 2.9A dei DS3225
const int ERRORI_CONSECUTIVI_SERVO = 20;   // Numero di letture consecutive fuori range prima di segnalare anomalia persistente

const int CAPACITA_TEENSY = 5200;
const int CAPACITA_MOTORE = 2200;

#endif