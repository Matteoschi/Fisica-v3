#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// COMUNICAZIONI
// ============================================================
const int BAUD_RATE_DEBUG = 115200;
const int BAUD_RATE_LORA  = 57600;
const int BAUD_RATE_GPS   = 9600;
const int BAUD_RATE_LIDAR = 115200;

// ============================================================
// BATTERIE
// ============================================================
// HOOVO 4S 14.8V (motore) e OVONIC 3S 11.1V (avionica)
const float VALORE_BATT_MOTORE_BASSA_V = 14.0f;
const float VALORE_BATT_TEENSY_BASSA_V = 10.5f;

const int CAPACITA_TEENSY_mAh = 5200;
const int CAPACITA_MOTORE_mAh = 2200;

// ============================================================
// SCHIANTO E LIMITI DI ASSETTO
// ============================================================
const float SOGLIA_ACCELERAZIONE_SCHIANTO_ms2 = 50.0f;
const float SOGLIA_VELOCITA_CRITICA_SCHIANTO_ms = 5.0f;
const unsigned long TEMPO_CONFERMA_SCHIANTO_ms = 350;

const int MAX_ROLL_deg  = 35;
const int MAX_PITCH_deg = 20;

// ============================================================
// MOTORE / GAS
// ============================================================
const int GAS_NEUTRO_us        = 1000;
const int GAS_MASSIMO_us       = 2000;
const int GAS_MINIMO_us        = 1200;
const int GAS_AVVICINAMENTO_us = 1250;
const int GAS_CROCIERA_us      = 1500;

// ============================================================
// VELOCITA'
// ============================================================
const float MAX_AIRSPEED_X8_kmh       = 90.0f;
const float VELOCITA_STALLO_X8_kmh    = 30.0f;
const float MARGINE_ISTERESI_kmh      = 5.0f;
const float SOGLIA_VELOCITA_DECOLLO_ms = 9.0f;

const float VELOCITA_CROCIERA_DEFAULT_kmh      = 60.0f;
const float VELOCITA_AVVICINAMENTO_DEFAULT_kmh = 45.0f;

// ============================================================
// FRENATA / NAVIGAZIONE
// ============================================================
const float DISTANZA_FRENATA_m = 150.0f;
const float RAGGIO_ACCETTAZIONE_MINIMO_m = 25.0f;

const double TARGET_LAT_DEFAULT_deg = 41.902782;
const double TARGET_LON_DEFAULT_deg = 12.496366;
const float ALTITUDINE_TARGET_DEFAULT_m = 40.0f;

// ============================================================
// ALTITUDINI
// ============================================================
const float ALTITUDINE_MAX_LIDAR_m  = 6.0f;
const float ALTITUDINE_MAX_OTTICO_m = 4.0f;

const float SOGLIA_ALTITUDINE_DECOLLO_m = 5.0f;
const float ALTITUDINE_MAX_m = 120.0f;
const float ALTITUDINE_MIN_m = 10.0f;

const float SOGLIA_DISCORDANZA_QUOTA_m = 3.0f;
const float MAX_VARIAZIONE_ALTITUDINE_PER_CICLO_m = 5.0f;

// Zone di transizione tra sensori (valori derivati, comportamento invariato)
const float ZONA_BLEND_LIDAR_START_m = ALTITUDINE_MAX_LIDAR_m - 2.0f;
const float ZONA_BLEND_LIDAR_END_m   = ALTITUDINE_MAX_LIDAR_m;
const float ZONA_BLEND_OTTICO_START_m = ALTITUDINE_MAX_OTTICO_m - 2.0f;

// ============================================================
// SENSORI: PARAMETRI FISICI / CALIBRAZIONE
// ============================================================
const int CENTRO_SERVO_deg = 90;

const float R_SPECIFIC_ARIA = 287.05f;
const float FATTORE_CONVERSIONE_PITOT_Pa = 3.22f;
const float ALPHA_LIDAR = 0.25f;
const float COSTANTE_CALIBRAZIONE_OTTICA = 0.0012f;
const float VELOCITA_SUOLO_GPS_AFFIDABILE_ms = 3.0f;

const float PRESSIONE_RIFERIMENTO_BARO_hPa = 1013.25f;

const int IMU_CAMPIONI_TARA = 200;
const int BARO_CAMPIONI_TARA = 20;
const int PITOT_CAMPIONI_TARA = 100;

const unsigned long TIMEOUT_CALIBRAZIONE_IMU_ms = 10000;
const unsigned long TIMEOUT_INIT_LIDAR_ms = 3000;
const unsigned long TIMEOUT_INIT_GPS_ms = 1500;
const unsigned long TEMPO_RITENTO_SENSORI_ms = 2000;

// Durante la tara IMU il drone deve rimanere fermo.
// Se roll o pitch cambiano piu' di questa soglia, la tara viene rifiutata.
const float MAX_MOVIMENTO_CALIBRAZIONE_IMU_deg = 1.5f;

// Plausibilita' letture sensori
const float BARO_ALTITUDINE_MIN_PLAUSIBILE_m = -500.0f;
const float BARO_ALTITUDINE_MAX_PLAUSIBILE_m = 8000.0f;
const float BARO_PRESSIONE_MIN_PLAUSIBILE_Pa = 10000.0f;
const float BARO_PRESSIONE_MAX_PLAUSIBILE_Pa = 120000.0f;
const float BARO_TEMPERATURA_MIN_PLAUSIBILE_C = -60.0f;
const float BARO_TEMPERATURA_MAX_PLAUSIBILE_C = 100.0f;

const int PITOT_ADC_MIN_VALIDO = 5;
const int PITOT_ADC_MAX_VALIDO = 1020;

const float INA219_TENSIONE_MIN_PLAUSIBILE_V = 1.0f;
const float INA219_TENSIONE_MAX_PLAUSIBILE_V = 30.0f;

const unsigned long TIMEOUT_LIDAR_DATI_ms = 500;

// ============================================================
// PROTEZIONE TERMICA
// ============================================================
const float ESC_TEMP_DERATING_START_C = 80.0f;
const float ESC_TEMP_DERATING_END_C   = 120.0f;

const float MOTORE_TEMP_DERATING_START_C = 70.0f;
const float MOTORE_TEMP_DERATING_END_C   = 90.0f;

// ============================================================
// PITCH FORZATO
// ============================================================
const float PITCH_DOWN_FORZATO_deg = -8.0f;
const float PITCH_UP_FORZATO_deg   = 12.0f;

// ============================================================
// SERVI / INA219
// ============================================================
const float SERVO_mA_MIN = 0.5f;
const float SERVO_mA_MAX = 3200.0f;
const int ERRORI_CONSECUTIVI_SERVO = 20;

// ============================================================
// GPS
// ============================================================
const unsigned long TIMEOUT_GPS_ms = 1500;
const float SALTO_GPS_MAX_m = 50.0f;
const int CICLI_GPS_CONGELATO_MAX = 500;

// ============================================================
// VERTICAL SPEED / CORRENTE
// ============================================================
const float SOGLIA_SINK_RATE_ms = -4.0f;
const float CORRENTE_MOTORE_MAX_PLAUSIBILE_mA = 45000.0f;

// ============================================================
// AVVIO
// ============================================================
const unsigned long TEMPO_DECOLLO_SICURO_ms = 1500;
const int MAX_TENTATIVI_INIT = 3;

// ============================================================
// PID: LIMITI E VALORI INIZIALI
// ============================================================
const float LIMITE_KP_MAX = 10.0f;
const float LIMITE_KI_MAX = 2.0f;
const float LIMITE_KD_MAX = 5.0f;

const float KP_VEL_DEFAULT   = 1.5f;
const float KI_VEL_DEFAULT   = 0.1f;
const float KD_VEL_DEFAULT   = 0.5f;

const float KP_ROLL_DEFAULT  = 1.2f;
const float KI_ROLL_DEFAULT  = 0.05f;
const float KD_ROLL_DEFAULT  = 0.5f;

const float KP_PITCH_DEFAULT = 1.2f;
const float KI_PITCH_DEFAULT = 0.05f;
const float KD_PITCH_DEFAULT = 0.5f;

const float KP_ALT_DEFAULT   = 0.5f;
const float KI_ALT_DEFAULT   = 0.05f;
const float KD_ALT_DEFAULT   = 0.2f;
// ============================================================
// LOG MICROSD
// ============================================================
const unsigned long INTERVALLO_LOG_SD_ms = 50; // 20 Hz
const int RIGHE_FLUSH_SD = 20;                  // flush circa ogni secondo

#endif
