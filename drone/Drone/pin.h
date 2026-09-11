#ifndef PIN_H
#define PIN_H

// ============================================================
// PORTE SERIALI HARDWARE
// ============================================================
#define GPS_SERIAL   Serial1
#define TELEMETRIA   Serial4
#define LIDAR_SERIAL Serial2
#define SBUS_SERIAL  Serial7

// ============================================================
// SENSORI ANALOGICI
// ============================================================
const int PIN_ARIA        = A0;
const int PIN_TEMP_MOTORE = A12;
const int PIN_TEMP_ESC    = A6;
const int PIN_TEMP_EST    = A8;

// ============================================================
// SERVI / MOTORE
// ============================================================
const int PIN_INT_SX = 6;
const int PIN_INT_DX = 22;
const int PIN_EST_SX = 23;
const int PIN_EST_DX = 24;
const int PIN_MOTORE = 10;

// ============================================================
// SEGNALAZIONI / SICUREZZA
// ============================================================
const int PIN_LED_ROSSO_ALARM = 2;
const int PIN_LED_VERDE_GPS   = 3;
const int PIN_LED_BLU_PID     = 4;
const int PIN_BUZZER          = 33;
const int PIN_RELE            = 20;

// ============================================================
// BUS / CHIP SELECT
// ============================================================
const int PIN_PMW3901_CS = 25;

#endif
