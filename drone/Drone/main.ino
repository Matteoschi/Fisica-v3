#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "SBUS.h"
#include <Adafruit_INA219.h>
#include <Adafruit_BMP3XX.h>
#include <Bitcraze_PMW3901.h>
#include <math.h>
#include "config.h"

//  CONFIGURAZIONE SERIALI
#define GPS_SERIAL      Serial1
#define TELEMETRIA      Serial4
#define BAUD_RATE_GPS   9600
#define BAUD_RATE_LORA  57600
#define BAUD_RATE_LIDAR 115200

//  SENSORI E OGGETTI GLOBALI
TinyGPSPlus       gps;
Adafruit_BNO055   giroscopio = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX   barometro;
Bitcraze_PMW3901  flussoOttico(25);
SBUS              ricevente(Serial7);

Adafruit_INA219 sensoreMotore(0x40);
Adafruit_INA219 sensoreIntSX(0x41);
Adafruit_INA219 sensoreIntDX(0x42);
Adafruit_INA219 sensoreEstSX(0x43);
Adafruit_INA219 sensoreEstDX(0x44);
Adafruit_INA219 sensoreTeensy(0x45);

Servo servoInternoSX;   // pitch
Servo servoInternoDX;   // pitch
Servo servoEsternoSX;   // pitch + roll
Servo servoEsternoDX;   // pitch + roll
Servo motore;

// PIN
const int PIN_ARIA         = A0;
const int PIN_TEMP_MOTORE  = A12;
const int PIN_INT_SX       = 6;
const int PIN_INT_DX       = 22;
const int PIN_EST_SX       = 23;
const int PIN_EST_DX       = 24;
const int PIN_MOTORE       = 10;
const int PIN_LED_ROSSO_ALARM = 2;   
const int PIN_LED_VERDE_GPS   = 3;   
const int PIN_LED_BLU_PID     = 4;   
const int PIN_BUZZER = 33;
const int PIN_RELE   = 20;

const int CENTRO_SERVO_deg  = 90;
const int IMU_CAMPIONI_TARA = 200;

//  COSTANTI FISICHE E DI CALIBRAZIONE SENSORI
const float R_SPECIFIC_ARIA              = 287.05f;  // Costante specifica dell'aria secca, J/(kg*K)
const float FATTORE_CONVERSIONE_PITOT_Pa = 3.22f;     // Da conteggi ADC a Pascal, dipende dal trasduttore usato
const float ALPHA_LIDAR                  = 0.25f;     // Coefficiente filtro EMA sul LIDAR (0-1, più alto = più reattivo)
const float COSTANTE_CALIBRAZIONE_OTTICA = 0.0012f;   // Costante di scala flusso ottico -> m/s
const int   CAMPIONI_CONFERMA_SCHIANTO   = 3;         // Cicli di loop consecutivi per confermare uno schianto

const unsigned long TEMPO_DECOLLO_SICURO_ms = 1500;   // ms sopra soglia velocità/altitudine prima di dichiarare "in volo"
const int MAX_TENTATIVI_INIT = 3;

// Limiti di validazione per i guadagni PID ricevuti da terra
const float LIMITE_KP_MAX = 10.0f;
const float LIMITE_KI_MAX = 2.0f;
const float LIMITE_KD_MAX = 5.0f;

//  VARIABILI GLOBALI — STATO INIZIALIZZAZIONE SENSORI
bool imuPronto         = false;
bool flussoOtticoOk    = false;
bool lidarOk           = false;
bool baroPronto        = false;
bool pitotCalibrato    = false;
bool sensoriCorrenteOk = true;   
int  tentativiInit     = 0;

//  VARIABILI GLOBALI — ARIA / PITOT (VELOCITÀ ARIA)

float DENSITA_ARIA_kgm3     = 1.225f;  // kg/m^3, aggiornata da pressione+temperatura del barometro
float PITOT_ZERO_adc        = 0.0f;    // Valore di zero calibrato del pitot, in conteggi ADC
int   PITOT_RAW_adc         = 0;       // Ultima lettura grezza pitot, in conteggi ADC (0-1023)
float PITOT_DIFFERENZA_adc  = 0.0f;    // Differenza tra lettura e zero, in conteggi ADC
bool  PITOT_VALIDO          = false;   // true se la differenza è positiva (pressione dinamica misurabile)
float VELOCITA_ARIA_ms      = 0.0f;    // Velocità relativa all'aria, dal Pitot, in m/s. NON viene mai mescolata con GPS o ottico.

//  VARIABILI GLOBALI — SUOLO / FLUSSO OTTICO (VELOCITÀ AL SUOLO)
float VELOCITA_SUOLO_ms          = 0.0f;   // m/s
float VELOCITA_OTTICA_X_ms       = 0.0f;   // Componente X stimata dal flusso ottico, in m/s (-1 = non valida)
float VELOCITA_OTTICA_Y_ms       = 0.0f;   // Componente Y stimata dal flusso ottico, in m/s (-1 = non valida)
int   FLUSSO_OTTICO_DX_conteggi  = 0;      // Conteggi grezzi di movimento ottico asse X (diagnostica)
int   FLUSSO_OTTICO_DY_conteggi  = 0;      // Conteggi grezzi di movimento ottico asse Y (diagnostica)

//  VARIABILI GLOBALI — ALTITUDINE

float ALTITUDINE_LIDAR_m      = -1.0f;  // Altitudine dal LIDAR TF-Luna, in metri (-1 = non disponibile/fuori range)
float ALTITUDINE_BARO_m       = 0.0f;   // Altitudine dal barometro (relativa al punto di decollo), in metri
float ALTITUDINE_m            = 0.0f;   // Altitudine effettivamente usata dal sistema, in metri
float TARA_ALTITUDINE_BARO_m  = 0.0f;   // Offset sottratto al barometro per azzerare l'altitudine al decollo, in metri
float PRESSIONE_BARO_Pa       = 0.0f;


//  VARIABILI GLOBALI — TEMPERATURE
float TEMPERATURA_MOTORE_C = 0.0f;
float TEMPERATURA_FUSOLIERA_C   = 0.0f;   

//  VARIABILI GLOBALI — NAVIGAZIONE
double TARGET_LAT_deg       = 41.902782;
double TARGET_LON_deg       = 12.496366;
float  ALTITUDINE_TARGET_m  = 40.0f;

float ROLL_TARGET_deg    = 0.0f;   // Target di rollio calcolato dalla guida L1
float DISTANZA_TARGET_m  = 0.0f;
float ROTTA_TARGET_deg   = 0.0f;   // Rotta (bearing) verso il target, 0=Nord/90=Est/180=Sud/270=Ovest
float ERRORE_ROTTA_deg   = 0.0f;   // Rotta target - rotta attuale, normalizzata in ±180°

//  VARIABILI GLOBALI — IMU
float OFFSET_ROLL_deg  = 0.0f;
float OFFSET_PITCH_deg = 0.0f;
float OFFSET_YAW_deg   = 0.0f;

float ACCEL_X_ms2 = 0.0f, ACCEL_Y_ms2 = 0.0f, ACCEL_Z_ms2 = 0.0f, ACCEL_TOTALE_ms2 = 0.0f;
float GYRO_X_degs = 0.0f, GYRO_Y_degs = 0.0f, GYRO_Z_degs = 0.0f;   // gradi/secondo (°/s)
uint8_t IMU_CAL_SYS = 0, IMU_CAL_GYRO = 0, IMU_CAL_ACCEL = 0, IMU_CAL_MAG = 0;   // 0-3

//  VARIABILI GLOBALI — ALIMENTAZIONE / CORRENTI
bool alimentazioneSicurezza = true;   
bool batteriaBassaMotore    = false;
bool batteriaBassaTeensy    = false;
bool releAttivato           = false;

float CORRENTE_SERVO_INT_SX_mA = 0.0f;
float CORRENTE_SERVO_INT_DX_mA = 0.0f;
float CORRENTE_SERVO_EST_SX_mA = 0.0f;
float CORRENTE_SERVO_EST_DX_mA = 0.0f;

//  VARIABILI GLOBALI — MOTORE / GAS (microsecondi, us)

int  GAS_LIMITE_TERMICO_us       = GAS_MASSIMO_us;
int  GAS_COMANDATO_PRE_LIMITE_us = GAS_NEUTRO_us;
bool limitazioneTermicaAttiva    = false;
bool limitazione_termica_gas        = true;   

//  VARIABILI GLOBALI — SERVI E SICUREZZA
bool Sicurezza_servo          = true;   // Abilita/disabilita la diagnostica di sicurezza sui servi
bool statoPrecedenteInterni  = true;
bool statoPrecedenteEsterni  = true;
bool estSxOk = true, estDxOk = true;
bool intSxOk = true, intDxOk = true;

bool schiantoSicurezza      = true;  
bool statoSchiantoRilevato  = false;
bool schiantoBloccato       = false;
bool droneInVolo            = false;
int  contatoreImpatto       = 0;
unsigned long TIMESTAMP_DECOLLO_ms = 0;

int global_modalitaVolo = 1;   // Modalità di volo corrente: 1=Manuale, 2=Auto (3=Failsafe gestito a parte)

float VELOCITA_CROCIERA_kmh      = 60.0f;
float VELOCITA_AVVICINAMENTO_kmh = 45.0f;

//  VARIABILI GLOBALI — PID (guadagni)
float Kp_vel = 1.5f,   Ki_vel = 0.1f,   Kd_vel = 0.5f;
float Kp_roll = 1.2f,  Ki_roll = 0.05f, Kd_roll = 0.5f;
float Kp_pitch = 1.2f, Ki_pitch = 0.05f, Kd_pitch = 0.5f;
float Kp_alt = 0.5f,   Ki_alt = 0.05f,  Kd_alt = 0.2f;

unsigned long TEMPO_PID_PRECEDENTE_ms = 0;

// --- Stato integrale/derivativo dei 4 PID ---
float PID_ALT_INTEGRALE = 0.0f,   PID_ALT_ERRORE_PRECEDENTE_m   = 0.0f;
float PID_PITCH_INTEGRALE = 0.0f, PID_PITCH_ERRORE_PRECEDENTE_deg = 0.0f;
float PID_ROLL_INTEGRALE = 0.0f,  PID_ROLL_ERRORE_PRECEDENTE_deg  = 0.0f;
float PID_VEL_INTEGRALE = 0.0f,   PID_VEL_ERRORE_PRECEDENTE_kmh   = 0.0f;

// --- Diagnostica PID (salvata ad ogni ciclo calcolaPID, per telemetria TEL3) ---
float PID_ALT_ERRORE_m = 0, PID_ALT_P = 0, PID_ALT_I = 0, PID_ALT_D = 0;
float PID_PITCH_TARGET_AUTO_deg = 0;
float PID_PITCH_ERRORE_deg = 0, PID_PITCH_P = 0, PID_PITCH_I = 0, PID_PITCH_D = 0;
float PID_ROLL_ERRORE_deg = 0,  PID_ROLL_P = 0,  PID_ROLL_I = 0,  PID_ROLL_D = 0;
float PID_VEL_ERRORE_kmh = 0,   PID_VEL_P = 0,   PID_VEL_I = 0,   PID_VEL_D = 0;
float VELOCITA_TARGET_ATTUALE_kmh = 0.0f;

//  VARIABILI GLOBALI — RICEVENTE RC / TELEMETRIA / COMANDI
uint16_t canaliRC[16];
bool failsafe      = false;
bool pacchettoPerso = false;

unsigned long TIMER_TELEMETRIA_ms      = 0;
unsigned long TIMER_TELEMETRIA_DIAG_ms = 0;
unsigned long numeroPacchettoTEL1      = 0;
uint8_t contatorePacchettoDiag         = 0;   // cicla 0..2 tra TEL2 / TEL3 / TEL4
bool forzaInvioDiagnostica             = false;


//  PROTOTIPI
void segnalaOK();
void segnalaErrore();
void segnalaCalibrazione(int pin_led);
void inizializzaServo();
void inizializzaMotore();

void leggiPitot();
void leggiBarometro();
void leggiTemperaturaMotore();
void aggiornaLidar();
void leggiVelocitaOttica(float yaw_deg);

void aggiornaDensitaAria(float pressione_pa, float temperatura_c);
void selezionaAltitudine();
void aggiornaVelocitaSuolo(float velocitaSuoloGps_ms);
void aggiornaNavigazione(float yaw_deg);
void calcolaPID(float targetAltitudine_m, float targetRoll_deg,
                 float pitchReale_deg, float rollReale_deg,
                 float velocitaAria_kmh, float targetVelocita_kmh,
                 int gasDiBase_us,
                 int &comandoPitchOut_deg, int &comandoRollOut_deg, int &comandoGasOut_us);
int  gasMaxTermico();
void resettaPID();

void applicaMixer4Servi(int pitch_deg, int roll_deg);
void diagnosticaServi();
void gestisciAlimentazione();
void gestisciSchianto();
void verificaDroneInVolo();
void gestisciAllarmi();
void aggiornaDiagnosticaIMU();

void inviaTelemetria(float pitch_deg, float roll_deg, float yaw_deg,
                      float velAria_kmh, float velSuoloGps_kmh,
                      int outPitch_deg, int outRoll_deg, int outGas_us);
void inviaAck(const String& campo, const String& valore);
void inviaNack(const String& campo, const String& motivo);
void comandiDaTerra();
void elaboraComando(const String& cmd);



//  FUNZIONI DI SEGNALAZIONE (LED / BUZZER)

// 1200 Hz per 150 ms con LED verde acceso
void segnalaOK() {
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    tone(PIN_BUZZER, 1200, 150);   // 1200 Hz, 150 ms
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
}
// 400 Hz per 100 ms con LED rosso lampeggiante 3 volte
void segnalaErrore() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        tone(PIN_BUZZER, 400, 100);   // 400 Hz, 100 ms
        delay(150);
        digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        delay(100);
    }
}
// 1000 Hz per 30 ms con LED lampeggiante (toggle) sul pin specificato
void segnalaCalibrazione(int pin_led) {
    digitalWrite(pin_led, !digitalRead(pin_led));  // toggle
    tone(PIN_BUZZER, 1000, 30);                    // Beep breve 1000 Hz, 30 ms
}

//  FUNZIONI DI LETTURA SENSORI

void leggiPitot() {
    int lettura_adc = constrain(analogRead(PIN_ARIA), 0, 1023);
    float differenza_adc = (float)lettura_adc - PITOT_ZERO_adc; // sottrae tara
    PITOT_VALIDO         = (differenza_adc > 0.0f);
    if (PITOT_VALIDO) {
        VELOCITA_ARIA_ms = sqrtf((2.0f * differenza_adc * FATTORE_CONVERSIONE_PITOT_Pa) / DENSITA_ARIA_kgm3);   // v = sqrt(2*p/rho)
    } else {
        VELOCITA_ARIA_ms = -1.0f;  
    }
}

void leggiBarometro() {
    ALTITUDINE_BARO_m  = barometro.readAltitude(1013.25f) - TARA_ALTITUDINE_BARO_m;
    PRESSIONE_BARO_Pa  = barometro.pressure;
    TEMPERATURA_FUSOLIERA_C = barometro.temperature;
}

void leggiTemperaturaMotore() {
    float voltaggioSensore_V = analogRead(PIN_TEMP_MOTORE) * (3.3f / 1023.0f);   // assumendo Vref 3.3V
    TEMPERATURA_MOTORE_C = (voltaggioSensore_V - 0.5f) * 100.0f;
}

void aggiornaLidar() {
    if (ALTITUDINE_BARO_m > ALTITUDINE_MAX_LIDAR_m) {  // Se il barometro è sopra la quota massima utile del LIDAR, non leggere più il LIDAR
        while (Serial2.available()) {
            Serial2.read();   // Svuota il buffer, scarta i dati
        }
        ALTITUDINE_LIDAR_m = -1.0f;
        return;
    }

    static uint8_t buffer[9];

    while (Serial2.available() >= 9) {
        if (Serial2.read() == 0x59 && Serial2.peek() == 0x59) {
            Serial2.read();
            buffer[0] = 0x59;
            buffer[1] = 0x59;
            for (int i = 2; i < 9; i++) {
                buffer[i] = Serial2.read();
            }

            // Checksum: somma degli 8 byte precedenti deve corrispondere al 9° byte
            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++) {
                checksum += buffer[i];
            }
            if (checksum != buffer[8]) continue;

            uint16_t distanza_cm = buffer[2] | ((uint16_t)buffer[3] << 8);
            float distanza_m = distanza_cm / 100.0f;

            // Filtro passa-basso (EMA)
            if (ALTITUDINE_LIDAR_m < 0.0f) {
                ALTITUDINE_LIDAR_m = distanza_m;
            } else {
                ALTITUDINE_LIDAR_m = ALPHA_LIDAR * distanza_m + (1.0f - ALPHA_LIDAR) * ALTITUDINE_LIDAR_m;
            }
            return;
        }
    }
}


void leggiVelocitaOttica(float yaw_deg) {
    static unsigned long tempoPrecedente_ms = 0;
    int dx = 0;
    int dy = 0;
    flussoOttico.readMotionCount(&dx, &dy);
    FLUSSO_OTTICO_DX_conteggi = dx;
    FLUSSO_OTTICO_DY_conteggi = dy;

    unsigned long adesso_ms = millis();
    float dt_s = (adesso_ms - tempoPrecedente_ms) / 1000.0f;
    tempoPrecedente_ms = adesso_ms;

    if (dt_s <= 0.0f || ALTITUDINE_m > ALTITUDINE_MAX_OTTICO_m) {
        VELOCITA_OTTICA_X_ms = -1.0f;   // -1 = valore non valido
        VELOCITA_OTTICA_Y_ms = -1.0f;
        return;
    }

    float vX_ms = (dx * COSTANTE_CALIBRAZIONE_OTTICA * ALTITUDINE_m) / dt_s;
    float vY_ms = (dy * COSTANTE_CALIBRAZIONE_OTTICA * ALTITUDINE_m) / dt_s;

    float yaw_rad = radians(yaw_deg);
    VELOCITA_OTTICA_X_ms = vX_ms * cos(yaw_rad) - vY_ms * sin(yaw_rad);
    VELOCITA_OTTICA_Y_ms = vX_ms * sin(yaw_rad) + vY_ms * cos(yaw_rad);
}

// Calibrazione/lettura diagnostica estesa dell'IMU (chiamata a bassa frequenza, dentro TEL4)
void aggiornaDiagnosticaIMU() {
    giroscopio.getCalibration(&IMU_CAL_SYS, &IMU_CAL_GYRO, &IMU_CAL_ACCEL, &IMU_CAL_MAG);
    imu::Vector<3> gyro = giroscopio.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);   // gradi/secondo (°/s)
    GYRO_X_degs = gyro.x();
    GYRO_Y_degs = gyro.y();
    GYRO_Z_degs = gyro.z();
}



//  FUNZIONI DI CALCOLO

void aggiornaDensitaAria(float pressione_pa, float temperatura_c) {
    float temperatura_K = temperatura_c + 273.15f;
    if (temperatura_K > 0.0f && pressione_pa > 0.0f) {
        DENSITA_ARIA_kgm3 = pressione_pa / (R_SPECIFIC_ARIA * temperatura_K);
    } else {
        DENSITA_ARIA_kgm3 = 1.225f;   // Valore di default al livello del mare
    }
}


void selezionaAltitudine() {
    if (lidarOk && ALTITUDINE_LIDAR_m > 0.0f && ALTITUDINE_BARO_m < ALTITUDINE_MAX_LIDAR_m) {
        ALTITUDINE_m = ALTITUDINE_LIDAR_m;
    } else {
        ALTITUDINE_m = ALTITUDINE_BARO_m;
    }
}

void aggiornaVelocitaSuolo(float velocitaSuoloGps_ms) {
    if (VELOCITA_OTTICA_X_ms != -1.0f && VELOCITA_OTTICA_Y_ms != -1.0f
        && ALTITUDINE_m < ALTITUDINE_MAX_OTTICO_m) {

        VELOCITA_SUOLO_ms = sqrtf(VELOCITA_OTTICA_X_ms * VELOCITA_OTTICA_X_ms +
                                   VELOCITA_OTTICA_Y_ms * VELOCITA_OTTICA_Y_ms);

    } else if (gps.speed.isValid()) {
        VELOCITA_SUOLO_ms = velocitaSuoloGps_ms;
    } else {
        VELOCITA_SUOLO_ms = -1.0f;
    }
}

//  NAVIGAZIONE GPS (guida L1) ----------------------------------------------------
void aggiornaNavigazione(float yaw_deg) {
    if (gps.location.isValid()) {
        // Geometria verso il target: distanza (m) e rotta (°)
        DISTANZA_TARGET_m = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LAT_deg, TARGET_LON_deg);
        ROTTA_TARGET_deg  = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LAT_deg, TARGET_LON_deg);

        float velocitaPerCalcolo_ms = max(VELOCITA_SUOLO_ms, 1.0f);
        float L1_m = max(velocitaPerCalcolo_ms * 4.0f, 1.0f);
        float raggioAccettazioneDinamico_m = max(RAGGIO_ACCETTAZIONE_MINIMO_m, L1_m * 0.75f);

        if (DISTANZA_TARGET_m <= raggioAccettazioneDinamico_m) {
            Serial.println("WAYPOINT RAGGIUNTO! Inizializza passaggio al prossimo target...");
            DISTANZA_TARGET_m = 0.0f;
            return;
        }

        // Rotta reale: se la velocità al suolo è < 2 m/s il GPS è impreciso sulla direzione (course),
        // quindi si usa la bussola (yaw) dell'IMU
        float rottaAttuale_deg;
        if (gps.course.isValid() && VELOCITA_SUOLO_ms > 2.0f) {
            rottaAttuale_deg = gps.course.deg();
        } else {
            rottaAttuale_deg = yaw_deg;
        }

        ERRORE_ROTTA_deg = ROTTA_TARGET_deg - rottaAttuale_deg;

        // Normalizzazione a ±180° (via più breve per girare)
        if (ERRORE_ROTTA_deg > 180.0f) {
            ERRORE_ROTTA_deg -= 360.0f;
        } else if (ERRORE_ROTTA_deg < -180.0f) {
            ERRORE_ROTTA_deg += 360.0f;
        }

        // Guida L1: accelerazione laterale necessaria per curvare verso la rotta target
        // a_lat = 2*V^2/L1 * sin(eta)
        float eta_rad = radians(ERRORE_ROTTA_deg);
        float aLaterale_ms2 = (2.0f * velocitaPerCalcolo_ms * velocitaPerCalcolo_ms / L1_m) * sin(eta_rad);
        float rollNecessario_rad = atan(aLaterale_ms2 / 9.81f);
        ROLL_TARGET_deg = constrain(degrees(rollNecessario_rad), -MAX_ROLL_deg, MAX_ROLL_deg);

        Serial.print("Dist WP: ");
        Serial.print(DISTANZA_TARGET_m);
        Serial.print("m | Rotta Target: ");
        Serial.print(ROTTA_TARGET_deg);
        Serial.print("° | Err: ");
        Serial.print(ERRORE_ROTTA_deg);
        Serial.print("° | Target Roll: ");
        Serial.println(ROLL_TARGET_deg);

    } else {
        Serial.println("GPS: In attesa di segnale valido (FIX 3D)...");
    }
}

//  CALCOLO PID (quota -> pitch target -> comando pitch, roll, gas) ---------------
void calcolaPID(float targetAltitudine_m, float targetRoll_deg,
                 float pitchReale_deg, float rollReale_deg,
                 float velocitaAria_kmh, float targetVelocita_kmh,
                 int gasDiBase_us,
                 int &comandoPitchOut_deg, int &comandoRollOut_deg, int &comandoGasOut_us)
{
    // 1. CALCOLO DEL TEMPO
    unsigned long tempoAttuale_ms = millis();
    float dt_s = (tempoAttuale_ms - TEMPO_PID_PRECEDENTE_ms) / 1000.0f;

    if (dt_s <= 0.001f) return;   // Evita divisioni per zero
    if (dt_s > 0.5f) dt_s = 0.5f; // Evita lag improvvisi: limita dt max a 0.5 s
    TEMPO_PID_PRECEDENTE_ms = tempoAttuale_ms;

    VELOCITA_TARGET_ATTUALE_kmh = targetVelocita_kmh;

    // 2. PROTEZIONE STALLO / OVERSPEED (basata sulla velocità ARIA, unica grandezza aerodinamicamente corretta)
    static bool inStallo = false;
    static bool inOverspeed = false;

    if (!inStallo) {
        inStallo = (velocitaAria_kmh < VELOCITA_STALLO_X8_kmh);
    } else {
        inStallo = (velocitaAria_kmh < VELOCITA_STALLO_X8_kmh + MARGINE_ISTERESI_kmh);
    }

    if (!inOverspeed) {
        inOverspeed = (velocitaAria_kmh > MAX_AIRSPEED_X8_kmh);
    } else {
        inOverspeed = (velocitaAria_kmh > MAX_AIRSPEED_X8_kmh - MARGINE_ISTERESI_kmh);
    }

    if (inStallo) inOverspeed = false;

    // 3. PID ALTITUDINE (bypassato se in stallo/overspeed/fuori range: il pitch è dettato dal recupero)
    float targetPitchAuto_deg = 0.0f;
    int gasCorrente_us = gasDiBase_us;

    if (inStallo) {
        targetPitchAuto_deg = PITCH_DOWN_FORZATO_deg;
        gasCorrente_us = GAS_MASSIMO_us;
        resettaPID();

    } else if (inOverspeed) {
        targetPitchAuto_deg = -PITCH_DOWN_FORZATO_deg;
        gasCorrente_us = GAS_MINIMO_us;
        resettaPID();

    } else if (ALTITUDINE_m > ALTITUDINE_MAX_m) {
        // Sopra la quota massima: forza un pitch negativo (scendi) e riduce il gas al minimo
        targetPitchAuto_deg = PITCH_DOWN_FORZATO_deg;
        gasCorrente_us = GAS_MINIMO_us;
        resettaPID();

    } else if (ALTITUDINE_m < ALTITUDINE_MIN_m) {
        // Sotto la quota minima: forza un pitch positivo (sali) e aumenta il gas quasi al massimo
        targetPitchAuto_deg = PITCH_UP_FORZATO_deg;
        gasCorrente_us = GAS_MASSIMO_us;
        resettaPID();

    } else {
        // Quota nel range ammesso, nessuna emergenza velocità: calcolo PID normale
        float erroreAltitudine_m = targetAltitudine_m - ALTITUDINE_m;
        erroreAltitudine_m = constrain(erroreAltitudine_m, -20.0f, 20.0f);   // Limitato a ±20 m

        float P_alt = Kp_alt * erroreAltitudine_m;

        PID_ALT_INTEGRALE += erroreAltitudine_m * dt_s;
        PID_ALT_INTEGRALE  = constrain(PID_ALT_INTEGRALE, -20.0f, 20.0f);   // Anti-windup
        float I_alt = Ki_alt * PID_ALT_INTEGRALE;

        float D_alt = Kd_alt * ((erroreAltitudine_m - PID_ALT_ERRORE_PRECEDENTE_m) / dt_s);
        PID_ALT_ERRORE_PRECEDENTE_m = erroreAltitudine_m;

        targetPitchAuto_deg = constrain(P_alt + I_alt + D_alt, -10.0f, 15.0f);

        PID_ALT_ERRORE_m = erroreAltitudine_m; PID_ALT_P = P_alt; PID_ALT_I = I_alt; PID_ALT_D = D_alt;
    }
    PID_PITCH_TARGET_AUTO_deg = targetPitchAuto_deg;

    // 4. PID PITCH — insegue il target (normale o di emergenza) calcolato sopra
    float errorePitch_deg = targetPitchAuto_deg - pitchReale_deg;

    float P_pitch = Kp_pitch * errorePitch_deg;

    PID_PITCH_INTEGRALE += errorePitch_deg * dt_s;
    PID_PITCH_INTEGRALE  = constrain(PID_PITCH_INTEGRALE, -40.0f, 40.0f);
    float I_pitch = Ki_pitch * PID_PITCH_INTEGRALE;

    float D_pitch = Kd_pitch * ((errorePitch_deg - PID_PITCH_ERRORE_PRECEDENTE_deg) / dt_s);
    PID_PITCH_ERRORE_PRECEDENTE_deg = errorePitch_deg;

    comandoPitchOut_deg = (int)(P_pitch + I_pitch + D_pitch);
    comandoPitchOut_deg = constrain(comandoPitchOut_deg, -MAX_PITCH_deg, MAX_PITCH_deg);

    PID_PITCH_ERRORE_deg = errorePitch_deg; PID_PITCH_P = P_pitch; PID_PITCH_I = I_pitch; PID_PITCH_D = D_pitch;

    // 5. PID ROLL — insegue il target di rollio dalla guida L1
    float erroreRoll_deg = targetRoll_deg - rollReale_deg;

    float P_roll = Kp_roll * erroreRoll_deg;

    PID_ROLL_INTEGRALE += erroreRoll_deg * dt_s;
    PID_ROLL_INTEGRALE  = constrain(PID_ROLL_INTEGRALE, -40.0f, 40.0f);
    float I_roll = Ki_roll * PID_ROLL_INTEGRALE;

    float D_roll = Kd_roll * ((erroreRoll_deg - PID_ROLL_ERRORE_PRECEDENTE_deg) / dt_s);
    PID_ROLL_ERRORE_PRECEDENTE_deg = erroreRoll_deg;

    comandoRollOut_deg = (int)(P_roll + I_roll + D_roll);
    comandoRollOut_deg = constrain(comandoRollOut_deg, -MAX_ROLL_deg, MAX_ROLL_deg);

    PID_ROLL_ERRORE_deg = erroreRoll_deg; PID_ROLL_P = P_roll; PID_ROLL_I = I_roll; PID_ROLL_D = D_roll;

    if (inStallo || inOverspeed) {
        comandoGasOut_us = gasCorrente_us;
        resettaPID();
        return;
    }

    // 6. PID VELOCITÀ (ARIA) — regola il gas attorno al gas di base
    float erroreVel_kmh = targetVelocita_kmh - velocitaAria_kmh;

    float P_vel = Kp_vel * erroreVel_kmh;

    PID_VEL_INTEGRALE += erroreVel_kmh * dt_s;
    PID_VEL_INTEGRALE  = constrain(PID_VEL_INTEGRALE, -30.0f, 30.0f);
    float I_vel = Ki_vel * PID_VEL_INTEGRALE;

    float D_vel = Kd_vel * ((erroreVel_kmh - PID_VEL_ERRORE_PRECEDENTE_kmh) / dt_s);
    PID_VEL_ERRORE_PRECEDENTE_kmh = erroreVel_kmh;

    int gasCalcolato_us = gasCorrente_us + (int)(P_vel + I_vel + D_vel);
    comandoGasOut_us = constrain(gasCalcolato_us, GAS_MINIMO_us, GAS_MASSIMO_us);

    PID_VEL_ERRORE_kmh = erroreVel_kmh; PID_VEL_P = P_vel; PID_VEL_I = I_vel; PID_VEL_D = D_vel;
}

// Determina il limite di gas per la protezione termica del motore (interpolazione lineare tra le due soglie)
int gasMaxTermico() {
    if (TEMPERATURA_MOTORE_C <= MIN_THROTTLE_START_TEMP_C) {
        return GAS_MASSIMO_us;   // Sotto la soglia minima: nessun limite
    }
    else if (TEMPERATURA_MOTORE_C >= MAX_THROTTLE_END_TEMP_C) {
        return GAS_MINIMO_us;    // Sopra la soglia massima: limite al minimo
    }else {

    float fattoreInterpolazione = (TEMPERATURA_MOTORE_C - MIN_THROTTLE_START_TEMP_C) /
                                   (MAX_THROTTLE_END_TEMP_C - MIN_THROTTLE_START_TEMP_C);   // 0-1
    int limite_us = (int)(GAS_MASSIMO_us - fattoreInterpolazione * (GAS_MASSIMO_us - GAS_MINIMO_us));
    return limite_us;
    }
}

// Azzera tutti gli stati integrali/derivativi dei 4 PID e il riferimento temporale dt
void resettaPID() {
    PID_ALT_INTEGRALE   = 0.0f;  PID_ALT_ERRORE_PRECEDENTE_m   = 0.0f;
    PID_PITCH_INTEGRALE = 0.0f;  PID_PITCH_ERRORE_PRECEDENTE_deg = 0.0f;
    PID_ROLL_INTEGRALE  = 0.0f;  PID_ROLL_ERRORE_PRECEDENTE_deg  = 0.0f;
    PID_VEL_INTEGRALE   = 0.0f;  PID_VEL_ERRORE_PRECEDENTE_kmh   = 0.0f;
    TEMPO_PID_PRECEDENTE_ms = millis();
}


//  FUNZIONI DI CONTROLLO

void applicaMixer4Servi(int pitch_deg, int roll_deg) {
    int posIntSX_deg = CENTRO_SERVO_deg;
    int posIntDX_deg = CENTRO_SERVO_deg;
    int posEstSX_deg = CENTRO_SERVO_deg;
    int posEstDX_deg = CENTRO_SERVO_deg;

    // Batteria Teensy bassa: (disattiva i servi interni)
    if (batteriaBassaTeensy) {
        intSxOk = false;
        intDxOk = false;
    }

    bool esterniAttivi = estSxOk && estDxOk;
    bool interniAttivi = intSxOk && intDxOk;

    if (esterniAttivi && interniAttivi) {
        // Caso A: tutto OK — interni = SOLO PITCH, esterni = SOLO ROLL
        posIntSX_deg = CENTRO_SERVO_deg + pitch_deg;
        posIntDX_deg = CENTRO_SERVO_deg + pitch_deg;
        posEstSX_deg = CENTRO_SERVO_deg + roll_deg;
        posEstDX_deg = CENTRO_SERVO_deg - roll_deg;
    } else if (esterniAttivi && !interniAttivi) {
        // Caso B: interni rotti — esterni fanno pitch + roll
        posEstSX_deg = CENTRO_SERVO_deg + pitch_deg + roll_deg;
        posEstDX_deg = CENTRO_SERVO_deg + pitch_deg - roll_deg;
    } else if (!esterniAttivi && interniAttivi) {
        // Caso C: esterni rotti — interni fanno pitch + roll
        posIntSX_deg = CENTRO_SERVO_deg + pitch_deg + roll_deg;
        posIntDX_deg = CENTRO_SERVO_deg + pitch_deg - roll_deg;
    } else {
        return;   // Tutti i servi rotti: nulla da comandare
    }

    // Attach/detach automatico in base a se i servi sono considerati attivi o no
    if (interniAttivi != statoPrecedenteInterni) {
        if (interniAttivi) {
            servoInternoSX.attach(PIN_INT_SX);
            servoInternoDX.attach(PIN_INT_DX);
            Serial.println("Servi interni: ATTIVATI");
        } else {
            servoInternoSX.detach();
            servoInternoDX.detach();
            Serial.println("Servi interni: STACCATI");
        }
        statoPrecedenteInterni = interniAttivi;
    }

    if (esterniAttivi != statoPrecedenteEsterni) {
        if (esterniAttivi) {
            servoEsternoSX.attach(PIN_EST_SX);
            servoEsternoDX.attach(PIN_EST_DX);
            Serial.println("Servi esterni: ATTIVATI");
        } else {
            servoEsternoSX.detach();
            servoEsternoDX.detach();
            Serial.println("Servi esterni: STACCATI");
        }
        statoPrecedenteEsterni = esterniAttivi;
    }

    // Limiti di sicurezza meccanici: 45°-135° (±45° dal centro 90°)
    posIntSX_deg = constrain(posIntSX_deg, 45, 135);
    posIntDX_deg = constrain(posIntDX_deg, 45, 135);
    posEstSX_deg = constrain(posEstSX_deg, 45, 135);
    posEstDX_deg = constrain(posEstDX_deg, 45, 135);

    if (interniAttivi) {
        servoInternoSX.write(posIntSX_deg);
        servoInternoDX.write(posIntDX_deg);
    }
    if (esterniAttivi) {
        servoEsternoSX.write(posEstSX_deg);
        servoEsternoDX.write(posEstDX_deg);
    }
}

//  DIAGNOSTICA SERVI ---------------------------------------------------------------
void diagnosticaServi() {
    if (!Sicurezza_servo) {
        estSxOk = estDxOk = intSxOk = intDxOk = true;
        return;
    }

    static int erroriConsecutivi[4] = {0, 0, 0, 0};   // [EstSX, EstDX, IntSX, IntDX]
    float corrente_mA = 0.0f;

    // Servo Esterno SX
    corrente_mA = sensoreEstSX.getCurrent_mA();
    CORRENTE_SERVO_EST_SX_mA = corrente_mA;
    if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
        erroriConsecutivi[0]++;
        if (erroriConsecutivi[0] > ERRORI_CONSECUTIVI_SERVO) {
            estSxOk = false;
            Serial.println("WARN: ServoEstSX anomalia corrente persistente!");
        }
    } else {
        erroriConsecutivi[0] = 0;
        estSxOk = true;
    }

    // Servo Esterno DX
    corrente_mA = sensoreEstDX.getCurrent_mA();
    CORRENTE_SERVO_EST_DX_mA = corrente_mA;
    if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
        erroriConsecutivi[1]++;
        if (erroriConsecutivi[1] > ERRORI_CONSECUTIVI_SERVO) {
            estDxOk = false;
            Serial.println("WARN: ServoEstDX anomalia corrente persistente!");
        }
    } else {
        erroriConsecutivi[1] = 0;
        estDxOk = true;
    }

    // Servo Interno SX
    corrente_mA = sensoreIntSX.getCurrent_mA();
    CORRENTE_SERVO_INT_SX_mA = corrente_mA;
    if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
        erroriConsecutivi[2]++;
        if (erroriConsecutivi[2] > ERRORI_CONSECUTIVI_SERVO) {
            intSxOk = false;
            Serial.println("WARN: ServoIntSX anomalia corrente persistente!");
        }
    } else {
        erroriConsecutivi[2] = 0;
        intSxOk = true;
    }

    // Servo Interno DX
    corrente_mA = sensoreIntDX.getCurrent_mA();
    CORRENTE_SERVO_INT_DX_mA = corrente_mA;
    if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
        erroriConsecutivi[3]++;
        if (erroriConsecutivi[3] > ERRORI_CONSECUTIVI_SERVO) {
            intDxOk = false;
            Serial.println("WARN: ServoIntDX anomalia corrente persistente!");
        }
    } else {
        erroriConsecutivi[3] = 0;
        intDxOk = true;
    }
}

void gestisciAlimentazione() {
    if (!alimentazioneSicurezza) {
        batteriaBassaTeensy = false;
        batteriaBassaMotore = false;
        return;
    }
    float vTeensy = sensoreTeensy.getBusVoltage_V();
    float vMotore = sensoreMotore.getBusVoltage_V();

    // Batteria Teensy
    batteriaBassaTeensy = (vTeensy < VALORE_BATT_TEENSY_BASSA_V);
    if (batteriaBassaTeensy) {
        Serial.println("WARN: Batteria Teensy bassa");
        if (!releAttivato) {
            digitalWrite(PIN_RELE, HIGH);   // Attiva il relè: la batteria motore subentra ad alimentare l'elettronica
            releAttivato = true;
            Serial.println(">>> FAILOVER: Rele' attivato, subentra batteria motore");
        }
    }

    // Batteria motore
    batteriaBassaMotore = (vMotore < VALORE_BATT_MOTORE_BASSA_V);
}

void gestisciSchianto() {
    if (!schiantoSicurezza) {
        statoSchiantoRilevato = false;
        return;
    }
    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO_us);   // Se già rilevato uno schianto, tiene il motore forzatamente spento
        return;
    }

    if (!droneInVolo) return;   // Non controlla schianti se il drone non è ancora considerato in volo (evita falsi positivi a terra)

    imu::Vector<3> accel = giroscopio.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // m/s^2
    float accelerazioneTotale_ms2 = sqrt((accel.x() * accel.x()) + (accel.y() * accel.y()) + (accel.z() * accel.z()));

    // Salvataggio per diagnostica/telemetria
    ACCEL_X_ms2 = accel.x();
    ACCEL_Y_ms2 = accel.y();
    ACCEL_Z_ms2 = accel.z();
    ACCEL_TOTALE_ms2 = accelerazioneTotale_ms2;

    if (accelerazioneTotale_ms2 > SOGLIA_ACCELERAZIONE_SCHIANTO_ms2) {
        contatoreImpatto++;
        if (contatoreImpatto >= CAMPIONI_CONFERMA_SCHIANTO) {   // Confermato dopo N cicli di loop consecutivi
            statoSchiantoRilevato = true;
            schiantoBloccato = true;
            contatoreImpatto = 0;
            motore.writeMicroseconds(GAS_NEUTRO_us);
            servoInternoSX.detach();
            servoInternoDX.detach();
            servoEsternoSX.detach();
            servoEsternoDX.detach();

            digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
            digitalWrite(PIN_LED_VERDE_GPS, HIGH);
            digitalWrite(PIN_LED_BLU_PID, HIGH);
            tone(PIN_BUZZER, 2000);
        }
    } else {
        contatoreImpatto = 0;
    }
}

void verificaDroneInVolo() {
    if (!droneInVolo) {
        bool velocitaSufficiente   = (VELOCITA_ARIA_ms > SOGLIA_VELOCITA_DECOLLO_ms);
        bool altitudineSufficiente = (ALTITUDINE_m > SOGLIA_ALTITUDINE_DECOLLO_m);

        if (velocitaSufficiente && altitudineSufficiente) {
            if (TIMESTAMP_DECOLLO_ms == 0) {
                TIMESTAMP_DECOLLO_ms = millis();
            }
            if (millis() - TIMESTAMP_DECOLLO_ms >= TEMPO_DECOLLO_SICURO_ms) {
                droneInVolo = true;
                TIMESTAMP_DECOLLO_ms = 0;
            }
        } else {
            TIMESTAMP_DECOLLO_ms = 0;
        }
    }
}

//  GESTIONE LUCI DI STATO -------------------------------------------------------------
void gestisciAllarmi() {
    // 1. EMERGENZA CRITICA: guasto servi (tutti i LED accesi fissi come segnale univoco di emergenza)
    if (!estSxOk || !estDxOk || !intSxOk || !intDxOk) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        digitalWrite(PIN_LED_VERDE_GPS, HIGH);
        digitalWrite(PIN_LED_BLU_PID, HIGH);
        return;
    }
    // 2. LED ROSSO: allarmi (batteria motore bassa o failsafe radio)
    digitalWrite(PIN_LED_ROSSO_ALARM, (batteriaBassaMotore || failsafe) ? HIGH : LOW);

    // 3. LED VERDE: stato fix GPS
    digitalWrite(PIN_LED_VERDE_GPS, gps.location.isValid() ? HIGH : LOW);

    // 4. LED BLU: modalità di volo (acceso = AUTO)
    digitalWrite(PIN_LED_BLU_PID, (global_modalitaVolo == 2) ? HIGH : LOW);
}

// Invia un ACK (comando accettato) sia via LoRa che via USB, nel formato "ACK:campo:valore"
void inviaAck(const String& campo, const String& valore) {
    TELEMETRIA.print("ACK:"); TELEMETRIA.print(campo); TELEMETRIA.print(":"); TELEMETRIA.println(valore);
    Serial.print("ACK:");     Serial.print(campo);     Serial.print(":");     Serial.println(valore);
}

// Invia un NACK (comando rifiutato, con motivo) sia via LoRa che via USB, nel formato "NACK:campo:motivo"
void inviaNack(const String& campo, const String& motivo) {
    TELEMETRIA.print("NACK:"); TELEMETRIA.print(campo); TELEMETRIA.print(":"); TELEMETRIA.println(motivo);
    Serial.print("NACK:");     Serial.print(campo);     Serial.print(":");     Serial.println(motivo);
}

//  INVIO TELEMETRIA COMPLETA (formato CSV) -----------------------------------------
//
// TEL1 ("$,")  -> stato + assetto + navigazione + batterie + servi (FORMATO ORIGINALE,
//                 invariato, stessa frequenza di prima ~2Hz, per non rompere il parser
//                 GCS esistente). In coda ci sono anche timestamp e contatore pacchetto.
// TEL2 ("$2,") -> correnti/potenze batterie + dettaglio flusso ottico grezzo + gas pre-limite
// TEL3 ("$3,") -> diagnostica PID completa
// TEL4 ("$4,") -> GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC estesi (4-16)
//
// TEL2/3/4 vengono inviati in round-robin ogni 2 secondi circa (uno alla volta,
// alternati), per non appesantire la banda LoRa. REQ_DIAG forza l'invio immediato del prossimo del ciclo.
void inviaTelemetria(float pitch_deg, float roll_deg, float yaw_deg,
                      float velAria_kmh, float velSuoloGps_kmh,
                      int outPitch_deg, int outRoll_deg, int outGas_us) {

    unsigned long tempoAttuale_ms = millis();

    // Invio TEL1 a 2 Hz (ogni 500 ms) per non saturare la banda radio LoRa
    if (tempoAttuale_ms - TIMER_TELEMETRIA_ms > 500) {
        TIMER_TELEMETRIA_ms = tempoAttuale_ms;

        // Letture voltaggi in tempo reale (Volt, V)
        float vBatt   = sensoreMotore.getBusVoltage_V();
        float vTeensy = sensoreTeensy.getBusVoltage_V();
        float vIntSX  = sensoreIntSX.getBusVoltage_V();
        float vIntDX  = sensoreIntDX.getBusVoltage_V();
        float vEstSX  = sensoreEstSX.getBusVoltage_V();
        float vEstDX  = sensoreEstDX.getBusVoltage_V();

        int codiceAllarme = 0;   // Bitmask allarmi globali
        if (failsafe)             codiceAllarme += 1;   // Bit 0
        if (batteriaBassaMotore)  codiceAllarme += 2;   // Bit 1
        if (releAttivato)         codiceAllarme += 4;   // Bit 2
        if (batteriaBassaTeensy)  codiceAllarme += 8;   // Bit 3
        if (statoSchiantoRilevato) codiceAllarme += 16; // Bit 4
        if (droneInVolo)          codiceAllarme += 32;  // Bit 5

        TELEMETRIA.print("$,"); // 0. Start indicatore pacchetto TEL1

        // --- STATO E ALLARMI ---
        TELEMETRIA.print(global_modalitaVolo); TELEMETRIA.print(","); // 1. Modalità (1=Manuale, 2=Auto, 3=Failsafe)
        TELEMETRIA.print(codiceAllarme);       TELEMETRIA.print(","); // 2. Bitmask allarmi globali

        // --- ALIMENTAZIONE (Volt, V) ---
        TELEMETRIA.print(vBatt, 2);   TELEMETRIA.print(","); // 3. V Motore
        TELEMETRIA.print(vTeensy, 2); TELEMETRIA.print(","); // 4. V Teensy
        TELEMETRIA.print(vIntSX, 2);  TELEMETRIA.print(","); // 5. V Servo Int SX
        TELEMETRIA.print(vIntDX, 2);  TELEMETRIA.print(","); // 6. V Servo Int DX
        TELEMETRIA.print(vEstSX, 2);  TELEMETRIA.print(","); // 7. V Servo Est SX
        TELEMETRIA.print(vEstDX, 2);  TELEMETRIA.print(","); // 8. V Servo Est DX

        // --- STATO SALUTE SERVI (4 cifre "1"/"0" concatenate) ---
        TELEMETRIA.print(intSxOk ? "1" : "0");
        TELEMETRIA.print(intDxOk ? "1" : "0");
        TELEMETRIA.print(estSxOk ? "1" : "0");
        TELEMETRIA.print(estDxOk ? "1" : "0");   TELEMETRIA.print(","); // 9. Salute Servi

        // --- ASSETTO E QUOTA (IMU + Baro) ---
        TELEMETRIA.print(pitch_deg, 1); TELEMETRIA.print(","); // 10. Pitch reale, °
        TELEMETRIA.print(roll_deg, 1);  TELEMETRIA.print(","); // 11. Roll reale, °
        TELEMETRIA.print(yaw_deg, 1);   TELEMETRIA.print(","); // 12. Yaw reale (bussola), °
        TELEMETRIA.print(ALTITUDINE_m, 1); TELEMETRIA.print(","); // 13. Altitudine usata dal sistema, m

        // --- VELOCITÀ (km/h, già convertite dal chiamante) ---
        TELEMETRIA.print(velAria_kmh, 1);      TELEMETRIA.print(","); // 14. Velocità Aria (Pitot), km/h
        TELEMETRIA.print(velSuoloGps_kmh, 1);  TELEMETRIA.print(","); // 15. Velocità Suolo GPS (grezza), km/h
        TELEMETRIA.print(VELOCITA_ARIA_ms * 3.6f, 1);  TELEMETRIA.print(","); // 16. Velocità Aria usata dal PID, km/h
        TELEMETRIA.print(VELOCITA_SUOLO_ms * 3.6f, 1); TELEMETRIA.print(","); // 17. Velocità Suolo usata (ottico/GPS), km/h

        // --- NAVIGAZIONE ---
        TELEMETRIA.print(DISTANZA_TARGET_m, 0); TELEMETRIA.print(","); // 18. Distanza target, m
        TELEMETRIA.print(ROTTA_TARGET_deg, 1);  TELEMETRIA.print(","); // 19. Rotta target, °
        TELEMETRIA.print(ROLL_TARGET_deg, 1);   TELEMETRIA.print(","); // 20. Rollio comandato da L1, °

        // --- INPUT RADIOCOMANDO (valori grezzi SBUS, range tipico 172-1811) ---
        TELEMETRIA.print(canaliRC[1]); TELEMETRIA.print(","); // 21. RC Pitch
        TELEMETRIA.print(canaliRC[0]); TELEMETRIA.print(","); // 22. RC Roll
        TELEMETRIA.print(canaliRC[2]); TELEMETRIA.print(","); // 23. RC Gas

        // --- OUTPUT PID/MIXER ---
        TELEMETRIA.print(outPitch_deg); TELEMETRIA.print(","); // 24. PID Pitch Out, °
        TELEMETRIA.print(outRoll_deg);  TELEMETRIA.print(","); // 25. PID Roll Out, °
        TELEMETRIA.print(outGas_us);    TELEMETRIA.print(","); // 26. PID Gas Out (effettivo, post-limite termico), us

        // --- POSIZIONE FISICA ATTUALE SERVI (gradi, range 45-135) ---
        TELEMETRIA.print(servoInternoSX.read()); TELEMETRIA.print(","); // 27. Pos Servo Int SX, °
        TELEMETRIA.print(servoInternoDX.read()); TELEMETRIA.print(","); // 28. Pos Servo Int DX, °
        TELEMETRIA.print(servoEsternoSX.read()); TELEMETRIA.print(","); // 29. Pos Servo Est SX, °
        TELEMETRIA.print(servoEsternoDX.read()); TELEMETRIA.print(","); // 30. Pos Servo Est DX, °

        // --- TEMPERATURE (°C) ---
        TELEMETRIA.print(TEMPERATURA_MOTORE_C, 1); TELEMETRIA.print(","); // 31. Temp Motore, °C
        TELEMETRIA.print(TEMPERATURA_FUSOLIERA_C, 1);   TELEMETRIA.print(","); // 32. Temp Avionica/Aria, °C

        // --- SATELLITI E COORDINATE GPS ---
        if (gps.location.isValid()) {
            TELEMETRIA.print(gps.satellites.value()); TELEMETRIA.print(","); // 33. Numero satelliti
            TELEMETRIA.print(gps.location.lat(), 6);  TELEMETRIA.print(","); // 34. Latitudine, °
            TELEMETRIA.print(gps.location.lng(), 6);                        // 35. Longitudine, °
        } else {
            TELEMETRIA.print("0,0.000000,0.000000"); // Satelliti=0, Lat=0, Lon=0 (nessun fix)
        }
        TELEMETRIA.print(",");
        TELEMETRIA.print(releAttivato ? "1" : "0");   // 36. Relè attivato (0/1)
        TELEMETRIA.print(",");

        // --- FLUSSO OTTICO (velocità stimata, m/s) ---
        TELEMETRIA.print(VELOCITA_OTTICA_X_ms, 2); TELEMETRIA.print(","); // 37. Vel X flusso ottico, m/s
        TELEMETRIA.print(VELOCITA_OTTICA_Y_ms, 2); TELEMETRIA.print(","); // 38. Vel Y flusso ottico, m/s

        // --- STATO SENSORI (bitmask: b0=flusso ottico OK, b1=LIDAR OK, b2=pacchetto SBUS perso) ---
        int statoSensori = 0;
        if (flussoOtticoOk) statoSensori += 1;
        if (lidarOk)         statoSensori += 2;
        if (pacchettoPerso)  statoSensori += 4;
        TELEMETRIA.print(statoSensori); TELEMETRIA.print(","); // 39. Bitmask stato sensori

        // --- NAVIGAZIONE (errore rotta) ---
        TELEMETRIA.print(ERRORE_ROTTA_deg, 1); TELEMETRIA.print(","); // 40. Errore rotta, °

        // --- PROTEZIONE TERMICA MOTORE ---
        TELEMETRIA.print(GAS_LIMITE_TERMICO_us); TELEMETRIA.print(","); // 41. Limite gas termico, us

        // --- ALTITUDINI GREZZE (per diagnostica sensori, prima della selezione), m ---
        TELEMETRIA.print(ALTITUDINE_LIDAR_m, 2); TELEMETRIA.print(","); // 42. Altitudine LIDAR grezza, m
        TELEMETRIA.print(ALTITUDINE_BARO_m, 2);  TELEMETRIA.print(","); // 43. Altitudine Baro grezza, m

        // --- Timestamp e contatore pacchetto (per rilevare pacchetti persi lato GCS) ---
        TELEMETRIA.print(tempoAttuale_ms); TELEMETRIA.print(","); // 44. millis() al momento dell'invio
        TELEMETRIA.print(numeroPacchettoTEL1);                   // 45. Numero progressivo pacchetto TEL1

        TELEMETRIA.println();
        numeroPacchettoTEL1++;

        // Pacchetti diagnostici supplementari (round-robin ogni ~2s, o forzati da REQ_DIAG)
        if (forzaInvioDiagnostica || (tempoAttuale_ms - TIMER_TELEMETRIA_DIAG_ms > 2000)) {
            TIMER_TELEMETRIA_DIAG_ms = tempoAttuale_ms;
            forzaInvioDiagnostica = false;

            if (contatorePacchettoDiag == 0) {
                // TEL2: correnti/potenze batterie + flusso ottico grezzo + gas pre-limite
                float pMotore = sensoreMotore.getPower_mW();
                float pTeensy = sensoreTeensy.getPower_mW();
                TELEMETRIA.print("$2,");
                TELEMETRIA.print(sensoreMotore.getCurrent_mA(), 1); TELEMETRIA.print(","); // corrente motore, mA
                TELEMETRIA.print(sensoreTeensy.getCurrent_mA(), 1); TELEMETRIA.print(","); // corrente Teensy, mA
                TELEMETRIA.print(CORRENTE_SERVO_INT_SX_mA, 1);      TELEMETRIA.print(","); // corrente servo Int SX, mA
                TELEMETRIA.print(CORRENTE_SERVO_INT_DX_mA, 1);      TELEMETRIA.print(","); // corrente servo Int DX, mA
                TELEMETRIA.print(CORRENTE_SERVO_EST_SX_mA, 1);      TELEMETRIA.print(","); // corrente servo Est SX, mA
                TELEMETRIA.print(CORRENTE_SERVO_EST_DX_mA, 1);      TELEMETRIA.print(","); // corrente servo Est DX, mA
                TELEMETRIA.print(pMotore, 1);                       TELEMETRIA.print(","); // potenza motore, mW
                TELEMETRIA.print(pTeensy, 1);                       TELEMETRIA.print(","); // potenza Teensy, mW
                TELEMETRIA.print(batteriaBassaMotore ? "1" : "0");  TELEMETRIA.print(","); // batteria motore bassa
                TELEMETRIA.print(batteriaBassaTeensy ? "1" : "0");  TELEMETRIA.print(","); // batteria Teensy bassa
                TELEMETRIA.print(FLUSSO_OTTICO_DX_conteggi);        TELEMETRIA.print(","); // flusso ottico dx grezzo
                TELEMETRIA.print(FLUSSO_OTTICO_DY_conteggi);        TELEMETRIA.print(","); // flusso ottico dy grezzo
                TELEMETRIA.print(GAS_COMANDATO_PRE_LIMITE_us);      TELEMETRIA.print(","); // gas pre-limite termico, us
                TELEMETRIA.print(limitazioneTermicaAttiva ? "1" : "0"); // limitazione termica attiva
                TELEMETRIA.println();

            } else if (contatorePacchettoDiag == 1) {
                // TEL3: diagnostica PID completa
                TELEMETRIA.print("$3,");
                TELEMETRIA.print(PID_ALT_ERRORE_m, 2);       TELEMETRIA.print(","); // errore quota, m
                TELEMETRIA.print(PID_ALT_P, 2);              TELEMETRIA.print(","); // termine P quota (pitch target, °)
                TELEMETRIA.print(PID_ALT_I, 2);              TELEMETRIA.print(",");
                TELEMETRIA.print(PID_ALT_D, 2);              TELEMETRIA.print(",");
                TELEMETRIA.print(PID_PITCH_TARGET_AUTO_deg, 2); TELEMETRIA.print(","); // pitch target risultante, °
                TELEMETRIA.print(PID_PITCH_ERRORE_deg, 2);   TELEMETRIA.print(","); // errore pitch, °
                TELEMETRIA.print(PID_PITCH_P, 2);            TELEMETRIA.print(",");
                TELEMETRIA.print(PID_PITCH_I, 2);            TELEMETRIA.print(",");
                TELEMETRIA.print(PID_PITCH_D, 2);            TELEMETRIA.print(",");
                TELEMETRIA.print(PID_ROLL_ERRORE_deg, 2);    TELEMETRIA.print(","); // errore roll, °
                TELEMETRIA.print(PID_ROLL_P, 2);             TELEMETRIA.print(",");
                TELEMETRIA.print(PID_ROLL_I, 2);             TELEMETRIA.print(",");
                TELEMETRIA.print(PID_ROLL_D, 2);             TELEMETRIA.print(",");
                TELEMETRIA.print(PID_VEL_ERRORE_kmh, 2);     TELEMETRIA.print(","); // errore velocità, km/h
                TELEMETRIA.print(PID_VEL_P, 2);              TELEMETRIA.print(",");
                TELEMETRIA.print(PID_VEL_I, 2);              TELEMETRIA.print(",");
                TELEMETRIA.print(PID_VEL_D, 2);              TELEMETRIA.print(",");
                TELEMETRIA.print(ALTITUDINE_TARGET_m, 1);    TELEMETRIA.print(","); // quota target, m
                TELEMETRIA.print(VELOCITA_TARGET_ATTUALE_kmh, 1);                    // velocità target, km/h
                TELEMETRIA.println();

            } else {
                // TEL4: GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC estesi
                aggiornaDiagnosticaIMU();   // Bassa frequenza, ~ogni 6 s
                TELEMETRIA.print("$4,");
                // GPS esteso
                TELEMETRIA.print(gps.altitude.isValid() ? gps.altitude.meters() : -1.0, 1); TELEMETRIA.print(","); // Alt GPS, m
                TELEMETRIA.print(gps.course.isValid() ? gps.course.deg() : -1.0, 1);        TELEMETRIA.print(","); // Rotta GPS, °
                TELEMETRIA.print(gps.speed.isValid() ? "1" : "0");    TELEMETRIA.print(","); // Validità velocità GPS
                TELEMETRIA.print(gps.course.isValid() ? "1" : "0");   TELEMETRIA.print(","); // Validità rotta GPS
                TELEMETRIA.print(gps.location.isValid() ? "1" : "0"); TELEMETRIA.print(","); // Validità posizione GPS
                // Barometro esteso
                TELEMETRIA.print(PRESSIONE_BARO_Pa, 1);      TELEMETRIA.print(","); // Pressione, Pa
                TELEMETRIA.print(TARA_ALTITUDINE_BARO_m, 1); TELEMETRIA.print(","); // Tara ASL, m
                TELEMETRIA.print(baroPronto ? "1" : "0");    TELEMETRIA.print(",");
                // Pitot grezzo
                TELEMETRIA.print(PITOT_RAW_adc);             TELEMETRIA.print(","); // conteggi ADC (0-1023)
                TELEMETRIA.print(PITOT_ZERO_adc, 1);         TELEMETRIA.print(","); // conteggi ADC (zero calibrato)
                TELEMETRIA.print(PITOT_DIFFERENZA_adc, 1);   TELEMETRIA.print(","); // conteggi ADC (differenza)
                TELEMETRIA.print(PITOT_VALIDO ? "1" : "0");  TELEMETRIA.print(",");
                // IMU estesa
                TELEMETRIA.print(OFFSET_PITCH_deg, 2); TELEMETRIA.print(","); // °
                TELEMETRIA.print(OFFSET_ROLL_deg, 2);  TELEMETRIA.print(","); // °
                TELEMETRIA.print(OFFSET_YAW_deg, 2);   TELEMETRIA.print(","); // °
                TELEMETRIA.print(ACCEL_X_ms2, 2);      TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(ACCEL_Y_ms2, 2);      TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(ACCEL_Z_ms2, 2);      TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(ACCEL_TOTALE_ms2, 2); TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(GYRO_X_degs, 2);      TELEMETRIA.print(","); // °/s
                TELEMETRIA.print(GYRO_Y_degs, 2);      TELEMETRIA.print(","); // °/s
                TELEMETRIA.print(GYRO_Z_degs, 2);      TELEMETRIA.print(","); // °/s
                TELEMETRIA.print(IMU_CAL_SYS);   TELEMETRIA.print(","); // 0-3
                TELEMETRIA.print(IMU_CAL_GYRO);  TELEMETRIA.print(","); // 0-3
                TELEMETRIA.print(IMU_CAL_ACCEL); TELEMETRIA.print(","); // 0-3
                TELEMETRIA.print(IMU_CAL_MAG);   TELEMETRIA.print(","); // 0-3
                // RC estesi (canali 4-16, indici 3..15). Canali 1-3 già inviati in TEL1.
                for (int i = 3; i < 16; i++) {
                    TELEMETRIA.print(canaliRC[i]);
                    if (i < 15) TELEMETRIA.print(",");
                }
                TELEMETRIA.println();
            }
            contatorePacchettoDiag = (contatorePacchettoDiag + 1) % 3;   // TEL2 -> TEL3 -> TEL4 -> TEL2...
        }
    }
}

// Legge comandi testuali (terminati da '\n') sia da USB (Serial) che da LoRa (TELEMETRIA) e li passa al parser
void comandiDaTerra() {
    static String buffer = "";

    HardwareSerial* fonti[] = { &TELEMETRIA, &Serial };   // Due possibili sorgenti di comando: LoRa e USB
    for (auto* porta : fonti) {
        while (porta->available()) {
            char c = porta->read();
            if (c == '\n') {
                segnalaOK();               // Beep di conferma ricezione riga di comando
                elaboraComando(buffer);    // Interpreta il comando accumulato
                buffer = "";
            } else {
                buffer += c;
                if (buffer.length() > 64) buffer = "";   // Protezione overflow: scarta il buffer se supera 64 caratteri senza newline
            }
        }
    }
}

//  PARSER COMANDI --------------------------------------------------------------------
void elaboraComando(const String& cmd) {
    if (!cmd.startsWith("CMD:")) return;   // Tutti i comandi validi iniziano con "CMD:"

    int sep = cmd.indexOf(':', 4);   // Cerca il secondo ":" che separa il nome campo dal valore
    if (sep < 0) return;

    String campo     = cmd.substring(4, sep);
    String valoreStr = cmd.substring(sep + 1);
    int    val       = valoreStr.toInt();

    // Servi: posizione (gradi, vincolati 45-135)
    if (campo == "SERVO_ISX") {
        servoInternoSX.write(constrain(val, 45, 135));
        inviaAck(campo, valoreStr);

    } else if (campo == "SERVO_IDX") {
        servoInternoDX.write(constrain(val, 45, 135));
        inviaAck(campo, valoreStr);

    } else if (campo == "SERVO_ESX") {
        servoEsternoSX.write(constrain(val, 45, 135));
        inviaAck(campo, valoreStr);

    } else if (campo == "SERVO_EDX") {
        servoEsternoDX.write(constrain(val, 45, 135));
        inviaAck(campo, valoreStr);

    // Servi: attach/detach manuale da terra
    } else if (campo == "SERVO_ISX_ATTACH") {
        servoInternoSX.attach(PIN_INT_SX);
        inviaAck(campo, "");

    } else if (campo == "SERVO_IDX_ATTACH") {
        servoInternoDX.attach(PIN_INT_DX);
        inviaAck(campo, "");

    } else if (campo == "SERVO_ESX_ATTACH") {
        servoEsternoSX.attach(PIN_EST_SX);
        inviaAck(campo, "");

    } else if (campo == "SERVO_EDX_ATTACH") {
        servoEsternoDX.attach(PIN_EST_DX);
        inviaAck(campo, "");

    } else if (campo == "SERVO_ISX_DETACH") {
        servoInternoSX.detach();
        inviaAck(campo, "");

    } else if (campo == "SERVO_IDX_DETACH") {
        servoInternoDX.detach();
        inviaAck(campo, "");

    } else if (campo == "SERVO_ESX_DETACH") {
        servoEsternoSX.detach();
        inviaAck(campo, "");

    } else if (campo == "SERVO_EDX_DETACH") {
        servoEsternoDX.detach();
        inviaAck(campo, "");

    } else if (campo == "RELE_ON") {
        digitalWrite(PIN_RELE, HIGH);
        releAttivato = true;
        inviaAck(campo, "");

    } else if (campo == "RELE_OFF") {
        digitalWrite(PIN_RELE, LOW);
        releAttivato = false;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SCHIANTO_ON") {
        schiantoSicurezza = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SCHIANTO_OFF") {
        schiantoSicurezza = false;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_ALIMENTAZIONE_ON") {
        alimentazioneSicurezza = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_ALIMENTAZIONE_OFF") {
        alimentazioneSicurezza = false;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SERVI_ON") {
        Sicurezza_servo = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SERVI_OFF") {
        Sicurezza_servo = false;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_TEMP_ON") {
        limitazione_termica_gas = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_TEMP_OFF") {
        limitazione_termica_gas = false;
        inviaAck(campo, "");

    // Gas — solo comandabile manualmente in modalità 1, in microsecondi (us), vincolato tra GAS_NEUTRO_us e GAS_MASSIMO_us
    } else if (campo == "GAS") {
        if (global_modalitaVolo == 1) {
            motore.writeMicroseconds(constrain(val, GAS_NEUTRO_us, GAS_MASSIMO_us));
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "non_in_manuale");
        }

    // Modalità di volo (1=Manuale, 2=Auto), rifiutata se in failsafe o valore fuori range
    } else if (campo == "MODO") {
        if (!failsafe && val >= 1 && val <= 2) {
            global_modalitaVolo = val;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "valore_non_valido_o_failsafe");
        }

    } else if (campo == "SET_LATITUDE") {
        float lat_deg = valoreStr.toFloat();
        if (lat_deg >= -90.0 && lat_deg <= 90.0) {
            TARGET_LAT_deg = lat_deg;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    } else if (campo == "SET_LONGITUDE") {
        float lon_deg = valoreStr.toFloat();
        if (lon_deg >= -180.0 && lon_deg <= 180.0) {
            TARGET_LON_deg = lon_deg;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    } else if (campo == "SET_ALTITUDE") {
        float alt_m = valoreStr.toFloat();
        if (alt_m >= 0.0 && alt_m <= 500.0) {
            ALTITUDINE_TARGET_m = alt_m;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    // ─── COMANDI: PARAMETRI PID (validati, entro limiti di sicurezza) ───
    } else if (campo == "SET_KP_VEL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KP_MAX) { Kp_vel = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_VEL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KI_MAX) { Ki_vel = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_VEL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KD_MAX) { Kd_vel = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KP_ROLL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KP_MAX) { Kp_roll = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_ROLL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KI_MAX) { Ki_roll = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_ROLL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KD_MAX) { Kd_roll = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KP_PITCH") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KP_MAX) { Kp_pitch = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_PITCH") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KI_MAX) { Ki_pitch = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_PITCH") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KD_MAX) { Kd_pitch = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KP_ALT") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KP_MAX) { Kp_alt = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_ALT") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KI_MAX) { Ki_alt = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_ALT") {
        float v = valoreStr.toFloat();
        if (v >= 0.0 && v <= LIMITE_KD_MAX) { Kd_alt = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

    // ─── COMANDI: PARAMETRI OPERATIVI ───
    } else if (campo == "SET_VEL_CROCIERA") {
        float v = valoreStr.toFloat();   // km/h
        if (v >= 20.0 && v <= 150.0) { VELOCITA_CROCIERA_kmh = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_VEL_AVVICINAMENTO") {
        float v = valoreStr.toFloat();   // km/h
        if (v >= 15.0 && v <= 150.0) { VELOCITA_AVVICINAMENTO_kmh = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

    // ─── COMANDI: CONTROLLO/DIAGNOSTICA ───
    } else if (campo == "REQ_DIAG") {
        forzaInvioDiagnostica = true;
        inviaAck(campo, "");
    } else if (campo == "RESET_PID") {
        resettaPID();
        inviaAck(campo, "");
    } else {
        inviaNack(campo, "comando_sconosciuto");
    }
}


// ============================================================
//  SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);
    Wire1.begin();
    Wire1.setClock(400000);

    pinMode(PIN_LED_ROSSO_ALARM, OUTPUT);
    pinMode(PIN_LED_VERDE_GPS, OUTPUT);
    pinMode(PIN_LED_BLU_PID, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_RELE, OUTPUT);

    digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
    digitalWrite(PIN_LED_BLU_PID, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    digitalWrite(PIN_RELE, LOW);

    // Bip di accensione (sequenza di 3 toni crescenti)
    tone(PIN_BUZZER, 800,  100); delay(150);
    tone(PIN_BUZZER, 1200, 100); delay(150);
    tone(PIN_BUZZER, 1600, 150); delay(300);

    Serial.println("     SISTEMA DRONE — AVVIO IN CORSO     ");

    ricevente.begin();
    TELEMETRIA.begin(BAUD_RATE_LORA);
    GPS_SERIAL.begin(BAUD_RATE_GPS);
    Serial2.begin(BAUD_RATE_LIDAR);
    delay(100);

    while (Serial2.available()) Serial2.read();   // Svuota eventuali byte residui nel buffer seriale del LIDAR

    // INIZIALIZZAZIONE SENSORI — ciclo ripetuto fino a MAX_TENTATIVI_INIT se qualcosa non è pronto
    while ((!imuPronto || !baroPronto || !pitotCalibrato || !sensoriCorrenteOk) && tentativiInit < MAX_TENTATIVI_INIT) {
        tentativiInit++;
        Serial.println("\n-----------------------------------------");
        Serial.print  ("  Tentativo ");
        Serial.print  (tentativiInit);
        Serial.print  (" / ");
        Serial.println(MAX_TENTATIVI_INIT);
        Serial.println("-----------------------------------------");

        // --- Flusso ottico: tentativo di inizializzazione via SPI ---
        if (!flussoOtticoOk) {
            Serial.println("[ ] Flusso Ottico PMW3901 ........... ");
            if (flussoOttico.begin()) {
                Serial.println("[OK] Flusso Ottico PMW3901");
                flussoOtticoOk = true;
            } else {
                Serial.println("ERRORE sensore flusso ottico (cavi SPI?)");
                segnalaErrore();
            }
        } else {
            Serial.println("[OK] Flusso Ottico PMW3901");
        }

        // --- LIDAR TF-Luna: attende 3000 ms un pacchetto valido con header 0x59 0x59 ---
        if (!lidarOk) {
            Serial.println("[ ] TF-Luna LIDAR .............. ");
            unsigned long t0 = millis();
            while (millis() - t0 < 3000) {
                if (Serial2.available() >= 9) {   // Un pacchetto TF-Luna è lungo 9 byte
                    if (Serial2.read() == 0x59 && Serial2.peek() == 0x59) {
                        Serial2.read();
                        for (int i = 0; i < 7; i++) {
                            Serial2.read();   // Scarta il resto del pacchetto
                        }
                        lidarOk = true;
                        break;
                    }
                }
            }
            if (lidarOk) {
                for (int i = 0; i < 10; i++) {
                    aggiornaLidar();   // 10 letture per "riscaldare" il filtro EMA sull'altezza LIDAR
                    delay(20);
                }
                Serial.println("[OK] TF-Luna LIDAR");
                segnalaOK();
            } else {
                Serial.println("[WARN] TF-Luna LIDAR assente — continuo senza");
            }
        } else {
            Serial.println("[OK] TF-Luna LIDAR");
        }

        // 1. IMU — inizializzazione, calibrazione interna e tara offset
        if (!imuPronto) {
            Serial.print("[ ] IMU BNO055 ................. ");
            if (giroscopio.begin()) {
                giroscopio.setExtCrystalUse(true);   // Cristallo esterno per un clock più stabile
                Serial.println("OK");

                // Calibrazione interna: attende che il giroscopio raggiunga almeno 2 su 3
                Serial.print("   Calibrazione interna (non muovere)");
                uint8_t sys, gyro, accel, mag;
                unsigned long timeout_ms = millis();
                do {
                    giroscopio.getCalibration(&sys, &gyro, &accel, &mag);
                    Serial.print(".");
                    delay(100);
                    if (millis() - timeout_ms > 10000) {   // Timeout massimo: 10000 ms
                        Serial.println(" timeout, continuo");
                        break;
                    }
                } while (gyro < 2);

                // Tara: media di IMU_CAMPIONI_TARA campioni per l'offset statico di roll/pitch/yaw
                Serial.println("\n   Tara offset in corso...");
                double sommaRoll_deg  = 0.0;
                double sommaPitch_deg = 0.0;
                double sommaYawSin = 0.0;
                double sommaYawCos = 0.0;
                for (int i = 0; i < IMU_CAMPIONI_TARA; i++) {
                    sensors_event_t ev;
                    giroscopio.getEvent(&ev);
                    sommaRoll_deg  += ev.orientation.z;   // roll
                    sommaPitch_deg += ev.orientation.y;   // pitch
                    float yawRad = radians(ev.orientation.x);
                    sommaYawSin += sin(yawRad);
                    sommaYawCos += cos(yawRad);
                    delay(10);
                }
                OFFSET_ROLL_deg  = (float)(sommaRoll_deg  / IMU_CAMPIONI_TARA);
                OFFSET_PITCH_deg = (float)(sommaPitch_deg / IMU_CAMPIONI_TARA);
                OFFSET_YAW_deg   = degrees(atan2(sommaYawSin, sommaYawCos));
                if (OFFSET_YAW_deg < 0.0f) OFFSET_YAW_deg += 360.0f;

                imuPronto = true;
                segnalaOK();
                Serial.print(" imu offsets: roll= ");
                Serial.print(OFFSET_ROLL_deg, 2);
                Serial.print(" ; pitch= ");
                Serial.print(OFFSET_PITCH_deg, 2);
                Serial.print(" ; yaw= ");
                Serial.println(OFFSET_YAW_deg, 2);
            } else {
                Serial.println("\n ERRORE (cavi I2C?)");
                segnalaErrore();
            }
        } else {
            Serial.println(" \n [OK] IMU BNO055");
        }

        // 2. BAROMETRO — inizializzazione, oversampling, filtro, tara altitudine ASL
        if (!baroPronto) {
            Serial.print("[ ] Barometro BMP390 ....... ");
            if (barometro.begin_I2C()) {
                barometro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
                barometro.setPressureOversampling(BMP3_OVERSAMPLING_32X);
                barometro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
                barometro.setOutputDataRate(BMP3_ODR_50_HZ);   // 50 Hz
                delay(100);

                // Letture a vuoto per scartare (le prime letture dopo un cambio config possono essere instabili)
                for (int j = 0; j < 3; j++) {
                    barometro.readAltitude(1013.25);   // Pressione di riferimento SLP standard: 1013.25 hPa
                    delay(25);
                }

                float sommaAltitudine_m = 0.0;
                bool erroreCalibrazione = false;

                for (int i = 0; i < 20; i++) {   // 20 campioni per la media di tara
                    segnalaCalibrazione(PIN_LED_BLU_PID);

                    float altitudineIstantanea_m = barometro.readAltitude(1013.25);   // Rispetto a 1013.25 hPa

                    // Validazione hardware: scarta letture fisicamente impossibili (range plausibile: -500..8000 m ASL)
                    if (altitudineIstantanea_m < -500.0 || altitudineIstantanea_m > 8000.0) {
                        Serial.println("\n ERRORE: Lettura barometrica impossibile");
                        Serial.print(" Altitudine letta: ");
                        Serial.print(altitudineIstantanea_m);
                        Serial.println(" m");

                        digitalWrite(PIN_LED_BLU_PID, LOW);
                        erroreCalibrazione = true;
                        break;
                    }
                    sommaAltitudine_m += altitudineIstantanea_m;
                    delay(25);   // 20 x 25 ms = 500 ms totali di campionamento
                }

                if (erroreCalibrazione) {
                    segnalaErrore();
                    continue;
                }

                digitalWrite(PIN_LED_BLU_PID, LOW);
                digitalWrite(PIN_BUZZER, LOW);

                baroPronto = true;
                float mediaBaroCalibrazione_m = sommaAltitudine_m / 20.0;

                // Se il LIDAR è disponibile ed entrambe le altezze indicano "vicino a terra" (<5 m),
                // usa il LIDAR per tarare l'offset barometrico
                if (mediaBaroCalibrazione_m < 5.0 && ALTITUDINE_LIDAR_m < 5.0 && ALTITUDINE_LIDAR_m > 0.0 && lidarOk) {
                    TARA_ALTITUDINE_BARO_m = mediaBaroCalibrazione_m - ALTITUDINE_LIDAR_m;
                    Serial.print("OK (Tara ASL corretta da LIDAR: ");
                } else {
                    TARA_ALTITUDINE_BARO_m = mediaBaroCalibrazione_m;
                    Serial.print("OK (Tara ASL standard: ");
                }

                Serial.print(TARA_ALTITUDINE_BARO_m, 1);
                Serial.println(" m)");
                segnalaOK();

            } else {
                Serial.println("ERRORE (cavi I2C?)");
                segnalaErrore();
            }
        } else {
            Serial.println("[OK] Barometro BMP390");
        }

        // 3. PITOT — calibrazione dello zero
        if (!pitotCalibrato) {
            Serial.print("[ ] Pitot (velocita') ...... ");
            long sommaLetture_adc = 0;
            for (int i = 0; i < 100; i++) {   // 100 campioni
                if (i % 10 == 0) {
                    segnalaCalibrazione(PIN_LED_VERDE_GPS);
                }
                sommaLetture_adc += analogRead(PIN_ARIA);
                delay(10);
            }
            digitalWrite(PIN_LED_VERDE_GPS, LOW);
            digitalWrite(PIN_BUZZER, LOW);

            PITOT_ZERO_adc = sommaLetture_adc / 100.0;
            if (PITOT_ZERO_adc > 5 && PITOT_ZERO_adc < 1020) {   // Verifica plausibilità
                pitotCalibrato = true;
                Serial.print("OK (zero: ");
                Serial.print(PITOT_ZERO_adc, 1);
                Serial.println(")");
                segnalaOK();
            } else {
                Serial.print("ERRORE (valore anomalo: ");
                Serial.print(PITOT_ZERO_adc);
                Serial.println(")");
                segnalaErrore();
            }
        } else {
            Serial.println("[OK] Pitot");
        }

        // 4. INA219 — verifica che tutti i sensori di corrente/tensione rispondano sul bus I2C
        sensoriCorrenteOk = true;
        Serial.print("[ ] INA219 Batteria motore........ ");
        if (sensoreMotore.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); sensoriCorrenteOk = false; }

        Serial.print("[ ] INA219 Batteria Teensy........ ");
        if (sensoreTeensy.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); sensoriCorrenteOk = false; }

        Serial.print("[ ] INA219 Servo IntSX ..... ");
        if (sensoreIntSX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); sensoriCorrenteOk = false; }

        Serial.print("[ ] INA219 Servo IntDX ..... ");
        if (sensoreIntDX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); sensoriCorrenteOk = false; }

        Serial.print("[ ] INA219 Servo EstSX ..... ");
        if (sensoreEstSX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); sensoriCorrenteOk = false; }

        Serial.print("[ ] INA219 Servo EstDX ..... ");
        if (sensoreEstDX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); sensoriCorrenteOk = false; }

        if (sensoriCorrenteOk) { segnalaOK(); } else { segnalaErrore(); }

        // Riepilogo tentativo: se manca ancora qualcosa, aspetta 2s e riprova
        if (!imuPronto || !baroPronto || !pitotCalibrato || !sensoriCorrenteOk) {
            Serial.println("\n  >> Sensori mancanti. Nuovo tentativo tra 2s...");
            digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
            delay(2000);
            digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        }
    }

    // ERRORE CRITICO — se dopo MAX_TENTATIVI_INIT manca un sensore necessario, blocca l'avvio
    if (!imuPronto || !baroPronto || !pitotCalibrato || !sensoriCorrenteOk) {
        Serial.println("\n!!! ERRORE CRITICO — AVVIO BLOCCATO !!!");
        Serial.println("    Controlla l'hardware e riavvia.");
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        while (1) {   // Blocco infinito
            tone(PIN_BUZZER, 2000, 300);
            delay(400);
        }
    }

    // TUTTO OK — INIT SERVO
    Serial.println("     TUTTI I SENSORI OPERATIVI          ");
    Serial.println("Inizializzazione servomotori...");

    inizializzaServo();    // Attacca tutti i servi e li porta al centro (90°)
    inizializzaMotore();   // Attacca l'ESC e invia comando neutro (GAS_NEUTRO_us)
    Serial.println("Settati flap neutri e gas al minimo");

    // Jingle avvio riuscito
    tone(PIN_BUZZER, 800,  120); delay(170);
    tone(PIN_BUZZER, 1200, 120); delay(170);
    tone(PIN_BUZZER, 1800, 200); delay(350);

    Serial.println("Verifica oculare luci di stato...");
    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    digitalWrite(PIN_LED_BLU_PID, HIGH);
    delay(1000);   // Tiene tutti i LED accesi 1 s
    digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
    digitalWrite(PIN_LED_BLU_PID, LOW);

    TEMPO_PID_PRECEDENTE_ms = millis();   // Riferimento temporale per il primo calcolo PID
    Serial.println("\n  >> SISTEMA PRONTO AL VOLO\n");
    delay(500);
}

// Collega i 4 servocomandi ai rispettivi pin e li porta tutti alla posizione centrale (90°)
void inizializzaServo() {
    servoInternoSX.attach(PIN_INT_SX);
    servoInternoDX.attach(PIN_INT_DX);
    servoEsternoSX.attach(PIN_EST_SX);
    servoEsternoDX.attach(PIN_EST_DX);
    servoInternoSX.write(CENTRO_SERVO_deg);
    servoInternoDX.write(CENTRO_SERVO_deg);
    servoEsternoSX.write(CENTRO_SERVO_deg);
    servoEsternoDX.write(CENTRO_SERVO_deg);
}

// Collega l'ESC del motore e invia il comando neutro
void inizializzaMotore() {
    motore.attach(PIN_MOTORE);
    motore.writeMicroseconds(GAS_NEUTRO_us);
}


// ============================================================
//  LOOP
// ============================================================
void loop()
{
    comandiDaTerra();
    gestisciAlimentazione();
    gestisciSchianto();
    verificaDroneInVolo();

    // 1. Lettura GPS (accumulo caratteri seriali)
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    diagnosticaServi();

    // 2. IMU — lettura angoli reali (pitch/roll con offset di tara sottratto)
    sensors_event_t event;
    giroscopio.getEvent(&event);
    float pitch_deg = event.orientation.y - OFFSET_PITCH_deg;
    float roll_deg  = event.orientation.z - OFFSET_ROLL_deg;
    float yaw_deg   = event.orientation.x;   // Yaw reale SENZA sottrarre OFFSET_YAW_deg (calcolato ma non usato qui)
    // Normalizzazione angolo nel range [0, 360)
    if (yaw_deg < 0.0f) {
        yaw_deg += 360.0f;
    } else if (yaw_deg >= 360.0f) {
        yaw_deg -= 360.0f;
    }

    // 3. Temperatura motore
    leggiTemperaturaMotore();

    // 4. Pitot — velocità ARIA (nessun fallback su altre velocità)
    leggiPitot();

    // 5. GPS — velocità al SUOLO grezza (Ground Speed)
    float velocitaSuoloGps_ms = 0.0f;
    if (gps.speed.isValid()) {
        velocitaSuoloGps_ms = gps.speed.mps();
    } else {
        velocitaSuoloGps_ms = -1.0f;   // Non valida
    }

    // 6. Barometro — altitudine relativa, pressione, temperatura aria
    leggiBarometro();

    aggiornaDensitaAria(PRESSIONE_BARO_Pa, TEMPERATURA_FUSOLIERA_C);
    aggiornaLidar();
    selezionaAltitudine();                    // metri (m)
    leggiVelocitaOttica(yaw_deg);              // m/s
    aggiornaVelocitaSuolo(velocitaSuoloGps_ms); // m/s
    aggiornaNavigazione(yaw_deg);

    // 7. PREPARAZIONE DATI MOTORE — sceglie velocità target e gas di base in base alla distanza dal target
    float targetVelocita_kmh = 0.0f;
    int gasDiBase_us = 0;
    if (DISTANZA_TARGET_m > DISTANZA_FRENATA_m) {
        targetVelocita_kmh = VELOCITA_CROCIERA_kmh;
        gasDiBase_us = GAS_CROCIERA_us;
    } else {
        targetVelocita_kmh = VELOCITA_AVVICINAMENTO_kmh;
        gasDiBase_us = GAS_AVVICINAMENTO_us;
    }

    int correzionePitch_deg = 0;
    int correzioneRoll_deg  = 0;
    int comandoGasFinale_us = GAS_NEUTRO_us;

    // 1 = Manuale, 2 = Auto, 3 = Failsafe
    if (ricevente.read(&canaliRC[0], &failsafe, &pacchettoPerso)) {   // Se è arrivato un nuovo pacchetto SBUS valido
        // Sblocco emergenza: se in stato di schianto rilevato, il canale 5 (indice 4) sotto 992 lo resetta manualmente
        if (statoSchiantoRilevato && canaliRC[4] < 992) {
            statoSchiantoRilevato = false;
            schiantoBloccato = false;
            droneInVolo = false;
            inizializzaServo();   // Ricentra e riattacca i servi

            statoPrecedenteInterni = true;
            statoPrecedenteEsterni = true;

            noTone(PIN_BUZZER);
            tone(PIN_BUZZER, 1000, 100);
            delay(150);
            tone(PIN_BUZZER, 1500, 100);
            Serial.println("!!! SBLOCCO EMERGENZA ESEGUITO DA RADIO !!! Servi Riarmati e centrati.");
        }

        // Canale 5 (indice 4) sceglie la modalità di volo: <992 = Manuale, >=992 = Auto (solo se già in volo e non in schianto)
        if (canaliRC[4] < 992) {
            global_modalitaVolo = 1;
        } else {
            if (droneInVolo && statoSchiantoRilevato == false) {
                global_modalitaVolo = 2;
            }
        }
    }

    int statoAttuale;
    static int ultimoStatoStampato = 0;
    if (failsafe) {
        statoAttuale = 3;   // Stato "3" = failsafe, per la sola logica di stampa/reset PID qui sotto
    } else {
        statoAttuale = global_modalitaVolo;
    }

    // RESET PID AL CAMBIO DI MODALITÀ — azzera gli integrali/derivate quando si passa da uno stato all'altro
    static int modalitaPrecedente = 1;
    if (statoAttuale != modalitaPrecedente) {
        resettaPID();
        Serial.print(">> Reset PID: modalita' ");
        Serial.print(modalitaPrecedente);
        Serial.print(" -> ");
        Serial.println(statoAttuale);
        modalitaPrecedente = statoAttuale;
    }

    if (!schiantoBloccato && global_modalitaVolo == 1 && !failsafe) {
        // MODALITÀ MANUALE: mappa i canali RC grezzi direttamente sui comandi
        // Gas: da 172-1811 (range SBUS) a GAS_NEUTRO_us-GAS_MASSIMO_us, vincolato ai limiti meccanici motore
        comandoGasFinale_us = constrain(map(canaliRC[2], 172, 1811, GAS_NEUTRO_us, GAS_MASSIMO_us), GAS_NEUTRO_us, GAS_MASSIMO_us);
        correzioneRoll_deg  = constrain(map(canaliRC[0], 172, 1811, -MAX_ROLL_deg,   MAX_ROLL_deg),   -MAX_ROLL_deg,  MAX_ROLL_deg);
        correzionePitch_deg = constrain(map(canaliRC[1], 172, 1811,  MAX_PITCH_deg, -MAX_PITCH_deg),  -MAX_PITCH_deg, MAX_PITCH_deg);
        if (statoAttuale != ultimoStatoStampato) {
            Serial.println("Volo: MANUALE (Comandi diretti dal radiocomando)");
            ultimoStatoStampato = statoAttuale;
        }
    } else if (!schiantoBloccato && (global_modalitaVolo == 2 || failsafe) && (droneInVolo || failsafe)) {
        // MODALITÀ AUTO o FAILSAFE: il PID calcola i comandi automaticamente
        if (failsafe) {
            if (statoAttuale != ultimoStatoStampato) {
                Serial.println("FAILSAFE ATTIVO! direzione a target automatico!");
                ultimoStatoStampato = statoAttuale;
            }
        } else {
            if (statoAttuale != ultimoStatoStampato) {
                Serial.println("Volo: GPS AUTO (Il PID comanda)");
                ultimoStatoStampato = statoAttuale;
            }
        }
        // Chiamata al PID: quota target (m), roll target (°), pitch/roll reali (°), velocità aria attuale/target (km/h), gas di base (us)
        calcolaPID(ALTITUDINE_TARGET_m, ROLL_TARGET_deg, pitch_deg, roll_deg,
                   VELOCITA_ARIA_ms * 3.6f, targetVelocita_kmh, gasDiBase_us,
                   correzionePitch_deg, correzioneRoll_deg, comandoGasFinale_us);
    }

    int gasEffettivo_us = GAS_NEUTRO_us;   // Gas realmente inviato al motore in questo ciclo (per la telemetria)

    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO_us);   // Dopo uno schianto: motore forzato a comando neutro
        gasEffettivo_us = GAS_NEUTRO_us;
    } else {
        gestisciAllarmi();   // Aggiorna i LED di stato in base ad allarmi/GPS/modalità
        applicaMixer4Servi(correzionePitch_deg, correzioneRoll_deg);

        int limiteTermico_us = gasMaxTermico();
        GAS_LIMITE_TERMICO_us = limiteTermico_us;
        GAS_COMANDATO_PRE_LIMITE_us = comandoGasFinale_us;
        if (comandoGasFinale_us > limiteTermico_us && limitazione_termica_gas) {
            comandoGasFinale_us = limiteTermico_us;   // Applica il taglio termico se abilitato
            limitazioneTermicaAttiva = true;
        } else {
            limitazioneTermicaAttiva = false;
        }
        motore.writeMicroseconds(comandoGasFinale_us);
        gasEffettivo_us = comandoGasFinale_us;
    }

    inviaTelemetria(
        pitch_deg, roll_deg, yaw_deg,
        VELOCITA_ARIA_ms * 3.6f,
        velocitaSuoloGps_ms * 3.6f,
        correzionePitch_deg, correzioneRoll_deg, gasEffettivo_us);
}
