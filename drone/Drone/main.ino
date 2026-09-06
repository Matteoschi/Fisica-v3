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
const int PIN_TEMP_ESC = A6;
const int PIN_TEMP_EST = A8;
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
const int   CAMPIONI_CONFERMA_SCHIANTO_ms   = 3;         // Cicli di loop consecutivi per confermare uno schianto
const float VELOCITA_SUOLO_GPS_AFFIDABILE_ms = 3.0f;   // Velocità al suolo minima per considerare affidabile la rotta GPS (m/s)
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
bool gpsOk = false;
int  tentativiInit     = 0;

//  VARIABILI GLOBALI — ARIA / PITOT (VELOCITÀ ARIA)

float DENSITA_ARIA_kgm3     = 1.225f;  // kg/m^3, aggiornata da pressione+temperatura del barometro
float PITOT_ZERO_adc        = 0.0f;    // Valore di zero calibrato del pitot, in conteggi ADC
int   PITOT_RAW_adc         = 0;       // Ultima lettura grezza pitot, in conteggi ADC (0-1023)
float PITOT_DIFFERENZA_adc  = 0.0f;    // Differenza tra lettura e zero, in conteggi ADC
bool  PITOT_VALIDO          = false;   // true se la differenza è positiva (pressione dinamica misurabile)
float VELOCITA_ARIA_ms      = 0.0f;    // Velocità relativa all'aria, dal Pitot, in m/s. NON viene mai mescolata con GPS o ottico.

//  VARIABILI GLOBALI — SUOLO / FLUSSO OTTICO (VELOCITÀ AL SUOLO)
float VELOCITA_SUOLO_ms          = 0.0f;   // m/s — UNICA velocità al suolo realmente usata dal controllo 
float VELOCITA_OTTICA_X_ms       = 0.0f;   // Componente X stimata dal flusso ottico, in m/s (-1 = non valida)
float VELOCITA_OTTICA_Y_ms       = 0.0f;   // Componente Y stimata dal flusso ottico, in m/s (-1 = non valida)
int   FLUSSO_OTTICO_DX_conteggi  = 0;      // Conteggi grezzi di movimento ottico asse X (diagnostica)
int   FLUSSO_OTTICO_DY_conteggi  = 0;      // Conteggi grezzi di movimento ottico asse Y (diagnostica)
float velocitaSuoloGps_ms       = 0.0f;   // Velocità al suolo stimata dal GPS, in m/s (-1 = non valida)
int Errore_gps = -2;   // Stato del GPS: 0=nessun dato, -1=nessun fix, 3=ottimo, 2=buono, 1=scadente
//  VARIABILI GLOBALI — ALTITUDINE

float ALTITUDINE_LIDAR_m      = -1.0f;  // Altitudine dal LIDAR TF-Luna, in metri (-1 = non disponibile/fuori range)
float ALTITUDINE_BARO_m       = 0.0f;   // Altitudine dal barometro (relativa al punto di decollo), in metri
float ALTITUDINE_m            = 0.0f;   // Altitudine effettivamente usata dal sistema, in metri
float TARA_ALTITUDINE_BARO_m  = 0.0f;   // Offset sottratto al barometro per azzerare l'altitudine al decollo, in metri
float PRESSIONE_BARO_Pa       = 0.0f;

// VENTO STIMATO
float VENTO_VELOCITA_ms   = 0.0f;   // Modulo del vento stimato, in m/s
float VENTO_DIREZIONE_deg = 0.0f;   // Direzione DA CUI soffia il vento, in gradi (0=Nord)

//  VARIABILI GLOBALI — TEMPERATURE
float TEMPERATURA_MOTORE_C = 0.0f;
float TEMPERATURA_FUSOLIERA_C   = 0.0f;   
float TEMPERATURA_ESC_C=0.0f;
float TEMPERATURA_ESTERNA_C = 0.0f;

//  VARIABILI GLOBALI — NAVIGAZIONE
double TARGET_LAT_deg       = 41.902782;
double TARGET_LON_deg       = 12.496366;
double DRONE_LAT_deg       = 41.902782;
double DRONE_LON_deg       = 12.496366;
int numero_satelliti=   0;
float  ALTITUDINE_TARGET_m  = 40.0f;

float ROLL_TARGET_deg    = 0.0f;   // Target di rollio calcolato dalla guida L1
float rottaAttuale_deg = 0.0f;   // Rotta attuale stimata dal GPS, in gradi (0=Nord/90=Est/180=Sud/270=Ovest)
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

unsigned long TEMPO_BATTERIA_PRECEDENTE_ms = 0;   // Riferimento temporale per l'integrazione di carica/autonomia

float G_carica_consumata_teensy = 0.0;
float G_carica_consumata_motore = 0.0;
float G_carica_rimanente_teensy_percentuale=0.0;
float G_carica_rimanente_MOTORE_percentuale=0.0;
float G_autonomia_teensy_residua = 0.0;
float G_autonomia_motore_residua=0.0;

float iTeensy = 0.0;
float iMotore =0.0;
float vTeensy=0.0;
float vMotore = 0.0;

float CORRENTE_SERVO_INT_SX_mA = 0.0f;
float CORRENTE_SERVO_INT_DX_mA = 0.0f;
float CORRENTE_SERVO_EST_SX_mA = 0.0f;
float CORRENTE_SERVO_EST_DX_mA = 0.0f;

//  VARIABILI GLOBALI — MOTORE / GAS (microsecondi, us)

int  GAS_LIMITE_TERMICO_us       = GAS_MASSIMO_us;
bool limitazioneTermicaAttiva    = false;
bool limitazione_termica_gas        = true;   
bool motoreDisabilitatoDaTerra   = false;   // Kill switch software: se true il motore viene forzato al neutro ogni ciclo, indipendentemente dalla modalità

//  VARIABILI GLOBALI — SERVI E SICUREZZA
bool servoSicurezza          = true;   // Abilita/disabilita la diagnostica di sicurezza sui servi
bool statoPrecedenteInterni  = true;
bool statoPrecedenteEsterni  = true;
bool estSxOk = true, estDxOk = true;
bool intSxOk = true, intDxOk = true;
bool errore_critico_inizializzazione= false;

bool schiantoSicurezza      = true;  
bool statoSchiantoRilevato  = false;
bool schiantoBloccato       = false;
bool droneInVolo            = false;
int  contatoreImpatto       = 0;
unsigned long TIMESTAMP_DECOLLO_ms = 0;

int global_modalitaVolo = 1;   // Modalità di volo corrente: 1=Manuale, 2=Auto (3=Failsafe gestito a parte)

float VELOCITA_CROCIERA_kmh      = 60.0f;
float VELOCITA_AVVICINAMENTO_kmh = 45.0f;


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

bool forzaInvioDiagnostica             = false;


//  PROTOTIPI
void segnalaOK();
void segnalaErrore();
void segnalaCalibrazione(int pin_led);
void inizializzaServo();
void inizializzaMotore();

void leggiPitot();
void leggiBarometro();
void leggiTemperatura();
void aggiornaLidar();
void leggiVelocitaOttica(float yaw_deg);
void aggiornaGPS();
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
void stimaVento(float yaw_deg);
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
void inviaMessaggioAvionica(const char* messaggio) {
    Serial.print("[AVIONICA] da banco: ");
    Serial.println(messaggio);
    SERIALE_LORA.print("MSG,");
    SERIALE_LORA.println(messaggio);
}

void aggiornaGPS() {

    if (gps.charsProcessed() < 10) {
        Errore_gps = 0;   // Gravissimo: Hardware scollegato o Baudrate errato
    } 
    else if (!gps.location.isValid()) {
        Errore_gps = -1;  // Grave: Nessun Fix di posizione
    } 
    else if (gps.location.age() > 1500) {
        Errore_gps = -6;  // Grave: Dati in ritardo/congelati (Timeout > 1.5s)
    } 
    else if (!gps.satellites.isValid() || gps.satellites.value() == 0) {
        Errore_gps = -3;  // Anomalia: Zero satelliti validi (nonostante il fix)
    } 
    else if (!gps.course.isValid()) {
        Errore_gps = -2;  // Attesa: Fix presente, ma rotta non calcolabile 
    } 
    else if (!gps.speed.isValid()) {
        Errore_gps = -5;  // Attesa: Fix presente, ma velocità non calcolabile
    } 
    else {
        float hdop_attuale = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9f;

        if (hdop_attuale < 1.5f) {
            Errore_gps = 3; // Eccellente
        } 
        else if (hdop_attuale < 2.0f) {
            Errore_gps = 2; // Buono
        } 
        else {
            Errore_gps = 1; // Scarso
        }
    }

    static int erroreGpsPrecedente = -99;
    if (Errore_gps != erroreGpsPrecedente) {
        if (Errore_gps == 0)      inviaMessaggioAvionica("GPS: 0 -> scollegato");
        else if (Errore_gps == -1) inviaMessaggioAvionica("GPS: -1 -> nessun fix");
        else if (Errore_gps == -2) inviaMessaggioAvionica("GPS: -2 -> ROTTA NON CALCOLABILE");
        else if (Errore_gps == -3) inviaMessaggioAvionica("GPS: -3 -> zero satelliti validi");
        else if (Errore_gps == -5) inviaMessaggioAvionica("GPS: -5 -> velocita NON calcolabile");
        else if (Errore_gps == -6) inviaMessaggioAvionica("GPS: -6 -> dati congelati timeout > 1,5s");
        else if (Errore_gps == 1)  inviaMessaggioAvionica("GPS: 1 -> fix scarso");
        else if (Errore_gps == 2)  inviaMessaggioAvionica("GPS: 2 -> fix buono");
        else if (Errore_gps == 3)  inviaMessaggioAvionica("GPS: 3 -> fix eccellente");
        erroreGpsPrecedente = Errore_gps;
    }

    static bool fixValidoPrecedente = false;
    bool fixValidoAdesso = (Errore_gps > 0);
    if (fixValidoAdesso != fixValidoPrecedente) {
        if (fixValidoAdesso) {
            inviaMessaggioAvionica("GPS: fix riacquisito, navigazione automatica affidabile");
        } else if (droneInVolo) {
            inviaMessaggioAvionica("ATTENZIONE: GPS fix perso in volo!");
        }
        fixValidoPrecedente = fixValidoAdesso;
    }

    // 2. AGGIORNAMENTO DELLE VARIABILI DI VOLO
    if (Errore_gps > 0) {
        // VOLO AUTONOMO AUTORIZZATO: Tutti i dati sono perfetti
        DRONE_LAT_deg       = gps.location.lat();
        DRONE_LON_deg       = gps.location.lng();
        velocitaSuoloGps_ms = gps.speed.mps();
        rottaAttuale_deg    = gps.course.deg();
        numero_satelliti    = gps.satellites.value();

        DISTANZA_TARGET_m = TinyGPSPlus::distanceBetween(
            DRONE_LAT_deg, DRONE_LON_deg, TARGET_LAT_deg, TARGET_LON_deg
        );
        ROTTA_TARGET_deg = TinyGPSPlus::courseTo(
            DRONE_LAT_deg, DRONE_LON_deg, TARGET_LAT_deg, TARGET_LON_deg
        );
    } 
    else {
       
        DRONE_LAT_deg       = gps.location.isValid() ? gps.location.lat() : -1.0f;
        DRONE_LON_deg       = gps.location.isValid() ? gps.location.lng() : -1.0f;
        velocitaSuoloGps_ms = gps.speed.isValid()    ? gps.speed.mps()    : -1.0f;
        rottaAttuale_deg    = gps.course.isValid()   ? gps.course.deg()   : -1.0f;
        numero_satelliti    = gps.satellites.isValid() ? gps.satellites.value() : 0;
        
        // Disattiviamo la navigazione target
        DISTANZA_TARGET_m   = -1.0f;
        ROTTA_TARGET_deg    = -1.0f;
    }
}


void stimaVento(float yaw_deg){
    bool stima_vento_disponibile= false;
    if ( PITOT_VALIDO && Errore_gps > 2) {
        float velocitaAriaX_ms = VELOCITA_ARIA_ms * cos(radians(yaw_deg));
        float velocitaAriaY_ms = VELOCITA_ARIA_ms * sin(radians(yaw_deg));

        float velocitaVentoX_ms = VELOCITA_SUOLO_ms * cos(radians(gps.course.deg())) - velocitaAriaX_ms;
        float velocitaVentoY_ms = VELOCITA_SUOLO_ms * sin(radians(gps.course.deg())) - velocitaAriaY_ms;

        VENTO_VELOCITA_ms = sqrt(velocitaVentoX_ms * velocitaVentoX_ms + velocitaVentoY_ms * velocitaVentoY_ms);
        VENTO_DIREZIONE_deg = atan2(velocitaVentoY_ms, velocitaVentoX_ms) * 180.0f / PI;
        if (VENTO_DIREZIONE_deg < 0.0f) {
            VENTO_DIREZIONE_deg += 360.0f; // Normalizza a 0-360°
        }
        stima_vento_disponibile = true;
    } else {
        VENTO_VELOCITA_ms = -1.0f;
        VENTO_DIREZIONE_deg = -1.0f;
        stima_vento_disponibile = false;
    }
    static bool stimaVentoPrecedente = false;
    static bool statoStimaInizializzato = false;
    if (!statoStimaInizializzato || stima_vento_disponibile != stimaVentoPrecedente) {
        if (!stima_vento_disponibile) {
            inviaMessaggioAvionica("Stima vento non disponibile: pitot non valido o GPS non affidabile (Errore_gps=" + String(Errore_gps) + ") deve essere > 2");
        } else {
            inviaMessaggioAvionica("Stima vento aggiornata: Velocita=" + String(VENTO_VELOCITA_ms, 2) + " m/s, Direzione=" + String(VENTO_DIREZIONE_deg, 1) + " deg");
        }
        stimaVentoPrecedente = stima_vento_disponibile;
        statoStimaInizializzato = true;
    }
}

void leggiPitot() {
    int lettura_adc = constrain(analogRead(PIN_ARIA), 0, 1023);
    PITOT_RAW_adc = lettura_adc;
    float differenza_adc = (float)lettura_adc - PITOT_ZERO_adc; // sottrae tara
    PITOT_DIFFERENZA_adc = differenza_adc;
    PITOT_VALIDO         = (differenza_adc > 0.0f);
    if (PITOT_VALIDO) {
        VELOCITA_ARIA_ms = sqrtf((2.0f * differenza_adc * FATTORE_CONVERSIONE_PITOT_Pa) / DENSITA_ARIA_kgm3);   // v = sqrt(2*p/rho)
    } else {
        VELOCITA_ARIA_ms = -1.0f;  
    }

    static bool pitotValidoPrecedente = false;
    static bool statoPitotInizializzato = false;
    if (!statoPitotInizializzato || PITOT_VALIDO != pitotValidoPrecedente) {
        if (PITOT_VALIDO) {
            inviaMessaggioAvionica("Pitot valido: velocita aria aggiornata (" + String(VELOCITA_ARIA_ms, 2) + " m/s)");
        } else {
            inviaMessaggioAvionica("Pitot non valido: velocita aria non disponibile");
        }
        pitotValidoPrecedente = PITOT_VALIDO;
        statoPitotInizializzato = true;
    }
}

void leggiBarometro() {
    ALTITUDINE_BARO_m  = barometro.readAltitude(1013.25f) - TARA_ALTITUDINE_BARO_m;
    PRESSIONE_BARO_Pa  = barometro.pressure;
    TEMPERATURA_FUSOLIERA_C = barometro.temperature;
}

void leggiTemperatura() {
    float voltaggioSensore_motore_V = analogRead(PIN_TEMP_MOTORE) * (3.3f / 1023.0f);   // assumendo Vref 3.3V
    TEMPERATURA_MOTORE_C = (voltaggioSensore_motore_V - 0.5f) * 100.0f;

    float voltaggioSensore_esc_V = analogRead(PIN_TEMP_ESC) * (3.3f / 1023.0f);
    TEMPERATURA_ESC_C = (voltaggioSensore_esc_V - 0.5f) * 100.0f;

    float voltaggioSensore_esterno_V = analogRead(PIN_TEMP_EST) * (3.3f / 1023.0f);
    TEMPERATURA_ESTERNA_C = (voltaggioSensore_esterno_V - 0.5f) * 100.0f;
}

void aggiornaLidar() {
    static bool lidarDisponibilePrecedente = false;
    static bool statoLidarInizializzato = false;

    if (ALTITUDINE_BARO_m > ALTITUDINE_MAX_LIDAR_m) {  // Se il barometro è sopra la quota massima utile del LIDAR, non leggere più il LIDAR
        while (Serial2.available()) {
            Serial2.read();   // Svuota il buffer, scarta i dati
        }
        ALTITUDINE_LIDAR_m = -1.0f;
        if (!statoLidarInizializzato || lidarDisponibilePrecedente) {
            inviaMessaggioAvionica("LIDAR non disponibile: quota sopra il limite operativo");
            lidarDisponibilePrecedente = false;
            statoLidarInizializzato = true;
        }
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
            if (!statoLidarInizializzato || !lidarDisponibilePrecedente) {
                inviaMessaggioAvionica("LIDAR disponibile: altitudine aggiornata");
                lidarDisponibilePrecedente = true;
                statoLidarInizializzato = true;
            }
            return;
        }
    }
}

void leggiVelocitaOttica(float yaw_deg) {
    static unsigned long tempoPrecedente_ms = 0;
    static bool velocitaOtticaDisponibilePrecedente = false;
    static bool statoVelocitaOtticaInizializzato = false;
    int16_t dx = 0;
    int16_t dy = 0;
    flussoOttico.readMotionCount(&dx, &dy);
    FLUSSO_OTTICO_DX_conteggi = dx;
    FLUSSO_OTTICO_DY_conteggi = dy;

    unsigned long adesso_ms = millis();
    float dt_s = (adesso_ms - tempoPrecedente_ms) / 1000.0f;
    tempoPrecedente_ms = adesso_ms;

    if (dt_s <= 0.0f || ALTITUDINE_m > ALTITUDINE_MAX_OTTICO_m) {
        VELOCITA_OTTICA_X_ms = -1.0f;   // -1 = valore non valido
        VELOCITA_OTTICA_Y_ms = -1.0f;
        if (!statoVelocitaOtticaInizializzato || velocitaOtticaDisponibilePrecedente) {
            inviaMessaggioAvionica("Flusso ottico non disponibile: dt non valido o quota sopra il limite operativo");
            velocitaOtticaDisponibilePrecedente = false;
            statoVelocitaOtticaInizializzato = true;
        }
        return;
    }

    float vX_ms = (dx * COSTANTE_CALIBRAZIONE_OTTICA * ALTITUDINE_m) / dt_s;
    float vY_ms = (dy * COSTANTE_CALIBRAZIONE_OTTICA * ALTITUDINE_m) / dt_s;

    float yaw_rad = radians(yaw_deg);
    VELOCITA_OTTICA_X_ms = vX_ms * cos(yaw_rad) - vY_ms * sin(yaw_rad);
    VELOCITA_OTTICA_Y_ms = vX_ms * sin(yaw_rad) + vY_ms * cos(yaw_rad);

    if (!statoVelocitaOtticaInizializzato || !velocitaOtticaDisponibilePrecedente) {
        inviaMessaggioAvionica("Flusso ottico disponibile: velocita al suolo aggiornata");
        velocitaOtticaDisponibilePrecedente = true;
        statoVelocitaOtticaInizializzato = true;
    }
}


void aggiornaDensitaAria(float pressione_pa, float temperatura_c) {
    float temperatura_K = temperatura_c + 273.15f;
    if (temperatura_K > 0.0f && pressione_pa > 0.0f) {
        DENSITA_ARIA_kgm3 = pressione_pa / (R_SPECIFIC_ARIA * temperatura_K);
    } else {
        DENSITA_ARIA_kgm3 = 1.225f;   // Valore di default al livello del mare
    }
}

void selezionaAltitudine() {

    const float ZONA_BLEND_START_m = ALTITUDINE_MAX_LIDAR_m - 2.0f; 
    const float ZONA_BLEND_END_m   = ALTITUDINE_MAX_LIDAR_m;

    static float offsetBaro_m = 0.0f;
    
    bool lidarValido = (lidarOk && ALTITUDINE_LIDAR_m > 0.0f);

    if (lidarValido && ALTITUDINE_LIDAR_m < ZONA_BLEND_START_m) {
        offsetBaro_m = ALTITUDINE_BARO_m - ALTITUDINE_LIDAR_m;
    }

    float baroCorretto_m = ALTITUDINE_BARO_m - offsetBaro_m;

    // 3. Macchina a stati per l'assegnazione basata sulla quota normalizzata
    if (baroCorretto_m < ZONA_BLEND_START_m) {
        // ZONA BASSA: 100% LIDAR
        if (lidarValido) {
            ALTITUDINE_m = ALTITUDINE_LIDAR_m;
        } else {
            // Se il LIDAR si acceca a bassa quota (es. volo su acqua o erba altissima), 
            // usiamo il barometro allineato all'ultimo dato utile.
            ALTITUDINE_m = baroCorretto_m;
        }
    } 
    else if (baroCorretto_m >= ZONA_BLEND_END_m) {
        // ZONA ALTA: 100% Barometro (corretto con l'offset congelato alla salita)
        ALTITUDINE_m = baroCorretto_m;
    } 
    else {
        // ZONA DI TRANSIZIONE (CROSS-FADE)
        if (lidarValido) {
            // Calcolo del peso (W) del Barometro da 0.0 a 1.0
            float pesoBaro = (baroCorretto_m - ZONA_BLEND_START_m) / (ZONA_BLEND_END_m - ZONA_BLEND_START_m);
            pesoBaro = constrain(pesoBaro, 0.0f, 1.0f);

            // Interpolazione lineare (LERP): (1 - W) * Lidar + W * Barometro
            ALTITUDINE_m = (ALTITUDINE_LIDAR_m * (1.0f - pesoBaro)) + (baroCorretto_m * pesoBaro);
        } else {
            // Fallback immediato se il LIDAR cede durante la transizione
            ALTITUDINE_m = baroCorretto_m;
        }
    }
}

void aggiornaVelocitaSuolo(float velocitaSuoloGps_ms) {

    const float ZONA_BLEND_OTTICA_START_m = ALTITUDINE_MAX_OTTICO_m - 2.0f;

    // 1. Calcolo preventivo dello stato dei sensori
    float vel_ottica = -1.0f;
    bool otticaValida = (VELOCITA_OTTICA_X_ms != -1.0f && VELOCITA_OTTICA_Y_ms != -1.0f);
    
    if (otticaValida) {
        vel_ottica = sqrtf((VELOCITA_OTTICA_X_ms * VELOCITA_OTTICA_X_ms) + 
                           (VELOCITA_OTTICA_Y_ms * VELOCITA_OTTICA_Y_ms));
    }

    bool gpsValido = (Errore_gps > 0);

    // 3. Macchina a stati per l'assegnazione
    if (ALTITUDINE_m < ZONA_BLEND_OTTICA_START_m) {
        // ZONA BASSA: 100% Ottico
        if (otticaValida) {
            VELOCITA_SUOLO_ms = vel_ottica;
        } else if (gpsValido) {
            VELOCITA_SUOLO_ms = velocitaSuoloGps_ms; // Fallback d'emergenza
        } else {
            VELOCITA_SUOLO_ms = -1.0f;
        }
    } 
    else if (ALTITUDINE_m >= ALTITUDINE_MAX_OTTICO_m) {
        // ZONA ALTA: 100% GPS
        if (gpsValido) {
            VELOCITA_SUOLO_ms = velocitaSuoloGps_ms;
        } else {
            VELOCITA_SUOLO_ms = -1.0f;
        }
    } 
    else {
        // ZONA DI TRANSIZIONE (CROSS-FADE)
        if (otticaValida && gpsValido) {
            // Calcolo del peso (W) del GPS da 0.0 (inizio blend) a 1.0 (fine blend)
            float pesoGPS = (ALTITUDINE_m - ZONA_BLEND_OTTICA_START_m) / (ALTITUDINE_MAX_OTTICO_m - ZONA_BLEND_OTTICA_START_m);
            pesoGPS = constrain(pesoGPS, 0.0f, 1.0f); 

            // Interpolazione lineare (LERP): (1 - W) * Sensore1 + W * Sensore2
            VELOCITA_SUOLO_ms = (vel_ottica * (1.0f - pesoGPS)) + (velocitaSuoloGps_ms * pesoGPS);
            
        } else if (otticaValida) {
            // Se in transizione perdiamo il GPS, usiamo solo l'ottico
            VELOCITA_SUOLO_ms = vel_ottica;
        } else if (gpsValido) {
            // Se in transizione il flusso ottico diventa cieco (es. terreno senza texture), usiamo il GPS
            VELOCITA_SUOLO_ms = velocitaSuoloGps_ms;
        } else {
            VELOCITA_SUOLO_ms = -1.0f;
        }
    }
}
//  NAVIGAZIONE GPS (guida L1) ----------------------------------------------------
void aggiornaNavigazione(float yaw_deg) {
    static unsigned long ultimoGpsValido_ms = 0;
    static bool waypointRaggiunto = false;
    static bool rottaConYawPrecedente = false;
    static bool correzioneVentoPrecedente = false;
    static bool gpsTimeoutPrecedente = false;
    unsigned long tempoAttuale_ms = millis();
    

    if (Errore_gps > 0) {
        ultimoGpsValido_ms = tempoAttuale_ms;
        gpsTimeoutPrecedente = false;

        // Geometria verso il target: distanza (m) e rotta (°)
        float velocitaPerCalcolo_ms = max(VELOCITA_SUOLO_ms, 1.0f);
        float L1_m = max(velocitaPerCalcolo_ms * 4.0f, 1.0f);
        float raggioAccettazioneDinamico_m = max(RAGGIO_ACCETTAZIONE_MINIMO_m, L1_m * 0.75f);

        if (DISTANZA_TARGET_m <= raggioAccettazioneDinamico_m) {
            if (!waypointRaggiunto) {
                inviaMessaggioAvionica("WAYPOINT RAGGIUNTO");
                waypointRaggiunto = true;
            }
            DISTANZA_TARGET_m = 0.0f;
            return;
        } else {
            waypointRaggiunto = false;
        }

        bool rottaConYaw = (rottaAttuale_deg < 0.0f || VELOCITA_SUOLO_ms < VELOCITA_SUOLO_GPS_AFFIDABILE_ms);
        if (rottaConYaw) { // fare un booleano: rotta_attuale_con_yaw
            rottaAttuale_deg = yaw_deg;
            if (!rottaConYawPrecedente) {
                inviaMessaggioAvionica("rotta attuale aggiornata con il yaw");
                rottaConYawPrecedente = true;
            }
        } else {
            rottaConYawPrecedente = false;
        }

        float rottaCorretta_deg = ROTTA_TARGET_deg;   

        bool correzioneVento = (VENTO_VELOCITA_ms > 3.0f && VELOCITA_ARIA_ms > 3.0f);
        if (correzioneVento) {
            if (!correzioneVentoPrecedente) {
                inviaMessaggioAvionica("navigazione corretta anche con il vento)");
                correzioneVentoPrecedente = true;
            }
            float deltaVento_deg = VENTO_DIREZIONE_deg - ROTTA_TARGET_deg;
            if (deltaVento_deg > 180.0f) deltaVento_deg -= 360.0f;
            if (deltaVento_deg < -180.0f) deltaVento_deg += 360.0f;

            float argomentoAsin = (VENTO_VELOCITA_ms / VELOCITA_ARIA_ms) * sin(radians(deltaVento_deg));
            argomentoAsin = constrain(argomentoAsin, -1.0f, 1.0f);   // Clamp: evita NaN se vento >= TAS

            float wca_deg = degrees(asin(argomentoAsin));
            rottaCorretta_deg = ROTTA_TARGET_deg + wca_deg;

            if (rottaCorretta_deg >= 360.0f) rottaCorretta_deg -= 360.0f;
            if (rottaCorretta_deg < 0.0f)    rottaCorretta_deg += 360.0f;
        } else {
            correzioneVentoPrecedente = false;
        }

        ERRORE_ROTTA_deg = rottaCorretta_deg - rottaAttuale_deg;

        // Normalizzazione a ±180° (via più breve per girare)
        if (ERRORE_ROTTA_deg > 180.0f) {
            ERRORE_ROTTA_deg -= 360.0f;
        } else if (ERRORE_ROTTA_deg < -180.0f) {
            ERRORE_ROTTA_deg += 360.0f;
        }

        // Guida L1: accelerazione laterale necessaria per curvare verso la rotta target
        float eta_rad = radians(ERRORE_ROTTA_deg);
        float aLaterale_ms2 = (2.0f * velocitaPerCalcolo_ms * velocitaPerCalcolo_ms / L1_m) * sin(eta_rad);
        float rollNecessario_rad = atan(aLaterale_ms2 / 9.81f);
        
        ROLL_TARGET_deg = constrain(degrees(rollNecessario_rad), -MAX_ROLL_deg, MAX_ROLL_deg);
        
    } else {
        if ((tempoAttuale_ms - ultimoGpsValido_ms) > TIMEOUT_GPS_ms) {
            if (!gpsTimeoutPrecedente) {
                inviaMessaggioAvionica("ATTENZIONE: GPS non valido da troppo tempo, navigazione disabilitata");
                gpsTimeoutPrecedente = true;
            }
            ROLL_TARGET_deg = 0.0f;
        } else {
            gpsTimeoutPrecedente = false;
        }
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

    bool inStalloPrecedente = inStallo;
    bool inOverspeedPrecedente = inOverspeed;

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


    if (inStallo != inStalloPrecedente) {
        inviaMessaggioAvionica(inStallo ? "ATTENZIONE: STALLO rilevato, pitch forzato a picchiare"
                                         : "Stallo rientrato, PID pitch ripristinato");
    }
    if (inOverspeed != inOverspeedPrecedente) {
        inviaMessaggioAvionica(inOverspeed ? "ATTENZIONE: OVERSPEED rilevato, gas ridotto al minimo"
                                            : "Overspeed rientrato, PID velocita' ripristinato");
    }

    // 3. PID ALTITUDINE (bypassato se in stallo/overspeed/fuori range: il pitch è dettato dal recupero)
    float targetPitchAuto_deg = 0.0f;
    int gasCorrente_us = gasDiBase_us;
    static bool quotaMassimaPrecedente = false;
    static bool quotaMinimaPrecedente = false;

    if (inStallo) {
        if (!inStalloPrecedente) {
            inviaMessaggioAvionica("ATTENZIONE: STALLO rilevato, pitch forzato a salire e gas al massimo PITCH_DOWN_FORZATO_deg=" + String(PITCH_DOWN_FORZATO_deg) + " GAS_MASSIMO_us=" + String(GAS_MASSIMO_us));
        }
        quotaMassimaPrecedente = false;
        quotaMinimaPrecedente = false;
        targetPitchAuto_deg = PITCH_DOWN_FORZATO_deg;
        gasCorrente_us = GAS_MASSIMO_us;
        resettaPID();

    } else if (inOverspeed) {
        if (!inOverspeedPrecedente) {
            inviaMessaggioAvionica("ATTENZIONE: OVERSPEED rilevato, pitch forzato a scendere e gas al minimo PITCH_DOWN_FORZATO_deg=" + String(PITCH_DOWN_FORZATO_deg) + " GAS_MINIMO_us=" + String(GAS_MINIMO_us));
        }
        quotaMassimaPrecedente = false;
        quotaMinimaPrecedente = false;
        targetPitchAuto_deg = -PITCH_DOWN_FORZATO_deg;
        gasCorrente_us = GAS_MINIMO_us;
        resettaPID();

    } else if (ALTITUDINE_m > ALTITUDINE_MAX_m) {
        if (!quotaMassimaPrecedente) {
            inviaMessaggioAvionica("ATTENZIONE: quota massima superata, pitch forzato a scendere e gas al minimo PITCH_DOWN_FORZATO_deg=" + String(PITCH_DOWN_FORZATO_deg) + " GAS_MINIMO_us=" + String(GAS_MINIMO_us));
            quotaMassimaPrecedente = true;
        }
        quotaMinimaPrecedente = false;
        // Sopra la quota massima: forza un pitch negativo (scendi) e riduce il gas al minimo
        targetPitchAuto_deg = PITCH_DOWN_FORZATO_deg;
        gasCorrente_us = GAS_MINIMO_us;
        resettaPID();

    } else if (ALTITUDINE_m && !inStallo < ALTITUDINE_MIN_m) {
        if (!quotaMinimaPrecedente) {
            inviaMessaggioAvionica("ATTENZIONE: quota minima superata, pitch forzato a salire e gas al massimo PITCH_UP_FORZATO_deg=" + String(PITCH_UP_FORZATO_deg) + " GAS_MASSIMO_us=" + String(GAS_MASSIMO_us));
            quotaMinimaPrecedente = true;
        }
        quotaMassimaPrecedente = false;
        // Sotto la quota minima: forza un pitch positivo (sali) e aumenta il gas quasi al massimo
        targetPitchAuto_deg = PITCH_UP_FORZATO_deg;
        gasCorrente_us = GAS_MASSIMO_us;
        resettaPID();

    } else {
        quotaMassimaPrecedente = false;
        quotaMinimaPrecedente = false;
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


int gasMaxTermico() {
    static int statoTermicoPrecedente = -1;

    // 1. Taglio immediato: se la temperatura supera il limite massimo consentito
    if (TEMPERATURA_MOTORE_C >= MAX_THROTTLE_END_TEMP_C || 
        TEMPERATURA_ESC_C >= MIN_ESC_END_TEMP_C) {
        if (statoTermicoPrecedente != 0) {
            inviaMessaggioAvionica("ATTENZIONE: temperatura motore o ESC troppo alta, gas ridotto al minimo");
            statoTermicoPrecedente = 0;
        }
        return GAS_MINIMO_us;
    }

    // 2. Zona sicura: se entrambe le temperature sono sotto la soglia di intervento
    if (TEMPERATURA_MOTORE_C <= MIN_THROTTLE_START_TEMP_C && 
        TEMPERATURA_ESC_C <= MIN_ESC_START_TEMP_C) {
        if (statoTermicoPrecedente != 1) {
            inviaMessaggioAvionica("Temperatura motore e ESC nella zona sicura, gas massimo consentito");
            statoTermicoPrecedente = 1;
        }
        return GAS_MASSIMO_us;
    }

    float fattoreMotore = 0.0f;
    float fattoreESC = 0.0f;

    // 3. Calcolo fattore Motore (solo se in zona di derating)
    if (TEMPERATURA_MOTORE_C > MIN_THROTTLE_START_TEMP_C) {
        // Protezione contro la divisione per zero
        if (MAX_THROTTLE_END_TEMP_C > MIN_THROTTLE_START_TEMP_C) {
            fattoreMotore = (TEMPERATURA_MOTORE_C - MIN_THROTTLE_START_TEMP_C) / 
                            (MAX_THROTTLE_END_TEMP_C - MIN_THROTTLE_START_TEMP_C);
        } else {
            fattoreMotore = 1.0f; // Massima limitazione di sicurezza
        }
    }

    // 4. Calcolo fattore ESC (usando la tua variabile MIN_ESC_END_TEMP_C)
    if (TEMPERATURA_ESC_C > MIN_ESC_START_TEMP_C) {
        // Protezione contro la divisione per zero
        if (MIN_ESC_END_TEMP_C > MIN_ESC_START_TEMP_C) {
            fattoreESC = (TEMPERATURA_ESC_C - MIN_ESC_START_TEMP_C) / 
                         (MIN_ESC_END_TEMP_C - MIN_ESC_START_TEMP_C);
        } else {
            fattoreESC = 1.0f; // Massima limitazione di sicurezza
        }
    }
    float fattoreInterpolazione = 0.0f;
    if (fattoreMotore > fattoreESC) {
        if (statoTermicoPrecedente != 2) {
            inviaMessaggioAvionica("ATTENZIONE: temperatura motore troppo alta, gas ridotto");
            statoTermicoPrecedente = 2;
        }
        fattoreInterpolazione = fattoreMotore;
    } else {
        if (statoTermicoPrecedente != 3) {
            inviaMessaggioAvionica("ATTENZIONE: temperatura ESC troppo alta, gas ridotto");
            statoTermicoPrecedente = 3;
        }
        fattoreInterpolazione = fattoreESC;
    }

    // Clamping: assicura che il fattore non superi mai 1.0 a causa di letture anomale dei sensori
    if (fattoreInterpolazione > 1.0f) {
        fattoreInterpolazione = 1.0f;
    }

    // 6. Calcolo del segnale PWM finale
    float limite_us = GAS_MASSIMO_us - (fattoreInterpolazione * (GAS_MASSIMO_us - GAS_MINIMO_us));
    
    // Arrotondamento professionale all'intero più vicino per sistemi embedded
    return (int)(limite_us + 0.5f);
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
    static int casoMixerPrecedente = -1;
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
        if (casoMixerPrecedente != 0) {
            inviaMessaggioAvionica("Caso A: tutto OK — interni = SOLO PITCH, esterni = SOLO ROLL");
            casoMixerPrecedente = 0;
        }
        posIntSX_deg = CENTRO_SERVO_deg + pitch_deg;
        posIntDX_deg = CENTRO_SERVO_deg + pitch_deg;
        posEstSX_deg = CENTRO_SERVO_deg + roll_deg;
        posEstDX_deg = CENTRO_SERVO_deg - roll_deg;
    } else if (esterniAttivi && !interniAttivi) {
        if (casoMixerPrecedente != 1) {
            inviaMessaggioAvionica("Caso B: interni rotti — esterni fanno pitch + roll");
            casoMixerPrecedente = 1;
        }
        posEstSX_deg = CENTRO_SERVO_deg + pitch_deg + roll_deg;
        posEstDX_deg = CENTRO_SERVO_deg + pitch_deg - roll_deg;
    } else if (!esterniAttivi && interniAttivi) {
        if (casoMixerPrecedente != 2) {
            inviaMessaggioAvionica("Caso C: esterni rotti — interni fanno pitch + roll");
            casoMixerPrecedente = 2;
        }
        posIntSX_deg = CENTRO_SERVO_deg + pitch_deg + roll_deg;
        posIntDX_deg = CENTRO_SERVO_deg + pitch_deg - roll_deg;
    } else {
        if (casoMixerPrecedente != 3) {
            inviaMessaggioAvionica("Caso D: tutti i servi rotti: nulla da comandare");
            casoMixerPrecedente = 3;
        }
        return;
    }

    // Attach/detach automatico in base a se i servi sono considerati attivi o no
    if (interniAttivi != statoPrecedenteInterni) {
        if (interniAttivi) {
            servoInternoSX.attach(PIN_INT_SX);
            servoInternoDX.attach(PIN_INT_DX);
            inviaMessaggioAvionica("Servi interni: ATTIVATI");
        } else {
            servoInternoSX.detach();
            servoInternoDX.detach();
            inviaMessaggioAvionica("Servi interni: STACCATI");
        }
        statoPrecedenteInterni = interniAttivi;
    }

    if (esterniAttivi != statoPrecedenteEsterni) {
        if (esterniAttivi) {
            servoEsternoSX.attach(PIN_EST_SX);
            servoEsternoDX.attach(PIN_EST_DX);
            inviaMessaggioAvionica("Servi esterni: ATTIVATI");
        } else {
            servoEsternoSX.detach();
            servoEsternoDX.detach();
            inviaMessaggioAvionica("Servi esterni: STACCATI");
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
    if (!servoSicurezza) {
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
            inviaMessaggioAvionica("WARN: ServoEstSX assunzione corrente fuori limiti per troppo tempo");
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
            inviaMessaggioAvionica("WARN: ServoEstDX assunzione corrente fuori limiti per troppo tempo");
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
            inviaMessaggioAvionica("WARN: ServoIntSX assunzione corrente fuori limiti per troppo tempo");
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
            inviaMessaggioAvionica("WARN: ServoIntDX assunzione corrente fuori limiti per troppo tempo");
        }
    } else {
        erroriConsecutivi[3] = 0;
        intDxOk = true;
    }
}


void gestisciAlimentazione() {

    unsigned long tempo_attuale = millis();
    float dt_ore = (tempo_attuale - TEMPO_BATTERIA_PRECEDENTE_ms) / 3600000.0;
    
    iTeensy = sensoreTeensy.getCurrent_mA();
    iMotore = sensoreMotore.getCurrent_mA();
    vTeensy = sensoreTeensy.getBusVoltage_V();
    vMotore = sensoreMotore.getBusVoltage_V();
    
    G_carica_consumata_teensy += (iTeensy * dt_ore);
    G_carica_consumata_motore += (iMotore * dt_ore);
    
    // 4. Calcolo percentuale residua
    G_carica_rimanente_teensy_percentuale = ((CAPACITA_TEENSY - G_carica_consumata_teensy) / CAPACITA_TEENSY) * 100.0;
    G_carica_rimanente_MOTORE_percentuale = ((CAPACITA_MOTORE - G_carica_consumata_motore) / CAPACITA_MOTORE) * 100.0;
    
    // 5. Calcolo dell'autonomia residua in ore
    if (iTeensy > 0.0) {
        G_autonomia_teensy_residua = (CAPACITA_TEENSY - G_carica_consumata_teensy) / iTeensy;
    } else {
        G_autonomia_teensy_residua = -1;
    }

    if (iMotore > 0.0) {
        G_autonomia_motore_residua = (CAPACITA_MOTORE - G_carica_consumata_motore) / iMotore;
    } else {
        G_autonomia_motore_residua = -1;
    }
    
    // 6. Gestione Alimentazione (Sicurezza e Failover)
    if (!alimentazioneSicurezza) {
        batteriaBassaTeensy = false;
        batteriaBassaMotore = false;
    } else {
        // FIX (obiettivo 2): edge detection sui due allarmi batteria, prima silenziosi.
        // Una batteria bassa non segnalata via radio è il classico modo in cui un volo finisce
        // prima del previsto senza che a terra ce ne si accorga in tempo.
        bool batteriaBassaTeensyPrecedente = batteriaBassaTeensy;
        bool batteriaBassaMotorePrecedente = batteriaBassaMotore;

        // Controllo Batteria Teensy
        batteriaBassaTeensy = (vTeensy < VALORE_BATT_TEENSY_BASSA_V);
        
        if (batteriaBassaTeensy && !batteriaBassaTeensyPrecedente) {
            inviaMessaggioAvionica("ATTENZIONE: batteria Teensy bassa, commutazione su batteria motore apertura relè");
        }

        if (batteriaBassaTeensy) {
            if (!releAttivato) {
                digitalWrite(PIN_RELE, HIGH);   // Attiva il relè: la batteria motore subentra
                releAttivato = true;
                inviaMessaggioAvionica("Rele' alimentazione ATTIVATO (failover su batteria motore)");
            }
        }

        // Controllo Batteria Motore
        batteriaBassaMotore = (vMotore < VALORE_BATT_MOTORE_BASSA_V);
        if (batteriaBassaMotore && !batteriaBassaMotorePrecedente) {
            inviaMessaggioAvionica("ATTENZIONE: batteria motore bassa, considerare atterraggio, semo fottuti");
        }
    }
    
    TEMPO_BATTERIA_PRECEDENTE_ms = tempo_attuale;
}

void gestisciSchianto() {
    if (!schiantoSicurezza) {
        statoSchiantoRilevato = false;
        return;
    }
    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO_us); // Mantiene il motore spento
        return;
    }

    if (!droneInVolo) return; // Non controlla schianti a terra

    imu::Vector<3> accel = giroscopio.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    float accelerazioneTotale_ms2 = sqrt((accel.x() * accel.x()) + (accel.y() * accel.y()) + (accel.z() * accel.z()));

    // Salvataggio per diagnostica/telemetria
    ACCEL_X_ms2 = accel.x();
    ACCEL_Y_ms2 = accel.y();
    ACCEL_Z_ms2 = accel.z();
    ACCEL_TOTALE_ms2 = accelerazioneTotale_ms2;

    static unsigned long tempoInizioPicco_ms = 0;
    static bool piccoInCorso = false;
    unsigned long tempoAttuale_ms = millis();
    
    
    const float SOGLIA_VELOCITA_CRITICA_ms = 5.0f;      

    if (accelerazioneTotale_ms2 > SOGLIA_ACCELERAZIONE_SCHIANTO_ms2) {
        if (!piccoInCorso) {
            // È il primissimo frame in cui rileviamo l'anomalia. Facciamo partire il cronometro.
            piccoInCorso = true;
            tempoInizioPicco_ms = tempoAttuale_ms;
        } else {
            // Il picco persiste. Controlliamo da quanto tempo.
            if ((tempoAttuale_ms - tempoInizioPicco_ms) >= TEMPO_CONFERMA_SCHIANTO_ms) {
                if (VELOCITA_ARIA_ms < SOGLIA_VELOCITA_CRITICA_ms) {
                    
                    inviaMessaggioAvionica("CRITICAL: SCHIANTO CONFERMATO ");
                    statoSchiantoRilevato = true;
                    schiantoBloccato = true;
                    inviaMessaggioAvionica("CRITICAL: DISATTIVO SERVI E MOTORE");
                    motore.writeMicroseconds(GAS_NEUTRO_us);
                    servoInternoSX.detach();
                    servoInternoDX.detach();
                    servoEsternoSX.detach();
                    servoEsternoDX.detach();
                    inviaMessaggioAvionica("LUCI E ALLARME ATTIVATI");
                    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
                    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
                    digitalWrite(PIN_LED_BLU_PID, HIGH);
                    tone(PIN_BUZZER, 2000);
                    inviaMessaggioAvionica("per armare il drone: Porta il selettore del canale 5 in posizione Manuale (portandolo sotto la soglia S.BUS di 992).");
                }
            }
        }
    } else {
        // L'accelerazione è tornata sotto la soglia. Reset pulito del timer.
        piccoInCorso = false;
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
                inviaMessaggioAvionica("velocita e altitudine di decollo confermate");
                droneInVolo = true;
                TIMESTAMP_DECOLLO_ms = 0;
                inviaMessaggioAvionica("DECOLLO CONFERMATO: drone in volo, monitoraggio schianto attivo");
            }
        } else {
            TIMESTAMP_DECOLLO_ms = 0;
        }
    }
}

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
    digitalWrite(PIN_LED_VERDE_GPS, Errore_gps>0 ? HIGH : LOW);

    // 4. LED BLU: modalità di volo (acceso = AUTO)
    digitalWrite(PIN_LED_BLU_PID, (global_modalitaVolo == 2) ? HIGH : LOW);
}

void aggiornaDiagnosticaIMU() {
    giroscopio.getCalibration(&IMU_CAL_SYS, &IMU_CAL_GYRO, &IMU_CAL_ACCEL, &IMU_CAL_MAG);
}



void inviaTelemetria(float pitch_deg, float roll_deg, float yaw_deg,
                     float velAria_kmh, float velSuoloGps_kmh,
                     int outPitch_deg, int outRoll_deg, int outGas_us) {

    // NOTA: le tensioni/correnti NON vengono più rilette qui (evitava letture I2C ridondanti
    // e una variabile locale "vTeensy" che oscurava quella globale). Si riusano i valori
    // globali già aggiornati una volta per ciclo da gestisciAlimentazione().
    float vIntSX  = sensoreIntSX.getBusVoltage_V();
    float vIntDX  = sensoreIntDX.getBusVoltage_V();
    float vEstSX  = sensoreEstSX.getBusVoltage_V();
    float vEstDX  = sensoreEstDX.getBusVoltage_V();

    TELEMETRIA.print("$,"); // 0. Start indicatore pacchetto TEL1

    // --- STATO E ALLARMI ---
    TELEMETRIA.print(global_modalitaVolo); TELEMETRIA.print(",");
    TELEMETRIA.print(droneInVolo ? "1" : "0"); TELEMETRIA.print(","); 
    TELEMETRIA.print(failsafe ? "1" : "0"); TELEMETRIA.print(","); 
    TELEMETRIA.print(statoSchiantoRilevato ? "1" : "0"); TELEMETRIA.print(","); 
    TELEMETRIA.print(imuPronto? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(flussoOtticoOk? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(baroPronto? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(sensoriCorrenteOk? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(gpsOk? "1":"0");TELEMETRIA.print(",");
    // FIX (obiettivo 2): prima veniva trasmesso solo IMU_CAL_SYS, che da solo non basta a capire
    // COSA non è ancora calibrato (giroscopio/accelerometro/magnetometro hanno dinamiche diverse).
    // Aggiunti i 3 sotto-livelli di calibrazione, già letti da aggiornaDiagnosticaIMU() ma mai inviati.
    TELEMETRIA.print(IMU_CAL_SYS); TELEMETRIA.print(",");
    TELEMETRIA.print(IMU_CAL_GYRO); TELEMETRIA.print(",");
    TELEMETRIA.print(IMU_CAL_ACCEL); TELEMETRIA.print(",");
    TELEMETRIA.print(IMU_CAL_MAG); TELEMETRIA.print(",");

    // --- ALIMENTAZIONE (Volt, V) ---
    TELEMETRIA.print(vMotore, 2); TELEMETRIA.print(","); 
    TELEMETRIA.print(vTeensy, 2); TELEMETRIA.print(","); 

    TELEMETRIA.print(G_carica_rimanente_teensy_percentuale, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_carica_rimanente_MOTORE_percentuale, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_autonomia_teensy_residua, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_autonomia_motore_residua, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(iTeensy, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(iMotore, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(vIntSX, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(vIntDX, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(vEstSX, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(vEstDX, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(servoSicurezza ? "1" : "0"); TELEMETRIA.print(",");

    // --- ASSETTO E QUOTA (IMU) ---
    TELEMETRIA.print(pitch_deg, 1); TELEMETRIA.print(","); 
    TELEMETRIA.print(roll_deg, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(yaw_deg, 1);   TELEMETRIA.print(","); 
            
    TELEMETRIA.print(ALTITUDINE_m, 1); TELEMETRIA.print(","); 
    TELEMETRIA.print(ALTITUDINE_LIDAR_m, 2); TELEMETRIA.print(","); 
    TELEMETRIA.print(ALTITUDINE_BARO_m, 2);  TELEMETRIA.print(","); 
    // FIX (obiettivo 2): la quota TARGET non veniva mai trasmessa. Senza questo dato il PFD a terra
    // non può disegnare il "bug" di quota target rispetto alla quota reale.
    TELEMETRIA.print(ALTITUDINE_TARGET_m, 1); TELEMETRIA.print(",");

    // --- VELOCITÀ ---
    TELEMETRIA.print(velAria_kmh, 1);      TELEMETRIA.print(","); 
    TELEMETRIA.print(velSuoloGps_kmh, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(VENTO_VELOCITA_ms * 3.6f, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(VENTO_DIREZIONE_deg, 1);       TELEMETRIA.print(","); 
    TELEMETRIA.print(VELOCITA_CROCIERA_kmh, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(VELOCITA_AVVICINAMENTO_kmh, 1);       TELEMETRIA.print(","); 


    // --- NAVIGAZIONE ---
    TELEMETRIA.print(DISTANZA_TARGET_m, 0); TELEMETRIA.print(","); 
    TELEMETRIA.print(ROTTA_TARGET_deg, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(ROLL_TARGET_deg, 1);   TELEMETRIA.print(","); 
    TELEMETRIA.print(rottaAttuale_deg, 1);     TELEMETRIA.print(",");

    // --- INPUT RADIOCOMANDO ---
    TELEMETRIA.print(canaliRC[1]); TELEMETRIA.print(","); 
    TELEMETRIA.print(canaliRC[0]); TELEMETRIA.print(","); 
    TELEMETRIA.print(canaliRC[2]); TELEMETRIA.print(","); 

    // --- OUTPUT PID/MIXER ---
    // FIX: outPitch_deg è un int (non un float): il secondo argomento ",1" veniva
    // interpretato da Print come BASE numerica (non decimali) -> valore trasmesso sbagliato.
    TELEMETRIA.print(outPitch_deg); TELEMETRIA.print(","); 
    TELEMETRIA.print(outRoll_deg);  TELEMETRIA.print(","); 
    TELEMETRIA.print(outGas_us);    TELEMETRIA.print(","); 

    // --- POSIZIONE FISICA ATTUALE SERVI ---
    TELEMETRIA.print(servoInternoSX.read()); TELEMETRIA.print(","); 
    TELEMETRIA.print(servoInternoDX.read()); TELEMETRIA.print(","); 
    TELEMETRIA.print(servoEsternoSX.read()); TELEMETRIA.print(","); 
    TELEMETRIA.print(servoEsternoDX.read()); TELEMETRIA.print(","); 

    // --- TEMPERATURE (°C) E LIMITI ---
    TELEMETRIA.print(limitazioneTermicaAttiva ? "1" : "0"); TELEMETRIA.print(",");
    TELEMETRIA.print(TEMPERATURA_MOTORE_C, 1);    TELEMETRIA.print(","); 
    TELEMETRIA.print(TEMPERATURA_FUSOLIERA_C, 1); TELEMETRIA.print(","); 
    TELEMETRIA.print(TEMPERATURA_ESTERNA_C, 1);   TELEMETRIA.print(","); 
    TELEMETRIA.print(TEMPERATURA_ESC_C, 1);       TELEMETRIA.print(","); 
    TELEMETRIA.print(GAS_LIMITE_TERMICO_us);      TELEMETRIA.print(","); 

    // --- SATELLITI E COORDINATE GPS ---
    // FIX: "errore_gps" non esisteva (refuso) -> "Errore_gps"
    if (Errore_gps>0) {
        // FIX: numero_satelliti è un int: ",1" veniva letto come BASE, non come decimali -> rimosso.
        TELEMETRIA.print(numero_satelliti); TELEMETRIA.print(","); 
        TELEMETRIA.print(DRONE_LAT_deg, 6);  TELEMETRIA.print(","); 
        TELEMETRIA.print(DRONE_LON_deg, 6);  TELEMETRIA.print(","); 
        
        // FIX: stesso problema per Errore_gps (int, non float) -> rimosso ",1"
        TELEMETRIA.print(Errore_gps);     TELEMETRIA.print(","); 
        
        TELEMETRIA.print(ERRORE_ROTTA_deg, 1);    TELEMETRIA.print(","); 
    } else {
        TELEMETRIA.print("0,0.0,0.0,-1.0,0.0,"); 
    }
    
    // --- STATI FINALI ---
    TELEMETRIA.print(alimentazioneSicurezza ? "1" : "0"); TELEMETRIA.print(",");
    TELEMETRIA.print(releAttivato ? "1" : "0");           TELEMETRIA.print(",");
    TELEMETRIA.print(motoreDisabilitatoDaTerra ? "1" : "0"); TELEMETRIA.print(",");

    // --- FLUSSO OTTICO ---
    TELEMETRIA.print(VELOCITA_OTTICA_X_ms, 2); TELEMETRIA.print(","); 
    TELEMETRIA.print(VELOCITA_OTTICA_Y_ms, 2); // Ultimo dato senza virgola finale

    TELEMETRIA.println(); // Chiusura pacchetto (CRLF)
}

// Legge comandi testuali (terminati da '\n') sia da USB (Serial) che da LoRa (TELEMETRIA) e li passa al parser
void comandiDaTerra() {
    // FIX: prima c'era un unico buffer condiviso tra le due porte seriali.
    // Se arrivavano byte da entrambe nella stessa finestra, i caratteri si
    // mescolavano corrompendo il comando. Ora ogni porta ha il suo buffer.
    static String buffer[2] = { "", "" };
    Stream* fonti[] = { &TELEMETRIA, &Serial };
    for (int i = 0; i < 2; i++) {
        Stream* porta = fonti[i];
        while (porta->available()) {
            char c = porta->read();
            if (c == '\n') {
                segnalaOK();               // Beep di conferma ricezione riga di comando
                elaboraComando(buffer[i]); // Interpreta il comando accumulato
                buffer[i] = "";
            } else {
                buffer[i] += c;
                if (buffer[i].length() > 64) buffer[i] = "";   // Protezione overflow: scarta il buffer se supera 64 caratteri senza newline
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
        servoSicurezza = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SERVI_OFF") {
        servoSicurezza = false;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_TEMP_ON") {
        limitazione_termica_gas = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_TEMP_OFF") {
        limitazione_termica_gas = false;
        inviaAck(campo, "");

    // Gas — solo comandabile manualmente in modalità 1, in microsecondi (us), vincolato tra GAS_NEUTRO_us e GAS_MASSIMO_us
    } else if (campo == "GAS") {
        if (motoreDisabilitatoDaTerra) {
            // FIX (obiettivo 3): se il kill switch software è attivo, il comando GAS manuale
            // deve essere rifiutato esplicitamente, altrimenti sembrerebbe accettato ma poi
            // il loop() lo sovrascrive comunque al neutro (comportamento confuso da terra).
            inviaNack(campo, "motore_disabilitato_da_terra");
        } else if (global_modalitaVolo == 1) {
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

    // FIX (obiettivo 3): kill switch software del motore, richiesto esplicitamente ("spegnimento
    // motore" da terra). Non basta scrivere GAS_NEUTRO_us una volta: il loop() ricalcola il gas
    // ad ogni ciclo (manuale o PID), quindi serve un flag persistente controllato in loop().
    } else if (campo == "STOP_MOTORE") {
        motoreDisabilitatoDaTerra = true;
        motore.writeMicroseconds(GAS_NEUTRO_us);
        inviaMessaggioAvionica("MOTORE DISABILITATO DA TERRA (kill switch software attivo)");
        inviaAck(campo, "");

    } else if (campo == "RIPRISTINA_MOTORE") {
        motoreDisabilitatoDaTerra = false;
        inviaMessaggioAvionica("Motore ripristinato, kill switch software disattivato");
        inviaAck(campo, "");

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


    // ─── COMANDI: PARAMETRI OPERATIVI ───
    } else if (campo == "SET_VEL_CROCIERA") {
        float v = valoreStr.toFloat();   // km/h
        if (v >= 20.0 && v <= 150.0) { VELOCITA_CROCIERA_kmh = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_VEL_AVVICINAMENTO") {
        float v = valoreStr.toFloat();   // km/h
        if (v >= 15.0 && v <= 150.0) { VELOCITA_AVVICINAMENTO_kmh = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

    // ─── COMANDI: SINTONIZZAZIONE GUADAGNI PID DA TERRA ───
    // FIX (obiettivo 3): i guadagni erano "const" quindi non tarabili a runtime, mentre i limiti
    // di validazione LIMITE_KP_MAX/KI_MAX/KD_MAX erano già dichiarati in cima al file ma inutilizzati.
    // Ogni comando valida il segno (guadagno fisicamente sempre >= 0) e il tetto massimo, poi
    // resetta gli integrali del PID interessato per evitare transitori dovuti al cambio di guadagno.
    } else if (campo == "SET_KP_ROLL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KP_MAX) { Kp_roll = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_ROLL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KI_MAX) { Ki_roll = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_ROLL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KD_MAX) { Kd_roll = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

    } else if (campo == "SET_KP_PITCH") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KP_MAX) { Kp_pitch = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_PITCH") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KI_MAX) { Ki_pitch = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_PITCH") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KD_MAX) { Kd_pitch = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

    } else if (campo == "SET_KP_ALT") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KP_MAX) { Kp_alt = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_ALT") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KI_MAX) { Ki_alt = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_ALT") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KD_MAX) { Kd_alt = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

    } else if (campo == "SET_KP_VEL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KP_MAX) { Kp_vel = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KI_VEL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KI_MAX) { Ki_vel = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_KD_VEL") {
        float v = valoreStr.toFloat();
        if (v >= 0.0f && v <= LIMITE_KD_MAX) { Kd_vel = v; resettaPID(); inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

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


//  SETUP
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

    

    ricevente.begin();
    TELEMETRIA.begin(BAUD_RATE_LORA);
    GPS_SERIAL.begin(BAUD_RATE_GPS);
    Serial2.begin(BAUD_RATE_LIDAR);
    delay(100);
    inviaMessaggioAvionica("     SISTEMA DRONE — AVVIO IN CORSO     ");
    while (Serial2.available()) Serial2.read();   // Svuota eventuali byte residui nel buffer seriale del LIDAR
    inviaMessaggioAvionica("     INIZIALIZZAZIONE SENSORI — ciclo ripetuto fino a MAX_TENTATIVI_INIT   ");
    while ((!imuPronto || !baroPronto || !pitotCalibrato || !sensoriCorrenteOk || !gpsOk) && tentativiInit < MAX_TENTATIVI_INIT) {
        tentativiInit++;
        inviaMessaggioAvionica("  Tentativo ");
        inviaMessaggioAvionica(String(tentativiInit));

        // --- Flusso ottico: tentativo di inizializzazione via SPI ---
        if (!flussoOtticoOk) {
            inviaMessaggioAvionica("[ ] Flusso Ottico PMW3901 ........... ");
            if (flussoOttico.begin()) {
                inviaMessaggioAvionica("[OK] Flusso Ottico PMW3901");
                flussoOtticoOk = true;
            } else {
                inviaMessaggioAvionica("ERRORE sensore flusso ottico (cavi SPI?)");
                segnalaErrore();
            }
        } else {
            inviaMessaggioAvionica("[OK] Flusso Ottico PMW3901");
        }

        // --- LIDAR TF-Luna: attende 3000 ms un pacchetto valido con header 0x59 0x59 ---
        if (!lidarOk) {
            inviaMessaggioAvionica("[ ] TF-Luna LIDAR .............. ");
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
                inviaMessaggioAvionica("[OK] TF-Luna LIDAR");
                segnalaOK();
            } else {
                inviaMessaggioAvionica("[WARN] TF-Luna LIDAR assente — continuo senza");
            }
        } else {
            inviaMessaggioAvionica("[OK] TF-Luna LIDAR");
        }
        // --- 5. GPS: Verifica comunicazione hardware e decodifica stream ---
        if (!gpsOk) {
            inviaMessaggioAvionica("[ ] Modulo GPS (UART) ............ ");
            unsigned long t_gps = millis();
            while (millis() - t_gps < 1500) {
                // FIX: "SerialGPS" non esisteva (la seriale del GPS è definita come GPS_SERIAL)
                while (GPS_SERIAL.available() > 0) {
                    gps.encode(GPS_SERIAL.read());
                }
            }
            aggiornaGPS();

            if (Errore_gps <= 0) {
                inviaMessaggioAvionica("ERRORE : ");
                inviaMessaggioAvionica(String(Errore_gps));
                segnalaErrore();
            } else {
                gpsOk = true;
                inviaMessaggioAvionica("[OK] Modulo GPS operativo (Stato diagnostico: ");
                inviaMessaggioAvionica(String(Errore_gps));
                inviaMessaggioAvionica(")");
                segnalaOK();
            }
        } else {
            inviaMessaggioAvionica("[OK] Modulo GPS");
        }
        // 1. IMU — inizializzazione, calibrazione interna e tara offset
        if (!imuPronto) {
            inviaMessaggioAvionica("[ ] IMU BNO055 ................. ");
            if (giroscopio.begin()) {
                giroscopio.setExtCrystalUse(true);   // Cristallo esterno per un clock più stabile
                Serial.println("OK");

                // Calibrazione interna: attende che il giroscopio raggiunga almeno 2 su 3
                inviaMessaggioAvionica("   Calibrazione interna (non muovere)");
                uint8_t sys, gyro, accel, mag;
                unsigned long timeout_ms = millis();
                do {
                    giroscopio.getCalibration(&sys, &gyro, &accel, &mag);
                    inviaMessaggioAvionica(".");

                    delay(100);
                    if (millis() - timeout_ms > 10000) {   // Timeout massimo: 10000 ms
                        Serial.println(" timeout, continuo");
                        break;
                    }
                } while (gyro < 2);

                // Tara: media di IMU_CAMPIONI_TARA campioni per l'offset statico di roll/pitch/yaw
                inviaMessaggioAvionica("\n   Tara offset in corso...");
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
                inviaMessaggioAvionica(" imu offsets: roll= ");
                inviaMessaggioAvionica(String(OFFSET_ROLL_deg, 2));
                inviaMessaggioAvionica(" ; pitch= ");
                inviaMessaggioAvionica(String(OFFSET_PITCH_deg, 2));
                inviaMessaggioAvionica(" ; yaw= ");
                inviaMessaggioAvionica(String(OFFSET_YAW_deg, 2));
            } else {
                inviaMessaggioAvionica("\n ERRORE (cavi I2C?)");
                segnalaErrore();
            }
        } else {
            inviaMessaggioAvionica(" \n [OK] IMU BNO055");
        }

        // 2. BAROMETRO — inizializzazione, oversampling, filtro, tara altitudine ASL
        if (!baroPronto) {
            inviaMessaggioAvionica("[ ] Barometro BMP390 ....... ");
            if (barometro.begin_I2C()) {
                inviaMessaggioAvionica("Settaggio oversempling ....... ");
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
                inviaMessaggioAvionica("Calibrazione barometro ....... ");
                for (int i = 0; i < 20; i++) {   // 20 campioni per la media di tara
                    segnalaCalibrazione(PIN_LED_BLU_PID);

                    float altitudineIstantanea_m = barometro.readAltitude(1013.25);   // Rispetto a 1013.25 hPa

                    // Validazione hardware: scarta letture fisicamente impossibili (range plausibile: -500..8000 m ASL)
                    if (altitudineIstantanea_m < -500.0 || altitudineIstantanea_m > 8000.0) {
                        inviaMessaggioAvionica("\n ERRORE: Lettura barometrica impossibile");
                        inviaMessaggioAvionica(" Altitudine letta: ");
                        inviaMessaggioAvionica(String(altitudineIstantanea_m));
                        inviaMessaggioAvionica(" m");

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
                    inviaMessaggioAvionica("OK (Tara ASL corretta da LIDAR: ");
                } else {
                    TARA_ALTITUDINE_BARO_m = mediaBaroCalibrazione_m;
                    inviaMessaggioAvionica("OK (Tara ASL standard: ");
                }

                inviaMessaggioAvionica(String(TARA_ALTITUDINE_BARO_m, 1));
                inviaMessaggioAvionica(" m)");
                segnalaOK();

            } else {
                inviaMessaggioAvionica("ERRORE (cavi I2C?)");
                segnalaErrore();
            }
        } else {
            inviaMessaggioAvionica("[OK] Barometro BMP390");
        }
        // 3. PITOT — calibrazione dello zero
        if (!pitotCalibrato) {
            inviaMessaggioAvionica("[ ] Pitot (velocita') ...... ");
            long sommaLetture_adc = 0;
            inviaMessaggioAvionica("Calibrazione pitot ....... ");
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
                inviaMessaggioAvionica("OK (zero: ");
                inviaMessaggioAvionica(String(PITOT_ZERO_adc, 1));
                inviaMessaggioAvionica(")");
                segnalaOK();
            } else {
                inviaMessaggioAvionica("ERRORE (valore anomalo: ");
                inviaMessaggioAvionica(String(PITOT_ZERO_adc));
                inviaMessaggioAvionica(")");
                segnalaErrore();
            }
        } else {
            inviaMessaggioAvionica("[OK] Pitot");
        }

        // 4. INA219 — verifica che tutti i sensori di corrente/tensione rispondano sul bus I2C
        sensoriCorrenteOk = true;
        inviaMessaggioAvionica("[ ] INA219 Batteria motore........ ");
        if (sensoreMotore.begin()) { inviaMessaggioAvionica("OK"); } else { inviaMessaggioAvionica("ERRORE"); sensoriCorrenteOk = false; }

        inviaMessaggioAvionica("[ ] INA219 Batteria Teensy........ ");
        if (sensoreTeensy.begin()) { inviaMessaggioAvionica("OK"); } else { inviaMessaggioAvionica("ERRORE"); sensoriCorrenteOk = false; }

        inviaMessaggioAvionica("[ ] INA219 Servo IntSX ..... ");
        if (sensoreIntSX.begin()) { inviaMessaggioAvionica("OK"); } else { inviaMessaggioAvionica("ERRORE"); sensoriCorrenteOk = false; }

        inviaMessaggioAvionica("[ ] INA219 Servo IntDX ..... ");
        if (sensoreIntDX.begin()) { inviaMessaggioAvionica("OK"); } else { inviaMessaggioAvionica("ERRORE"); sensoriCorrenteOk = false; }

        inviaMessaggioAvionica("[ ] INA219 Servo EstSX ..... ");
        if (sensoreEstSX.begin()) { inviaMessaggioAvionica("OK"); } else { inviaMessaggioAvionica("ERRORE"); sensoriCorrenteOk = false; }

        inviaMessaggioAvionica("[ ] INA219 Servo EstDX ..... ");
        if (sensoreEstDX.begin()) { inviaMessaggioAvionica("OK"); } else { inviaMessaggioAvionica("ERRORE"); sensoriCorrenteOk = false; }

        if (sensoriCorrenteOk) { segnalaOK(); } else { segnalaErrore(); }


        if (!imuPronto || !baroPronto || !pitotCalibrato || !sensoriCorrenteOk || !gpsOk) {
            inviaMessaggioAvionica("\n  >> Sensori mancanti. Nuovo tentativo tra 2s...");
            digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
            delay(2000);
            digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        }
    }

    // ERRORE CRITICO — se dopo MAX_TENTATIVI_INIT il modulo GPS non parla con la Teensy, BLOCCO TOTALE
    if (!imuPronto || !baroPronto || !pitotCalibrato || !sensoriCorrenteOk || !gpsOk) {
        errore_critico_inizializzazione = true;
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        inviaMessaggioAvionica("\n[FATAL ERROR] Fallimento inizializzazione hardware critico. Sistema bloccato.");
        while (1) {   // Blocco infinito
            tone(PIN_BUZZER, 2000, 300);
            delay(400);
        }
    }
    inviaMessaggioAvionica("\n  >> Sensori inizializzati correttamente\n");
    // TUTTO OK — INIT SERVO
    inizializzaServo();    // Attacca tutti i servi e li porta al centro (90°)
    inizializzaMotore();   // Attacca l'ESC e invia comando neutro (GAS_NEUTRO_us)

    // Jingle avvio riuscito
    tone(PIN_BUZZER, 800,  120); delay(170);
    tone(PIN_BUZZER, 1200, 120); delay(170);
    tone(PIN_BUZZER, 1800, 200); delay(350);

    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    digitalWrite(PIN_LED_BLU_PID, HIGH);
    delay(1000);   // Tiene tutti i LED accesi 1 s
    digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
    digitalWrite(PIN_LED_BLU_PID, LOW);

    TEMPO_PID_PRECEDENTE_ms = millis();   // Riferimento temporale per il primo calcolo PID
    TEMPO_BATTERIA_PRECEDENTE_ms = millis();
    inviaMessaggioAvionica("\n  >> SISTEMA PRONTO AL VOLO\n");
    delay(500);
}

// Collega i 4 servocomandi ai rispettivi pin e li porta tutti alla posizione centrale (90°)
void inizializzaServo() {
    inviaMessaggioAvionica("Inizializzazione servi in corso...");
    servoInternoSX.attach(PIN_INT_SX);
    servoInternoDX.attach(PIN_INT_DX);
    servoEsternoSX.attach(PIN_EST_SX);
    servoEsternoDX.attach(PIN_EST_DX);
    inviaMessaggioAvionica("Servo inizializzati.Posizione neutra (90°) inviata a tutti i servi.");
    servoInternoSX.write(CENTRO_SERVO_deg);
    servoInternoDX.write(CENTRO_SERVO_deg);
    servoEsternoSX.write(CENTRO_SERVO_deg);
    servoEsternoDX.write(CENTRO_SERVO_deg);
}

// Collega l'ESC del motore e invia il comando neutro
void inizializzaMotore() {
    inviaMessaggioAvionica("Inizializzazione motore in corso...");
    motore.attach(PIN_MOTORE);
    inviaMessaggioAvionica("Motore inizializzato. Comando neutro inviato (GAS_NEUTRO_us).");
    motore.writeMicroseconds(GAS_NEUTRO_us);
}

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

    aggiornaDiagnosticaIMU();   // Aggiorna i livelli di calibrazione IMU (0-3) per diagnostica/telemetria

    // 3. Temperatura motore/ESC/esterna
    leggiTemperatura();

    // 4. Pitot — velocità ARIA (nessun fallback su altre velocità)
    leggiPitot();

    // 5. GPS — velocità al SUOLO grezza (Ground Speed)
    // FIX: rinominata da gps() ad aggiornaGPS() (collisione col nome dell'oggetto TinyGPSPlus gps)
    aggiornaGPS();

    // 6. Barometro — altitudine relativa, pressione, temperatura aria
    leggiBarometro();

    aggiornaDensitaAria(PRESSIONE_BARO_Pa, TEMPERATURA_FUSOLIERA_C);
    aggiornaLidar();
    selezionaAltitudine();                    // metri (m)
    leggiVelocitaOttica(yaw_deg);              // m/s
    aggiornaVelocitaSuolo(velocitaSuoloGps_ms); // m/s
    stimaVento(yaw_deg);                       // m/s e direzione (0-360°)
    aggiornaNavigazione(yaw_deg);

    // 7. PREPARAZIONE DATI MOTORE — sceglie velocità target e gas di base in base alla distanza dal target
    float targetVelocita_kmh = 0.0f;
    int gasDiBase_us = 0;
    static int fasciaTargetPrecedente = -1;
    if (DISTANZA_TARGET_m > DISTANZA_FRENATA_m) {
        if (fasciaTargetPrecedente != 1) {
            inviaMessaggioAvionica("Target lontano: velocita' crociera");
            fasciaTargetPrecedente = 1;
        }
        targetVelocita_kmh = VELOCITA_CROCIERA_kmh;
        gasDiBase_us = GAS_CROCIERA_us;
    } else {
        if (fasciaTargetPrecedente != 0) {
            inviaMessaggioAvionica("Target vicino: velocita' avvicinamento");
            fasciaTargetPrecedente = 0;
        }
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
            inviaMessaggioAvionica("!!! SBLOCCO EMERGENZA ESEGUITO DA RADIO !!! Servi Riarmati e centrati.");
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
        inviaMessaggioAvionica("Cambio modalità di volo: reset PID , azzerati integrali/derivate");
        modalitaPrecedente = statoAttuale;
    }

    if (!schiantoBloccato && global_modalitaVolo == 1 && !failsafe) {
        // MODALITÀ MANUALE: mappa i canali RC grezzi direttamente sui comandi
        // Gas: da 172-1811 (range SBUS) a GAS_NEUTRO_us-GAS_MASSIMO_us, vincolato ai limiti meccanici motore
        comandoGasFinale_us = constrain(map(canaliRC[2], 172, 1811, GAS_NEUTRO_us, GAS_MASSIMO_us), GAS_NEUTRO_us, GAS_MASSIMO_us);
        correzioneRoll_deg  = constrain(map(canaliRC[0], 172, 1811, -MAX_ROLL_deg,   MAX_ROLL_deg),   -MAX_ROLL_deg,  MAX_ROLL_deg);
        correzionePitch_deg = constrain(map(canaliRC[1], 172, 1811,  MAX_PITCH_deg, -MAX_PITCH_deg),  -MAX_PITCH_deg, MAX_PITCH_deg);
        if (statoAttuale != ultimoStatoStampato) {
            inviaMessaggioAvionica("Volo: MANUALE (Comandi diretti dal radiocomando)");
            ultimoStatoStampato = statoAttuale;
        }
    } else if (!schiantoBloccato && (global_modalitaVolo == 2 || failsafe) && (droneInVolo || failsafe)) {
        // MODALITÀ AUTO o FAILSAFE: il PID calcola i comandi automaticamente
        if (failsafe) {
            if (statoAttuale != ultimoStatoStampato) {
                inviaMessaggioAvionica("FAILSAFE ATTIVO! direzione a target automatico!");
                ultimoStatoStampato = statoAttuale;
            }
        } else {
            if (statoAttuale != ultimoStatoStampato) {
                inviaMessaggioAvionica("Volo: GPS AUTO (Il PID comanda)");
                ultimoStatoStampato = statoAttuale;
            }
        }
        // Chiamata al PID: quota target (m), roll target (°), pitch/roll reali (°), velocità aria attuale/target (km/h), gas di base (us)
        calcolaPID(ALTITUDINE_TARGET_m, ROLL_TARGET_deg, pitch_deg, roll_deg,
                   VELOCITA_ARIA_ms * 3.6f, targetVelocita_kmh, gasDiBase_us,
                   correzionePitch_deg, correzioneRoll_deg, comandoGasFinale_us);
    }

    // FIX (obiettivo 3): applicazione del kill switch software STOP_MOTORE. Va fatta qui,
    // subito prima delle scritture finali sul motore, così sovrascrive sia il calcolo manuale
    // che quello automatico/PID senza toccarne la logica interna.
    if (motoreDisabilitatoDaTerra) {
        comandoGasFinale_us = GAS_NEUTRO_us;
        inviaMessaggioAvionica("Comando STOP_MOTORE ricevuto da terra: motore disabilitato impostatoa GAS_NEUTRO_us");
    }

    if (statoSchiantoRilevato) {
 
        comandoGasFinale_us = GAS_NEUTRO_us;
        motore.writeMicroseconds(comandoGasFinale_us);
    } else {
        gestisciAllarmi();   // Aggiorna i LED di stato in base ad allarmi/GPS/modalità
        applicaMixer4Servi(correzionePitch_deg, correzioneRoll_deg);

        GAS_LIMITE_TERMICO_us = gasMaxTermico();
        // FIX (obiettivo 2): la limitazione termica del gas non veniva mai segnalata a terra
        // quando entrava/usciva in funzione. Edge detection sullo stato precedente per notificare
        // solo il cambiamento, non ogni ciclo.
        static bool limitazioneTermicaPrecedente = false;
        if (comandoGasFinale_us > GAS_LIMITE_TERMICO_us && limitazione_termica_gas) {
            comandoGasFinale_us = GAS_LIMITE_TERMICO_us;   // Applica il taglio termico se abilitato
            limitazioneTermicaAttiva = true;
        } else {
            limitazioneTermicaAttiva = false;
        }
        if (limitazioneTermicaAttiva != limitazioneTermicaPrecedente) {
            inviaMessaggioAvionica(limitazioneTermicaAttiva ? "Limitazione termica gas ATTIVA (motore/ESC in surriscaldamento)"
                                                              : "Limitazione termica gas rientrata");
            limitazioneTermicaPrecedente = limitazioneTermicaAttiva;
        }
        motore.writeMicroseconds(comandoGasFinale_us);
    }

    inviaTelemetria(
        pitch_deg, roll_deg, yaw_deg,
        VELOCITA_ARIA_ms * 3.6f,
        VELOCITA_SUOLO_ms * 3.6f,
        correzionePitch_deg, correzioneRoll_deg, comandoGasFinale_us);
}
