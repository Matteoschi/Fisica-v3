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
#include "pin.h"

// ============================================================
// SENSORI E OGGETTI GLOBALI
// ============================================================
TinyGPSPlus       gps;
Adafruit_BNO055   giroscopio = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX   barometro;
Bitcraze_PMW3901  flussoOttico(PIN_PMW3901_CS);
SBUS              ricevente(SBUS_SERIAL);

Adafruit_INA219 sensoreMotore(0x40);
Adafruit_INA219 sensoreIntSX(0x41);
Adafruit_INA219 sensoreIntDX(0x42);
Adafruit_INA219 sensoreEstSX(0x43);
Adafruit_INA219 sensoreEstDX(0x44);
Adafruit_INA219 sensoreTeensy(0x45);

Servo servoInternoSX;
Servo servoInternoDX;
Servo servoEsternoSX;
Servo servoEsternoDX;
Servo motore;

// ============================================================
// STATO SENSORI
// B_*_INIZIALIZZATO = trovato/configurato.
// B_*_OK            = affidabile in questo momento.
// ============================================================
bool B_BNO055_INIZIALIZZATO = false;
bool B_PMW3901_INIZIALIZZATO = false;
bool B_LIDAR_INIZIALIZZATO = false;
bool B_BMP390_INIZIALIZZATO = false;
bool B_PITOT_INIZIALIZZATO = false;
bool B_INA219_INIZIALIZZATO = false;

bool B_BNO055_OK = false;
bool B_PMW3901_OK = false;
bool B_LIDAR_OK = false;
bool B_BMP390_OK = false;
bool B_PITOT_OK = false;
bool B_INA219_OK = false;
bool B_INA219_MOTORE_OK = false;
bool B_INA219_TEENSY_OK = false;
bool B_INA219_INT_SX_OK = false;
bool B_INA219_INT_DX_OK = false;
bool B_INA219_EST_SX_OK = false;
bool B_INA219_EST_DX_OK = false;
bool B_GPS_OK = false;

unsigned long G_ultimo_lidar_valido_ms = 0;
int G_tentativi_init = 0;

float G_densita_aria_kgm3     = 1.225f;
float G_pitot_zero_adc        = 0.0f;
bool  B_pitot_disponibile          = false;
float G_velocita_aria_ms      = 0.0f;

float G_velocita_suolo_ms          = 0.0f;
float G_velocita_ottica_x_ms       = 0.0f;
float G_velocita_ottica_y_ms       = 0.0f;
float G_velocita_suolo_gps_ms   = 0.0f;
int G_errore_gps = -2;

float G_altitudine_lidar_m      = -1.0f;
float G_altitudine_baro_m       = 0.0f;
float G_altitudine_m            = 0.0f;
float G_tara_altitudine_baro_m  = 0.0f;
float G_pressione_baro_pa       = 0.0f;

// Vertical speed / sink rate (punto 12)
float G_velocita_verticale_ms  = 0.0f;   // positivo = salita, negativo = discesa
bool  B_sink_rate_eccessivo    = false;

float G_vento_velocita_ms   = 0.0f;
float G_vento_direzione_deg = 0.0f;

float G_temperatura_motore_c = 0.0f;
float G_temperatura_fusoliera_c   = 0.0f;   
float G_temperatura_esc_c=0.0f;
float G_temperatura_esterna_c = 0.0f;

double G_target_lat_deg       = TARGET_LAT_DEFAULT_deg;
double G_target_lon_deg       = TARGET_LON_DEFAULT_deg;
double G_drone_lat_deg       = 41.902782;
double G_drone_lon_deg       = 12.496366;
int G_numero_satelliti=   0;
float  G_altitudine_target_m  = ALTITUDINE_TARGET_DEFAULT_m;

float G_roll_target_deg    = 0.0f;
float G_rotta_attuale_deg = 0.0f;
float G_distanza_target_m  = 0.0f;
float G_rotta_target_deg   = 0.0f;
float G_errore_rotta_deg   = 0.0f;

float G_offset_roll_deg  = 0.0f;
float G_offset_pitch_deg = 0.0f;
float G_offset_yaw_deg   = 0.0f;

uint8_t G_imu_cal_sys = 0, G_imu_cal_gyro = 0, G_imu_cal_accel = 0, G_imu_cal_mag = 0;

bool B_alimentazione_sicurezza = true;   
bool B_batteria_bassa_motore    = false;
bool B_batteria_bassa_teensy    = false;
bool B_rele_attivato           = false;

unsigned long G_tempo_batteria_precedente_ms = 0;

float G_carica_consumata_teensy = 0.0;
float G_carica_consumata_motore = 0.0;
float G_carica_rimanente_teensy_percentuale=0.0;
float G_carica_rimanente_motore_percentuale=0.0;
float G_autonomia_teensy_residua = 0.0;
float G_autonomia_motore_residua=0.0;

float G_corrente_teensy_ma = 0.0;
float G_corrente_motore_ma =0.0;
float G_tensione_teensy_v=0.0;
float G_tensione_motore_v = 0.0;

// Corrente motore (punto 13)
bool B_corrente_motore_eccessiva = false;


float G_tensione_servo_int_sx_v = 0.0f;
float G_tensione_servo_int_dx_v = 0.0f;
float G_tensione_servo_est_sx_v = 0.0f;
float G_tensione_servo_est_dx_v = 0.0f;

int  G_gas_limite_termico_us       = GAS_MASSIMO_us;
bool B_limitazione_termica_attiva    = false;
bool B_limitazione_termica_abilitata        = true;   
bool B_motore_disabilitato_da_terra   = false;

bool B_servo_sicurezza          = true;
// Stato di attach/detach per singolo servo (punto 7): permette di lasciare
// attaccato un servo ancora funzionante anche se il "compagno" di coppia e'
// guasto, invece di staccare l'intera coppia.
bool B_SERVO_EST_SX_OK = true, B_SERVO_EST_DX_OK = true;
bool B_SERVO_INT_SX_OK = true, B_SERVO_INT_DX_OK = true;

bool B_schianto_sicurezza      = true;  
bool B_stato_schianto_rilevato  = false;
bool B_schianto_bloccato       = false;
bool B_drone_in_volo            = false;
unsigned long TIMESTAMP_DECOLLO_ms = 0;

int G_modalita_volo = 1;

// ============================================================
// STATO CORRENTE DEL CONTROLLO
// Queste variabili sono globali solo perche' servono a piu' funzioni.
// Tutti i calcoli temporanei restano invece locali dentro le funzioni.
// In questo modo loop(), telemetria e attuatori non devono passarsi
// continuamente gli stessi 6-7 parametri.
// ============================================================
float G_pitch_deg = 0.0f;
float G_roll_deg  = 0.0f;
float G_yaw_deg   = 0.0f;

float G_target_velocita_kmh = 0.0f;
int   G_gas_base_us = GAS_NEUTRO_us;

int G_comando_pitch_deg = 0;
int G_comando_roll_deg  = 0;
int G_comando_gas_us    = GAS_NEUTRO_us;

// Codici semplici usati da verificaProtezioniVolo().
// Non sono soglie modificabili: servono solo per rendere leggibile il codice.
const int PROTEZIONE_NESSUNA       = 0;
const int PROTEZIONE_STALLO        = 1;
const int PROTEZIONE_OVERSPEED     = 2;
const int PROTEZIONE_QUOTA_MASSIMA = 3;
const int PROTEZIONE_QUOTA_MINIMA  = 4;

float G_velocita_crociera_kmh      = VELOCITA_CROCIERA_DEFAULT_kmh;
float G_velocita_avvicinamento_kmh = VELOCITA_AVVICINAMENTO_DEFAULT_kmh;


unsigned long G_tempo_pid_precedente_ms = 0;

float G_pid_alt_integrale = 0.0f,   G_pid_alt_errore_precedente_m   = 0.0f;
float G_pid_pitch_integrale = 0.0f, G_pid_pitch_errore_precedente_deg = 0.0f;
float G_pid_roll_integrale = 0.0f,  G_pid_roll_errore_precedente_deg  = 0.0f;
float G_pid_vel_integrale = 0.0f,   G_pid_vel_errore_precedente_kmh   = 0.0f;


uint16_t G_canali_rc[16];
bool B_failsafe      = false;

bool B_forza_invio_diagnostica             = false;

// ============================================================
// PROTOTIPI - organizzati per responsabilita'
// ============================================================

// Segnalazioni / comunicazione
void segnalaOK();
void segnalaErrore();
void segnalaCalibrazione(int pin_led);
void inviaMessaggioAvionica(const char* messaggio);
void inviaMessaggioAvionica(const String& messaggio);

// Setup
void setupSegnalazioni();
void setupComunicazioni();
void setupSensori();
void setupAttuatori();
void finalizzaSetup();
bool sensoriCriticiOK();
void bloccaAvvioSeSensoriCriticiKO();

// Inizializzazione sensori
bool inizializzaFlussoOttico();
bool inizializzaLidar();
bool inizializzaGPS();
bool inizializzaIMU();
bool inizializzaBarometro();
bool inizializzaPitot();
bool inizializzaINA219();

// Calibrazione richiamabile
bool calibraIMU();
bool calibraBarometro();
bool calibraPitot();
bool calibraDopoSchianto();
bool calibrazioneConsentita();

// Lettura / stato sensori
bool gpsValido();
void leggiIMU();
void leggiPitot();
void leggiBarometro();
void leggiTemperatura();
void aggiornaLidar();
void leggiVelocitaOttica();
void aggiornaGPS();
void aggiornaDiagnosticaIMU();
void aggiornaStatoSensori();
void inviaDiagnosticaSensori();
bool letturaINAValida(float tensione_V);

// Elaborazione dati
void aggiornaDensitaAria();
void selezionaAltitudine();
void aggiornaVelocitaVerticale();
void aggiornaVelocitaSuolo();
void stimaVento();
void aggiornaNavigazione();

// Controllo
float aggiornaTempoPID();
void calcolaPIDAssetto(float targetPitch_deg, float targetRoll_deg, float dt_s);
void calcolaPID();
int verificaProtezioniVolo();
void calcolaComandiProtezione(int protezione);
int gasMaxTermico();
void resettaPID();
void scegliTargetVelocita();
void aggiornaModalitaVoloDaRadio();
void calcolaComandiVolo();

// Attuatori / sicurezza
void inizializzaServo();
void inizializzaMotore();
void scriviMotore(int gas_us);
void applicaMixer4Servi();
void diagnosticaServi();
void gestisciAlimentazione();
void gestisciSchianto();
void verificaDroneInVolo();
void gestisciAllarmi();
void aggiornaAttuatori();

// Loop ad alto livello
void gestisciInizioCiclo();
void aggiornaSensori();

// Telemetria / comandi
void inviaTelemetria();
void comandiDaTerra();
void elaboraComando(const String& cmd);


void segnalaOK() {
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    tone(PIN_BUZZER, 1200, 150);
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
}
void segnalaErrore() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        tone(PIN_BUZZER, 400, 100);
        delay(150);
        digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        delay(100);
    }
}
void segnalaCalibrazione(int pin_led) {
    digitalWrite(pin_led, !digitalRead(pin_led));
    tone(PIN_BUZZER, 1000, 30);
}
void inviaMessaggioAvionica(const char* messaggio) {
    Serial.print("[AVIONICA] da banco: ");
    Serial.println(messaggio);
    TELEMETRIA.print("MSG,");
    TELEMETRIA.println(messaggio);
}

void inviaMessaggioAvionica(const String& messaggio) {
    inviaMessaggioAvionica(messaggio.c_str());
}


// Restituisce true quando il GPS possiede un fix utilizzabile.
// Tutto il codice deve usare questa funzione invece di ripetere
// continuamente "G_errore_gps > 0".
bool gpsValido() {
    return G_errore_gps > 0;
}

void aggiornaGPS() {

    if (gps.charsProcessed() < 10) {
        G_errore_gps = 0;
    } 
    else if (!gps.location.isValid()) {
        G_errore_gps = -1;
    } 
    else if (gps.location.age() > TIMEOUT_GPS_ms) {
        G_errore_gps = -6;
    } 
    else if (!gps.satellites.isValid() || gps.satellites.value() == 0) {
        G_errore_gps = -3;
    } 
    else if (!gps.course.isValid()) {
        G_errore_gps = -2;
    } 
    else if (!gps.speed.isValid()) {
        G_errore_gps = -5;
    } 
    else {
        float hdop_attuale = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9f;

        if (hdop_attuale < 1.5f) {
            G_errore_gps = 3;
        } 
        else if (hdop_attuale < 2.0f) {
            G_errore_gps = 2;
        } 
        else {
            G_errore_gps = 1;
        }
    }

    static int erroreGpsPrecedente = -99;
    if (G_errore_gps != erroreGpsPrecedente) {
        if (G_errore_gps == 0)      inviaMessaggioAvionica("GPS: 0 -> scollegato");
        else if (G_errore_gps == -1) inviaMessaggioAvionica("GPS: -1 -> nessun fix");
        else if (G_errore_gps == -2) inviaMessaggioAvionica("GPS: -2 -> ROTTA NON CALCOLABILE");
        else if (G_errore_gps == -3) inviaMessaggioAvionica("GPS: -3 -> zero satelliti validi");
        else if (G_errore_gps == -5) inviaMessaggioAvionica("GPS: -5 -> velocita NON calcolabile");
        else if (G_errore_gps == -6) inviaMessaggioAvionica("GPS: -6 -> dati congelati timeout > 1,5s");
        else if (G_errore_gps == 1)  inviaMessaggioAvionica("GPS: 1 -> fix scarso");
        else if (G_errore_gps == 2)  inviaMessaggioAvionica("GPS: 2 -> fix buono");
        else if (G_errore_gps == 3)  inviaMessaggioAvionica("GPS: 3 -> fix eccellente");
        erroreGpsPrecedente = G_errore_gps;
    }

    static bool b_fix_valido_precedente = false;
    bool b_fix_valido_adesso = gpsValido();
    if (b_fix_valido_adesso != b_fix_valido_precedente) {
        if (b_fix_valido_adesso) {
            inviaMessaggioAvionica("GPS: fix riacquisito, navigazione automatica affidabile");
        } else if (B_drone_in_volo) {
            inviaMessaggioAvionica("ATTENZIONE: GPS fix perso in volo!");
        }
        b_fix_valido_precedente = b_fix_valido_adesso;
    }

    // Controlli aggiuntivi sul GPS:
    // 1. rileviamo salti di posizione troppo grandi tra due letture;
    // 2. rileviamo una posizione che rimane identica per troppo tempo.
    static double latPrecedente_deg = 0.0, lonPrecedente_deg = 0.0;
    static bool   b_precedente_disponibile = false;
    static int    cicliPosizioneUguale = 0;
    static bool   b_gps_salto_precedente = false;
    static bool   b_gps_congelato_precedente = false;

    if (gpsValido()) {
        G_drone_lat_deg       = gps.location.lat();
        G_drone_lon_deg       = gps.location.lng();
        G_velocita_suolo_gps_ms = gps.speed.mps();
        G_rotta_attuale_deg    = gps.course.deg();
        G_numero_satelliti    = gps.satellites.value();

        G_distanza_target_m = TinyGPSPlus::distanceBetween(
            G_drone_lat_deg, G_drone_lon_deg, G_target_lat_deg, G_target_lon_deg
        );
        G_rotta_target_deg = TinyGPSPlus::courseTo(
            G_drone_lat_deg, G_drone_lon_deg, G_target_lat_deg, G_target_lon_deg
        );

        if (b_precedente_disponibile) {
            float saltoDistanza_m = TinyGPSPlus::distanceBetween(
                latPrecedente_deg, lonPrecedente_deg, G_drone_lat_deg, G_drone_lon_deg
            );
            bool b_gps_salto = (saltoDistanza_m > SALTO_GPS_MAX_m);
            if (b_gps_salto != b_gps_salto_precedente) {
                if (b_gps_salto) inviaMessaggioAvionica("ATTENZIONE: salto GPS irrealistico tra due letture consecutive");
                b_gps_salto_precedente = b_gps_salto;
            }

            bool b_posizione_uguale = (fabs(G_drone_lat_deg - latPrecedente_deg) < 1e-7 &&
                                        fabs(G_drone_lon_deg - lonPrecedente_deg) < 1e-7);
            cicliPosizioneUguale = b_posizione_uguale ? (cicliPosizioneUguale + 1) : 0;

            bool b_gps_congelato = (cicliPosizioneUguale > CICLI_GPS_CONGELATO_MAX);
            if (b_gps_congelato != b_gps_congelato_precedente) {
                if (b_gps_congelato) inviaMessaggioAvionica("ATTENZIONE: posizione GPS apparentemente congelata nonostante fix valido");
                b_gps_congelato_precedente = b_gps_congelato;
            }
        }
        latPrecedente_deg = G_drone_lat_deg;
        lonPrecedente_deg = G_drone_lon_deg;
        b_precedente_disponibile = true;
    } else {
        G_drone_lat_deg       = gps.location.isValid() ? gps.location.lat() : -1.0f;
        G_drone_lon_deg       = gps.location.isValid() ? gps.location.lng() : -1.0f;
        G_velocita_suolo_gps_ms = gps.speed.isValid()    ? gps.speed.mps()    : -1.0f;
        G_rotta_attuale_deg    = gps.course.isValid()   ? gps.course.deg()   : -1.0f;
        G_numero_satelliti    = gps.satellites.isValid() ? gps.satellites.value() : 0;
        
        G_distanza_target_m   = -1.0f;
        G_rotta_target_deg    = -1.0f;
    }

    // B_GPS_OK rappresenta lo stato del GPS ADESSO, non soltanto al boot.
    B_GPS_OK = gpsValido();
}


// Stima il vento confrontando il vettore airspeed con il vettore ground speed.
// Richiede Pitot valido e GPS di qualita' eccellente (G_errore_gps == 3).
void stimaVento() {
    // Per questa stima non basta un fix GPS qualsiasi:
    // manteniamo il requisito originale G_errore_gps > 2.
    bool b_stima_disponibile =
        B_pitot_disponibile &&
        G_errore_gps > 2;

    if (!b_stima_disponibile) {
        G_vento_velocita_ms = -1.0f;
        G_vento_direzione_deg = -1.0f;
        return;
    }

    // Componenti della velocita' dell'aria.
    float velocitaAriaX_ms =
        G_velocita_aria_ms * cos(radians(G_yaw_deg));

    float velocitaAriaY_ms =
        G_velocita_aria_ms * sin(radians(G_yaw_deg));

    // Componenti della velocita' rispetto al terreno misurata dal GPS.
    float velocitaSuoloX_ms =
        G_velocita_suolo_ms * cos(radians(G_rotta_attuale_deg));

    float velocitaSuoloY_ms =
        G_velocita_suolo_ms * sin(radians(G_rotta_attuale_deg));

    // Vento = velocita' suolo - velocita' aria.
    float ventoX_ms = velocitaSuoloX_ms - velocitaAriaX_ms;
    float ventoY_ms = velocitaSuoloY_ms - velocitaAriaY_ms;

    // Modulo del vento.
    G_vento_velocita_ms =
        sqrtf(ventoX_ms * ventoX_ms + ventoY_ms * ventoY_ms);

    // Direzione del vento.
    G_vento_direzione_deg =
        degrees(atan2(ventoY_ms, ventoX_ms));

    // Portiamo anche la direzione del vento nel range 0...360.
    if (G_vento_direzione_deg < 0.0f) {
        G_vento_direzione_deg += 360.0f;
    }
}

// Legge l'assetto dall'IMU.
// Se la lettura e' valida aggiorna G_pitch_deg, G_roll_deg e G_yaw_deg.
// Se la lettura e' sbagliata lascia gli ultimi valori validi: in questo modo
// un NaN non arriva mai direttamente al controllo di volo.
void leggiIMU() {
    // Chiediamo al BNO055 l'orientamento corrente.
    sensors_event_t event;
    giroscopio.getEvent(&event);

    // Correggiamo pitch e roll con la tara fatta durante la calibrazione.
    float nuovoPitch_deg = event.orientation.y - G_offset_pitch_deg;
    float nuovoRoll_deg  = event.orientation.z - G_offset_roll_deg;

    // Lo yaw viene letto direttamente come direzione 0...360 gradi.
    float nuovoYaw_deg = event.orientation.x;

    // Normalizziamo lo yaw nell'intervallo 0...360.
    if (nuovoYaw_deg < 0.0f) nuovoYaw_deg += 360.0f;
    if (nuovoYaw_deg >= 360.0f) nuovoYaw_deg -= 360.0f;

    // Leggiamo lo stato interno dell'IMU.
    uint8_t statoSistema = 0;
    uint8_t selfTest = 0;
    uint8_t erroreSistema = 0;
    giroscopio.getSystemStatus(&statoSistema, &selfTest, &erroreSistema);

    // Tutti e tre gli angoli devono essere numeri reali.
    bool b_valori_finiti =
        isfinite(nuovoPitch_deg) &&
        isfinite(nuovoRoll_deg) &&
        isfinite(nuovoYaw_deg);

    // Controllo molto semplice contro valori impossibili.
    bool b_valori_plausibili =
        fabs(nuovoPitch_deg) <= 180.0f &&
        fabs(nuovoRoll_deg) <= 180.0f &&
        nuovoYaw_deg >= 0.0f &&
        nuovoYaw_deg < 360.0f;

    // Il sensore e' OK solo se era stato inizializzato e la lettura e' valida.
    B_BNO055_OK =
        B_BNO055_INIZIALIZZATO &&
        b_valori_finiti &&
        b_valori_plausibili &&
        erroreSistema == 0;

    // Aggiorniamo lo stato usato dal resto del drone soltanto con dati validi.
    if (B_BNO055_OK) {
        G_pitch_deg = nuovoPitch_deg;
        G_roll_deg  = nuovoRoll_deg;
        G_yaw_deg   = nuovoYaw_deg;
    }
}


// Legge il Pitot e calcola la velocita' dell'aria.
// Il valore ADC rimane locale: al resto del programma interessa solo
// se il Pitot e' affidabile e quale airspeed ha misurato.
void leggiPitot() {
    // Leggiamo l'ingresso analogico.
    int lettura_adc = constrain(analogRead(PIN_ARIA), 0, 1023);

    // Un ADC troppo vicino ai bordi puo' indicare un problema elettrico.
    bool b_lettura_elettrica_valida =
        lettura_adc > PITOT_ADC_MIN_VALIDO &&
        lettura_adc < PITOT_ADC_MAX_VALIDO;

    // Stato reale del sensore in questo ciclo.
    B_PITOT_OK = B_PITOT_INIZIALIZZATO && b_lettura_elettrica_valida;

    // Differenza rispetto allo zero misurato in calibrazione.
    float differenza_adc = (float)lettura_adc - G_pitot_zero_adc;

    // Il dato e' utilizzabile solo se il sensore e' OK e la pressione dinamica e' positiva.
    B_pitot_disponibile =
        B_PITOT_OK &&
        differenza_adc > 0.0f &&
        G_densita_aria_kgm3 > 0.0f;

    // Se il dato e' valido calcoliamo l'airspeed.
    if (B_pitot_disponibile) {
        G_velocita_aria_ms =
            sqrtf((2.0f * differenza_adc * FATTORE_CONVERSIONE_PITOT_Pa) /
                  G_densita_aria_kgm3);
    } else {
        // -1 significa: velocita' aria non disponibile.
        G_velocita_aria_ms = -1.0f;
    }
}

void leggiBarometro() {
    float altitudine_m = barometro.readAltitude(PRESSIONE_RIFERIMENTO_BARO_hPa);
    float pressione_pa = barometro.pressure;
    float temperatura_c = barometro.temperature;

    bool b_valori_validi = isfinite(altitudine_m) &&
                           isfinite(pressione_pa) &&
                           isfinite(temperatura_c) &&
                           altitudine_m >= BARO_ALTITUDINE_MIN_PLAUSIBILE_m &&
                           altitudine_m <= BARO_ALTITUDINE_MAX_PLAUSIBILE_m &&
                           pressione_pa >= BARO_PRESSIONE_MIN_PLAUSIBILE_Pa &&
                           pressione_pa <= BARO_PRESSIONE_MAX_PLAUSIBILE_Pa &&
                           temperatura_c >= BARO_TEMPERATURA_MIN_PLAUSIBILE_C &&
                           temperatura_c <= BARO_TEMPERATURA_MAX_PLAUSIBILE_C;

    B_BMP390_OK = B_BMP390_INIZIALIZZATO && b_valori_validi;

    if (B_BMP390_OK) {
        G_altitudine_baro_m = altitudine_m - G_tara_altitudine_baro_m;
        G_pressione_baro_pa = pressione_pa;
        G_temperatura_fusoliera_c = temperatura_c;
    }
}

void leggiTemperatura() {
    float voltaggioSensore_motore_V = analogRead(PIN_TEMP_MOTORE) * (3.3f / 1023.0f);
    G_temperatura_motore_c = (voltaggioSensore_motore_V - 0.5f) * 100.0f;

    float voltaggioSensore_esc_V = analogRead(PIN_TEMP_ESC) * (3.3f / 1023.0f);
    G_temperatura_esc_c = (voltaggioSensore_esc_V - 0.5f) * 100.0f;

    float voltaggioSensore_esterno_V = analogRead(PIN_TEMP_EST) * (3.3f / 1023.0f);
    G_temperatura_esterna_c = (voltaggioSensore_esterno_V - 0.5f) * 100.0f;
}

void aggiornaLidar() {
    static uint8_t buffer[9];

    bool b_quota_operativa = !B_BMP390_OK ||
                             G_altitudine_baro_m <= ALTITUDINE_MAX_LIDAR_m;
    bool b_frame_valido = false;

    while (LIDAR_SERIAL.available() >= 9) {
        if (LIDAR_SERIAL.read() == 0x59 && LIDAR_SERIAL.peek() == 0x59) {
            LIDAR_SERIAL.read();
            buffer[0] = 0x59;
            buffer[1] = 0x59;

            for (int i = 2; i < 9; i++) {
                buffer[i] = LIDAR_SERIAL.read();
            }

            uint8_t checksum = 0;
            for (int i = 0; i < 8; i++) checksum += buffer[i];

            if (checksum != buffer[8]) continue;

            uint16_t distanza_cm = buffer[2] | ((uint16_t)buffer[3] << 8);
            float distanza_m = distanza_cm / 100.0f;

            if (!isfinite(distanza_m) || distanza_m <= 0.0f) continue;

            B_LIDAR_INIZIALIZZATO = true;
            B_LIDAR_OK = true;
            G_ultimo_lidar_valido_ms = millis();
            b_frame_valido = true;

            if (b_quota_operativa) {
                if (G_altitudine_lidar_m < 0.0f) {
                    G_altitudine_lidar_m = distanza_m;
                } else {
                    G_altitudine_lidar_m = ALPHA_LIDAR * distanza_m +
                                           (1.0f - ALPHA_LIDAR) * G_altitudine_lidar_m;
                }
            } else {
                G_altitudine_lidar_m = -1.0f;
            }

            break;
        }
    }

    if (!b_frame_valido &&
        B_LIDAR_INIZIALIZZATO &&
        (millis() - G_ultimo_lidar_valido_ms > TIMEOUT_LIDAR_DATI_ms)) {
        B_LIDAR_OK = false;
        G_altitudine_lidar_m = -1.0f;
    }

    if (!b_quota_operativa) {
        G_altitudine_lidar_m = -1.0f;
    }
}


// Legge il PMW3901 e trasforma i conteggi in velocita' al suolo.
// Usa direttamente G_yaw_deg per ruotare la misura nel riferimento del mondo.
void leggiVelocitaOttica() {
    // Tempo dell'ultima lettura, mantenuto tra una chiamata e la successiva.
    static unsigned long tempoPrecedente_ms = 0;

    // Conteggi grezzi del PMW3901: servono solo qui, quindi sono locali.
    int16_t dx = 0;
    int16_t dy = 0;
    flussoOttico.readMotionCount(&dx, &dy);

    // Calcoliamo il tempo trascorso tra due misure.
    unsigned long adesso_ms = millis();
    float dt_s = (adesso_ms - tempoPrecedente_ms) / 1000.0f;
    tempoPrecedente_ms = adesso_ms;

    // La libreria non restituisce un vero errore ad ogni lettura.
    // Per questo B_PMW3901_OK indica soprattutto che il sensore e' stato inizializzato.
    B_PMW3901_OK = B_PMW3901_INIZIALIZZATO;

    // Sopra la quota utile del sensore il dato non viene usato.
    if (!B_PMW3901_OK || dt_s <= 0.0f || G_altitudine_m > ALTITUDINE_MAX_OTTICO_m) {
        G_velocita_ottica_x_ms = -1.0f;
        G_velocita_ottica_y_ms = -1.0f;
        return;
    }

    // Convertiamo i conteggi in metri al secondo.
    float velocitaXDrone_ms =
        (dx * COSTANTE_CALIBRAZIONE_OTTICA * G_altitudine_m) / dt_s;

    float velocitaYDrone_ms =
        (dy * COSTANTE_CALIBRAZIONE_OTTICA * G_altitudine_m) / dt_s;

    // Convertiamo lo yaw da gradi a radianti.
    float yaw_rad = radians(G_yaw_deg);

    // Ruotiamo le velocita' dal riferimento del drone al riferimento globale.
    G_velocita_ottica_x_ms =
        velocitaXDrone_ms * cos(yaw_rad) -
        velocitaYDrone_ms * sin(yaw_rad);

    G_velocita_ottica_y_ms =
        velocitaXDrone_ms * sin(yaw_rad) +
        velocitaYDrone_ms * cos(yaw_rad);
}


// Calcola la densita' dell'aria usando pressione e temperatura del barometro.
// Non servono parametri perche' entrambe le misure sono gia' nello stato globale.
void aggiornaDensitaAria() {
    // Convertiamo i gradi Celsius in Kelvin.
    float temperatura_K = G_temperatura_fusoliera_c + 273.15f;

    // Usiamo la legge dei gas perfetti solo con valori sensati.
    if (temperatura_K > 0.0f && G_pressione_baro_pa > 0.0f) {
        G_densita_aria_kgm3 =
            G_pressione_baro_pa / (R_SPECIFIC_ARIA * temperatura_K);
    } else {
        // Valore standard a livello del mare come fallback.
        G_densita_aria_kgm3 = 1.225f;
    }
}

// Sceglie la sorgente di quota piu' affidabile tra barometro e LIDAR.
// Le soglie di blend sono definite una sola volta in config.h.
void selezionaAltitudine() {
    static float offsetBaro_m = 0.0f;
    static float altitudinePrecedente_m = 0.0f;
    static bool b_prima_esecuzione = true;

    bool b_lidar_disponibile = B_LIDAR_OK && G_altitudine_lidar_m > 0.0f;
    bool b_baro_disponibile = B_BMP390_OK;

    if (!b_lidar_disponibile && !b_baro_disponibile) {
        // Nessuna sorgente affidabile: manteniamo l'ultima quota valida.
        return;
    }

    if (!b_baro_disponibile && b_lidar_disponibile) {
        G_altitudine_m = G_altitudine_lidar_m;
        altitudinePrecedente_m = G_altitudine_m;
        b_prima_esecuzione = false;
        return;
    }

    if (b_lidar_disponibile) {
        float baroCorrettoPreliminare_m = G_altitudine_baro_m - offsetBaro_m;
        if (fabs(baroCorrettoPreliminare_m - G_altitudine_lidar_m) > SOGLIA_DISCORDANZA_QUOTA_m) {
            b_lidar_disponibile = false;
        }
    }

    if (b_lidar_disponibile && G_altitudine_lidar_m < ZONA_BLEND_LIDAR_START_m) {
        offsetBaro_m = G_altitudine_baro_m - G_altitudine_lidar_m;
    }

    float baroCorretto_m = G_altitudine_baro_m - offsetBaro_m;
    float altitudineCandidata_m = baroCorretto_m;

    if (baroCorretto_m < ZONA_BLEND_LIDAR_START_m) {
        if (b_lidar_disponibile) altitudineCandidata_m = G_altitudine_lidar_m;
    } else if (baroCorretto_m < ZONA_BLEND_LIDAR_END_m && b_lidar_disponibile) {
        float pesoBaro = (baroCorretto_m - ZONA_BLEND_LIDAR_START_m) /
                         (ZONA_BLEND_LIDAR_END_m - ZONA_BLEND_LIDAR_START_m);
        pesoBaro = constrain(pesoBaro, 0.0f, 1.0f);

        altitudineCandidata_m = (G_altitudine_lidar_m * (1.0f - pesoBaro)) +
                                (baroCorretto_m * pesoBaro);
    }

    if (b_prima_esecuzione) {
        G_altitudine_m = altitudineCandidata_m;
        b_prima_esecuzione = false;
    } else {
        float variazione_m = altitudineCandidata_m - altitudinePrecedente_m;
        variazione_m = constrain(variazione_m,
                                 -MAX_VARIAZIONE_ALTITUDINE_PER_CICLO_m,
                                  MAX_VARIAZIONE_ALTITUDINE_PER_CICLO_m);
        G_altitudine_m = altitudinePrecedente_m + variazione_m;
    }

    altitudinePrecedente_m = G_altitudine_m;
}

// Calcola il rateo verticale dalla variazione di quota nel tempo.
// Valore positivo = salita, valore negativo = discesa.
void aggiornaVelocitaVerticale() {
    static float altitudinePrecedente_m = 0.0f;
    static unsigned long tempoPrecedente_ms = 0;
    static bool b_prima_esecuzione = true;

    unsigned long tempoAttuale_ms = millis();
    float dt_s = (tempoAttuale_ms - tempoPrecedente_ms) / 1000.0f;
    tempoPrecedente_ms = tempoAttuale_ms;

    if (!b_prima_esecuzione && dt_s > 0.0f && dt_s < 2.0f) {
        G_velocita_verticale_ms = (G_altitudine_m - altitudinePrecedente_m) / dt_s;
    }
    altitudinePrecedente_m = G_altitudine_m;
    b_prima_esecuzione = false;

    static bool b_sink_rate_precedente = false;
    B_sink_rate_eccessivo = (G_velocita_verticale_ms < SOGLIA_SINK_RATE_ms);
    if (B_sink_rate_eccessivo != b_sink_rate_precedente) {
        inviaMessaggioAvionica(B_sink_rate_eccessivo ? "ATTENZIONE: sink rate anomalo rilevato"
                                                      : "Sink rate rientrato nei limiti normali");
        b_sink_rate_precedente = B_sink_rate_eccessivo;
    }
}


// Sceglie la migliore velocita' al suolo disponibile.
// A bassa quota preferisce il flusso ottico, ad alta quota il GPS.
// Nella zona intermedia fa un blend graduale tra i due.
void aggiornaVelocitaSuolo() {
    // -1 significa che il modulo della velocita' ottica non e' disponibile.
    float velocitaOttica_ms = -1.0f;

    // Il flusso ottico e' utilizzabile solo se entrambe le componenti sono valide.
    bool b_ottico_disponibile =
        B_PMW3901_OK &&
        G_velocita_ottica_x_ms != -1.0f &&
        G_velocita_ottica_y_ms != -1.0f;

    // Calcoliamo il modulo della velocita' ottica.
    if (b_ottico_disponibile) {
        velocitaOttica_ms =
            sqrtf(G_velocita_ottica_x_ms * G_velocita_ottica_x_ms +
                  G_velocita_ottica_y_ms * G_velocita_ottica_y_ms);
    }

    // Per il GPS usiamo sempre la funzione unica gpsValido().
    bool b_gps_disponibile = B_GPS_OK && gpsValido();

    // A bassa quota preferiamo il sensore ottico.
    if (G_altitudine_m < ZONA_BLEND_OTTICO_START_m) {
        if (b_ottico_disponibile) {
            G_velocita_suolo_ms = velocitaOttica_ms;
        } else if (b_gps_disponibile) {
            G_velocita_suolo_ms = G_velocita_suolo_gps_ms;
        } else {
            G_velocita_suolo_ms = -1.0f;
        }
        return;
    }

    // Sopra il limite ottico usiamo soltanto il GPS.
    if (G_altitudine_m >= ALTITUDINE_MAX_OTTICO_m) {
        G_velocita_suolo_ms =
            b_gps_disponibile ? G_velocita_suolo_gps_ms : -1.0f;
        return;
    }

    // Siamo nella zona di transizione.
    if (b_ottico_disponibile && b_gps_disponibile) {
        // 0 = tutto ottico, 1 = tutto GPS.
        float pesoGPS =
            (G_altitudine_m - ZONA_BLEND_OTTICO_START_m) /
            (ALTITUDINE_MAX_OTTICO_m - ZONA_BLEND_OTTICO_START_m);

        pesoGPS = constrain(pesoGPS, 0.0f, 1.0f);

        G_velocita_suolo_ms =
            velocitaOttica_ms * (1.0f - pesoGPS) +
            G_velocita_suolo_gps_ms * pesoGPS;
    } else if (b_ottico_disponibile) {
        G_velocita_suolo_ms = velocitaOttica_ms;
    } else if (b_gps_disponibile) {
        G_velocita_suolo_ms = G_velocita_suolo_gps_ms;
    } else {
        G_velocita_suolo_ms = -1.0f;
    }
}

// Aggiorna la navigazione verso il waypoint usando i dati GPS gia' globali.
// Se la rotta GPS non e' affidabile a bassa velocita', usa lo yaw dell'IMU.
void aggiornaNavigazione() {
    static unsigned long ultimoGpsValido_ms = 0;
    static bool b_waypoint_raggiunto = false;
    static bool b_rotta_con_yaw_precedente = false;
    static bool b_correzione_vento_precedente = false;
    static bool b_gps_timeout_precedente = false;
    unsigned long tempoAttuale_ms = millis();
    

    if (gpsValido()) {
        ultimoGpsValido_ms = tempoAttuale_ms;
        b_gps_timeout_precedente = false;

        float velocitaPerCalcolo_ms = max(G_velocita_suolo_ms, 1.0f);
        float L1_m = max(velocitaPerCalcolo_ms * 4.0f, 1.0f);
        float raggioAccettazioneDinamico_m = max(RAGGIO_ACCETTAZIONE_MINIMO_m, L1_m * 0.75f);

        if (G_distanza_target_m <= raggioAccettazioneDinamico_m) {
            if (!b_waypoint_raggiunto) {
                inviaMessaggioAvionica("WAYPOINT RAGGIUNTO");
                b_waypoint_raggiunto = true;
            }
            G_distanza_target_m = 0.0f;
            return;
        } else {
            b_waypoint_raggiunto = false;
        }

        bool b_rotta_con_yaw = (G_rotta_attuale_deg < 0.0f || G_velocita_suolo_ms < VELOCITA_SUOLO_GPS_AFFIDABILE_ms);
        if (b_rotta_con_yaw) {
            G_rotta_attuale_deg = G_yaw_deg;
            if (!b_rotta_con_yaw_precedente) {
                inviaMessaggioAvionica("rotta attuale aggiornata con il yaw");
                b_rotta_con_yaw_precedente = true;
            }
        } else {
            b_rotta_con_yaw_precedente = false;
        }

        float rottaCorretta_deg = G_rotta_target_deg;   

        bool b_correzione_vento = (G_vento_velocita_ms > 3.0f && G_velocita_aria_ms > 3.0f);
        if (b_correzione_vento) {
            if (!b_correzione_vento_precedente) {
                inviaMessaggioAvionica("navigazione corretta anche con il vento)");
                b_correzione_vento_precedente = true;
            }
            float deltaVento_deg = G_vento_direzione_deg - G_rotta_target_deg;
            if (deltaVento_deg > 180.0f) deltaVento_deg -= 360.0f;
            if (deltaVento_deg < -180.0f) deltaVento_deg += 360.0f;

            float argomentoAsin = (G_vento_velocita_ms / G_velocita_aria_ms) * sin(radians(deltaVento_deg));
            argomentoAsin = constrain(argomentoAsin, -1.0f, 1.0f);

            float wca_deg = degrees(asin(argomentoAsin));
            rottaCorretta_deg = G_rotta_target_deg + wca_deg;

            if (rottaCorretta_deg >= 360.0f) rottaCorretta_deg -= 360.0f;
            if (rottaCorretta_deg < 0.0f)    rottaCorretta_deg += 360.0f;
        } else {
            b_correzione_vento_precedente = false;
        }

        G_errore_rotta_deg = rottaCorretta_deg - G_rotta_attuale_deg;

        if (G_errore_rotta_deg > 180.0f) {
            G_errore_rotta_deg -= 360.0f;
        } else if (G_errore_rotta_deg < -180.0f) {
            G_errore_rotta_deg += 360.0f;
        }

        float eta_rad = radians(G_errore_rotta_deg);
        float aLaterale_ms2 = (2.0f * velocitaPerCalcolo_ms * velocitaPerCalcolo_ms / L1_m) * sin(eta_rad);
        float rollNecessario_rad = atan(aLaterale_ms2 / 9.81f);
        
        G_roll_target_deg = constrain(degrees(rollNecessario_rad), -MAX_ROLL_deg, MAX_ROLL_deg);
        
    } else {
        if ((tempoAttuale_ms - ultimoGpsValido_ms) > TIMEOUT_GPS_ms) {
            if (!b_gps_timeout_precedente) {
                inviaMessaggioAvionica("ATTENZIONE: GPS non valido da troppo tempo, navigazione disabilitata");
                b_gps_timeout_precedente = true;
            }
            G_roll_target_deg = 0.0f;
        } else {
            b_gps_timeout_precedente = false;
        }
    }

}


// Restituisce il tempo trascorso dall'ultimo aggiornamento del PID.
// Tutti i PID usano lo stesso dt, quindi l'aggiornamento del tempo e' in un solo punto.
float aggiornaTempoPID() {
    unsigned long adesso_ms = millis();

    float dt_s =
        (adesso_ms - G_tempo_pid_precedente_ms) / 1000.0f;

    // Se siamo stati chiamati troppo presto non calcoliamo il PID.
    if (dt_s <= 0.001f) {
        return 0.0f;
    }

    // Evita una derivata enorme dopo una pausa lunga.
    if (dt_s > 0.5f) {
        dt_s = 0.5f;
    }

    // Memorizziamo il tempo per il ciclo successivo.
    G_tempo_pid_precedente_ms = adesso_ms;

    return dt_s;
}


// Calcola soltanto i PID di assetto.
// Questa funzione NON decide se siamo in stallo, overspeed o fuori quota.
// Riceve semplicemente un target pitch e un target roll e prova a raggiungerli.
void calcolaPIDAssetto(float targetPitch_deg, float targetRoll_deg, float dt_s) {
    // ---------------- PID PITCH ----------------

    // Errore = dove vogliamo essere - dove siamo.
    float errorePitch_deg = targetPitch_deg - G_pitch_deg;

    // Termine proporzionale.
    float P_pitch = KP_PITCH_DEFAULT * errorePitch_deg;

    // Termine integrale.
    G_pid_pitch_integrale += errorePitch_deg * dt_s;
    G_pid_pitch_integrale =
        constrain(G_pid_pitch_integrale, -40.0f, 40.0f);

    float I_pitch =
        KI_PITCH_DEFAULT * G_pid_pitch_integrale;

    // Termine derivativo.
    float D_pitch =
        KD_PITCH_DEFAULT *
        ((errorePitch_deg - G_pid_pitch_errore_precedente_deg) / dt_s);

    G_pid_pitch_errore_precedente_deg = errorePitch_deg;

    // Comando finale pitch verso il mixer dei servi.
    G_comando_pitch_deg =
        (int)(P_pitch + I_pitch + D_pitch);

    G_comando_pitch_deg =
        constrain(G_comando_pitch_deg, -MAX_PITCH_deg, MAX_PITCH_deg);


    // ---------------- PID ROLL ----------------

    // Errore di roll.
    float erroreRoll_deg = targetRoll_deg - G_roll_deg;

    // Termine proporzionale.
    float P_roll = KP_ROLL_DEFAULT * erroreRoll_deg;

    // Termine integrale.
    G_pid_roll_integrale += erroreRoll_deg * dt_s;
    G_pid_roll_integrale =
        constrain(G_pid_roll_integrale, -40.0f, 40.0f);

    float I_roll =
        KI_ROLL_DEFAULT * G_pid_roll_integrale;

    // Termine derivativo.
    float D_roll =
        KD_ROLL_DEFAULT *
        ((erroreRoll_deg - G_pid_roll_errore_precedente_deg) / dt_s);

    G_pid_roll_errore_precedente_deg = erroreRoll_deg;

    // Comando finale roll verso il mixer.
    G_comando_roll_deg =
        (int)(P_roll + I_roll + D_roll);

    G_comando_roll_deg =
        constrain(G_comando_roll_deg, -MAX_ROLL_deg, MAX_ROLL_deg);
}


// Questo e' il PID NORMALE del volo automatico.
// Qui dentro non esiste nessuna decisione di stallo, overspeed o quota critica.
// Quelle decisioni vengono prese prima da verificaProtezioniVolo().
void calcolaPID() {
    // Otteniamo il dt comune a tutti i controllori.
    float dt_s = aggiornaTempoPID();

    // Se dt non e' valido lasciamo i comandi dell'ultimo ciclo.
    if (dt_s <= 0.0f) {
        return;
    }

    // ========================================================
    // PID ALTITUDINE -> genera il target di pitch
    // ========================================================

    // Errore di quota.
    float erroreAltitudine_m =
        G_altitudine_target_m - G_altitudine_m;

    // Limitiamo l'errore per evitare richieste eccessive.
    erroreAltitudine_m =
        constrain(erroreAltitudine_m, -20.0f, 20.0f);

    // Parte proporzionale.
    float P_alt =
        KP_ALT_DEFAULT * erroreAltitudine_m;

    // Parte integrale.
    G_pid_alt_integrale += erroreAltitudine_m * dt_s;
    G_pid_alt_integrale =
        constrain(G_pid_alt_integrale, -20.0f, 20.0f);

    float I_alt =
        KI_ALT_DEFAULT * G_pid_alt_integrale;

    // Parte derivativa.
    float D_alt =
        KD_ALT_DEFAULT *
        ((erroreAltitudine_m - G_pid_alt_errore_precedente_m) / dt_s);

    G_pid_alt_errore_precedente_m = erroreAltitudine_m;

    // Il PID quota non muove direttamente il servo:
    // produce il pitch che il PID di assetto deve raggiungere.
    float targetPitch_deg =
        constrain(P_alt + I_alt + D_alt, -10.0f, 15.0f);

    // PID pitch + roll.
    calcolaPIDAssetto(targetPitch_deg, G_roll_target_deg, dt_s);


    // ========================================================
    // PID VELOCITA -> genera il comando gas
    // ========================================================

    // Se il Pitot non e' affidabile non usiamo una velocita' falsa (-1)
    // dentro al PID: manteniamo semplicemente il gas base previsto.
    if (!B_pitot_disponibile) {
        G_pid_vel_integrale = 0.0f;
        G_pid_vel_errore_precedente_kmh = 0.0f;
        G_comando_gas_us = G_gas_base_us;
        return;
    }

    // Airspeed in km/h.
    float velocitaAria_kmh =
        G_velocita_aria_ms * 3.6f;

    // Errore di velocita'.
    float erroreVelocita_kmh =
        G_target_velocita_kmh - velocitaAria_kmh;

    // Parte proporzionale.
    float P_vel =
        KP_VEL_DEFAULT * erroreVelocita_kmh;

    // Parte integrale.
    G_pid_vel_integrale += erroreVelocita_kmh * dt_s;
    G_pid_vel_integrale =
        constrain(G_pid_vel_integrale, -30.0f, 30.0f);

    float I_vel =
        KI_VEL_DEFAULT * G_pid_vel_integrale;

    // Parte derivativa.
    float D_vel =
        KD_VEL_DEFAULT *
        ((erroreVelocita_kmh - G_pid_vel_errore_precedente_kmh) / dt_s);

    G_pid_vel_errore_precedente_kmh = erroreVelocita_kmh;

    // Sommiamo la correzione PID al gas base scelto dalla fase di volo.
    int gasCalcolato_us =
        G_gas_base_us + (int)(P_vel + I_vel + D_vel);

    // Limiti fisici dell'ESC.
    G_comando_gas_us =
        constrain(gasCalcolato_us, GAS_MINIMO_us, GAS_MASSIMO_us);
}


// Decide se il drone e' fuori dall'inviluppo normale di volo.
// IMPORTANTE: questa funzione decide SOLO quale protezione serve.
// Non contiene nessun PID.
int verificaProtezioniVolo() {
    // Stato mantenuto per applicare isteresi a stallo e overspeed.
    static bool b_in_stallo = false;
    static bool b_in_overspeed = false;

    // Stati precedenti usati solo per stampare i messaggi una volta.
    bool b_stallo_precedente = b_in_stallo;
    bool b_overspeed_precedente = b_in_overspeed;

    // Il Pitot e' valido solo se la lettura corrente e' realmente disponibile.
    bool b_airspeed_valida = B_pitot_disponibile;

    // Convertiamo l'airspeed una sola volta.
    float velocitaAria_kmh =
        b_airspeed_valida ? G_velocita_aria_ms * 3.6f : -1.0f;

    // --------------------------------------------------------
    // STALLO MULTI-PARAMETRO
    // --------------------------------------------------------
    // 1) rilevamento classico: airspeed sotto la soglia;
    // 2) rilevamento anticipato: airspeed vicina alla soglia +
    //    pitch alto + il drone non sta piu' salendo.
    //
    // Non usiamo G_accel_z come "accelerazione verticale":
    // quella e' sull'asse Z del drone, non sull'asse verticale terrestre.
    // Il rateo G_velocita_verticale_ms e' piu' semplice e meno ambiguo.
    const float PITCH_STALLO_PRECOCE_deg = 15.0f;

    bool b_stallo_da_velocita =
        b_airspeed_valida &&
        velocitaAria_kmh < VELOCITA_STALLO_X8_kmh;

    bool b_stallo_precoce =
        b_airspeed_valida &&
        velocitaAria_kmh < (VELOCITA_STALLO_X8_kmh + MARGINE_ISTERESI_kmh) &&
        G_pitch_deg > PITCH_STALLO_PRECOCE_deg &&
        G_velocita_verticale_ms <= 0.0f;

    // Quando entriamo nello stallo basta una delle due condizioni.
    if (!b_in_stallo) {
        b_in_stallo =
            b_stallo_da_velocita ||
            b_stallo_precoce;
    }
    // Quando siamo gia' nello stallo richiediamo un margine di recupero.
    else if (b_airspeed_valida) {
        b_in_stallo =
            velocitaAria_kmh <
                (VELOCITA_STALLO_X8_kmh + MARGINE_ISTERESI_kmh) ||
            b_stallo_precoce;
    }

    // --------------------------------------------------------
    // OVERSPEED CON ISTERESI
    // --------------------------------------------------------
    if (!b_in_overspeed) {
        b_in_overspeed =
            b_airspeed_valida &&
            velocitaAria_kmh > MAX_AIRSPEED_X8_kmh;
    } else if (b_airspeed_valida) {
        b_in_overspeed =
            velocitaAria_kmh >
            (MAX_AIRSPEED_X8_kmh - MARGINE_ISTERESI_kmh);
    }

    // Lo stallo ha priorita' sull'overspeed.
    if (b_in_stallo) {
        b_in_overspeed = false;
    }

    // Messaggio soltanto quando cambia lo stato.
    if (b_in_stallo != b_stallo_precedente) {
        inviaMessaggioAvionica(
            b_in_stallo
                ? "ATTENZIONE: STALLO / PRE-STALLO rilevato"
                : "Stallo rientrato"
        );
    }

    // Messaggio soltanto quando cambia lo stato.
    if (b_in_overspeed != b_overspeed_precedente) {
        inviaMessaggioAvionica(
            b_in_overspeed
                ? "ATTENZIONE: OVERSPEED rilevato"
                : "Overspeed rientrato"
        );
    }

    // Priorita' delle protezioni.
    if (b_in_stallo) {
        return PROTEZIONE_STALLO;
    }

    if (b_in_overspeed) {
        return PROTEZIONE_OVERSPEED;
    }

    if (G_altitudine_m > ALTITUDINE_MAX_m) {
        return PROTEZIONE_QUOTA_MASSIMA;
    }

    // Forma corretta della condizione:
    // prima confrontiamo direttamente la quota con ALTITUDINE_MIN_m.
    if (G_altitudine_m < ALTITUDINE_MIN_m) {
        return PROTEZIONE_QUOTA_MINIMA;
    }

    return PROTEZIONE_NESSUNA;
}


// Applica i comandi richiesti dalla protezione.
// La logica di safety e' quindi completamente fuori da calcolaPID().
void calcolaComandiProtezione(int protezione) {
    // Aggiorniamo il tempo del controllore di assetto.
    float dt_s = aggiornaTempoPID();

    if (dt_s <= 0.0f) {
        return;
    }

    // Evitiamo wind-up dei PID quota e velocita' mentre una protezione
    // sta comandando direttamente pitch/gas.
    G_pid_alt_integrale = 0.0f;
    G_pid_alt_errore_precedente_m = 0.0f;
    G_pid_vel_integrale = 0.0f;
    G_pid_vel_errore_precedente_kmh = 0.0f;

    // Target scelti dalla protezione.
    float targetPitch_deg = 0.0f;
    float targetRoll_deg = G_roll_target_deg;
    int gas_us = G_gas_base_us;

    if (protezione == PROTEZIONE_STALLO) {
        // Per recuperare dallo stallo abbassiamo il muso,
        // livelliamo le ali e chiediamo gas massimo.
        targetPitch_deg = PITCH_DOWN_FORZATO_deg;
        targetRoll_deg = 0.0f;
        gas_us = GAS_MASSIMO_us;
    }
    else if (protezione == PROTEZIONE_OVERSPEED) {
        // Per ridurre velocita' alziamo il muso e portiamo il gas al minimo.
        targetPitch_deg = -PITCH_DOWN_FORZATO_deg;
        gas_us = GAS_MINIMO_us;
    }
    else if (protezione == PROTEZIONE_QUOTA_MASSIMA) {
        // Sopra la quota massima chiediamo discesa e gas minimo.
        targetPitch_deg = PITCH_DOWN_FORZATO_deg;
        gas_us = GAS_MINIMO_us;
    }
    else if (protezione == PROTEZIONE_QUOTA_MINIMA) {
        // Sotto la quota minima chiediamo salita e gas massimo.
        targetPitch_deg = PITCH_UP_FORZATO_deg;
        gas_us = GAS_MASSIMO_us;
    }

    // Il target di assetto viene comunque raggiunto tramite i PID pitch/roll.
    // La safety decide COSA chiedere, il controllore decide COME muovere i servi.
    calcolaPIDAssetto(targetPitch_deg, targetRoll_deg, dt_s);

    // Il gas invece e' imposto direttamente dalla protezione.
    G_comando_gas_us = gas_us;
}

int gasMaxTermico() {
    static int statoTermicoPrecedente = -1;

    if (G_temperatura_motore_c >= MOTORE_TEMP_DERATING_END_C || 
        G_temperatura_esc_c >= ESC_TEMP_DERATING_END_C) {
        if (statoTermicoPrecedente != 0) {
            inviaMessaggioAvionica("ATTENZIONE: temperatura motore o ESC troppo alta, gas ridotto al minimo");
            statoTermicoPrecedente = 0;
        }
        return GAS_MINIMO_us;
    }

    if (G_temperatura_motore_c <= MOTORE_TEMP_DERATING_START_C && 
        G_temperatura_esc_c <= ESC_TEMP_DERATING_START_C) {
        if (statoTermicoPrecedente != 1) {
            inviaMessaggioAvionica("Temperatura motore e ESC nella zona sicura, gas massimo consentito");
            statoTermicoPrecedente = 1;
        }
        return GAS_MASSIMO_us;
    }

    float fattoreMotore = 0.0f;
    float fattoreESC = 0.0f;

    if (G_temperatura_motore_c > MOTORE_TEMP_DERATING_START_C) {
        if (MOTORE_TEMP_DERATING_END_C > MOTORE_TEMP_DERATING_START_C) {
            fattoreMotore = (G_temperatura_motore_c - MOTORE_TEMP_DERATING_START_C) / 
                            (MOTORE_TEMP_DERATING_END_C - MOTORE_TEMP_DERATING_START_C);
        } else {
            fattoreMotore = 1.0f;
        }
    }

    if (G_temperatura_esc_c > ESC_TEMP_DERATING_START_C) {
        if (ESC_TEMP_DERATING_END_C > ESC_TEMP_DERATING_START_C) {
            fattoreESC = (G_temperatura_esc_c - ESC_TEMP_DERATING_START_C) / 
                         (ESC_TEMP_DERATING_END_C - ESC_TEMP_DERATING_START_C);
        } else {
            fattoreESC = 1.0f;
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

    if (fattoreInterpolazione > 1.0f) {
        fattoreInterpolazione = 1.0f;
    }

    float limite_us = GAS_MASSIMO_us - (fattoreInterpolazione * (GAS_MASSIMO_us - GAS_MINIMO_us));
    
    return (int)(limite_us + 0.5f);
}

void resettaPID() {
    G_pid_alt_integrale   = 0.0f;  G_pid_alt_errore_precedente_m   = 0.0f;
    G_pid_pitch_integrale = 0.0f;  G_pid_pitch_errore_precedente_deg = 0.0f;
    G_pid_roll_integrale  = 0.0f;  G_pid_roll_errore_precedente_deg  = 0.0f;
    G_pid_vel_integrale   = 0.0f;  G_pid_vel_errore_precedente_kmh   = 0.0f;
    G_tempo_pid_precedente_ms = millis();
}


// Trasforma i comandi pitch/roll nei quattro angoli dei servi.
// Legge direttamente G_comando_pitch_deg e G_comando_roll_deg.
void applicaMixer4Servi() {
    // Copie locali con nomi corti: rendono leggibile tutta la matematica del mixer.
    int pitch_deg = G_comando_pitch_deg;
    int roll_deg  = G_comando_roll_deg;

    // Questi stati servono solo a questa funzione, quindi NON sono globali.
    static bool b_interni_precedenti = true;
    static bool b_esterni_precedenti = true;

    // Anche lo stato precedente dei singoli servi serve solo al mixer.
    static bool b_int_sx_precedente = true;
    static bool b_int_dx_precedente = true;
    static bool b_est_sx_precedente = true;
    static bool b_est_dx_precedente = true;

    static int casoMixerPrecedente = -1;
    int posIntSX_deg = CENTRO_SERVO_deg;
    int posIntDX_deg = CENTRO_SERVO_deg;
    int posEstSX_deg = CENTRO_SERVO_deg;
    int posEstDX_deg = CENTRO_SERVO_deg;

    if (B_batteria_bassa_teensy) {
        B_SERVO_INT_SX_OK = false;
        B_SERVO_INT_DX_OK = false;
    }

    bool b_esterni_attivi = B_SERVO_EST_SX_OK && B_SERVO_EST_DX_OK;
    bool b_interni_attivi = B_SERVO_INT_SX_OK && B_SERVO_INT_DX_OK;

    if (b_esterni_attivi && b_interni_attivi) {
        if (casoMixerPrecedente != 0) {
            inviaMessaggioAvionica("Caso A: tutto OK — interni = SOLO PITCH, esterni = SOLO ROLL");
            casoMixerPrecedente = 0;
        }
        posIntSX_deg = CENTRO_SERVO_deg + pitch_deg;
        posIntDX_deg = CENTRO_SERVO_deg + pitch_deg;
        posEstSX_deg = CENTRO_SERVO_deg + roll_deg;
        posEstDX_deg = CENTRO_SERVO_deg - roll_deg;
    } else if (b_esterni_attivi && !b_interni_attivi) {
        if (casoMixerPrecedente != 1) {
            inviaMessaggioAvionica("Caso B: interni rotti — esterni fanno pitch + roll");
            casoMixerPrecedente = 1;
        }
        posEstSX_deg = CENTRO_SERVO_deg + pitch_deg + roll_deg;
        posEstDX_deg = CENTRO_SERVO_deg + pitch_deg - roll_deg;
    } else if (!b_esterni_attivi && b_interni_attivi) {
        if (casoMixerPrecedente != 2) {
            inviaMessaggioAvionica("Caso C: esterni rotti — interni fanno pitch + roll");
            casoMixerPrecedente = 2;
        }
        posIntSX_deg = CENTRO_SERVO_deg + pitch_deg + roll_deg;
        posIntDX_deg = CENTRO_SERVO_deg + pitch_deg - roll_deg;
    } else {
        // Se non esiste piu' una coppia completa, controlliamo se almeno
        // un singolo servo e' ancora funzionante. In quel caso lo usiamo
        // soltanto per il pitch, con autorita' minima di emergenza.
        bool b_singolo_int_sx = B_SERVO_INT_SX_OK && !B_SERVO_INT_DX_OK;
        bool b_singolo_int_dx = !B_SERVO_INT_SX_OK && B_SERVO_INT_DX_OK;
        bool b_singolo_est_sx = B_SERVO_EST_SX_OK && !B_SERVO_EST_DX_OK;
        bool b_singolo_est_dx = !B_SERVO_EST_SX_OK && B_SERVO_EST_DX_OK;

        if (b_singolo_int_sx || b_singolo_int_dx || b_singolo_est_sx || b_singolo_est_dx) {
            if (casoMixerPrecedente != 4) {
                inviaMessaggioAvionica("Caso E: nessuna coppia completa — uso il singolo servo superstite solo per pitch");
                casoMixerPrecedente = 4;
            }
            if (b_singolo_int_sx) posIntSX_deg = CENTRO_SERVO_deg + pitch_deg;
            if (b_singolo_int_dx) posIntDX_deg = CENTRO_SERVO_deg + pitch_deg;
            if (b_singolo_est_sx) posEstSX_deg = CENTRO_SERVO_deg + pitch_deg;
            if (b_singolo_est_dx) posEstDX_deg = CENTRO_SERVO_deg + pitch_deg;
        } else {
            if (casoMixerPrecedente != 3) {
                inviaMessaggioAvionica("Caso D: tutti i servi rotti: nulla da comandare");
                casoMixerPrecedente = 3;
            }
            return;
        }
    }

    if (b_interni_attivi != b_interni_precedenti) {
        inviaMessaggioAvionica(b_interni_attivi ? "Servi interni: coppia ATTIVA" : "Servi interni: coppia NON completa");
        b_interni_precedenti = b_interni_attivi;
    }
    if (b_esterni_attivi != b_esterni_precedenti) {
        inviaMessaggioAvionica(b_esterni_attivi ? "Servi esterni: coppia ATTIVA" : "Servi esterni: coppia NON completa");
        b_esterni_precedenti = b_esterni_attivi;
    }

    // Attach/detach viene gestito per ogni servo singolarmente:
    // un servo sano rimane utilizzabile anche se il suo compagno e' guasto.
    if (B_SERVO_INT_SX_OK != b_int_sx_precedente) {
        if (B_SERVO_INT_SX_OK) servoInternoSX.attach(PIN_INT_SX); else servoInternoSX.detach();
        inviaMessaggioAvionica(B_SERVO_INT_SX_OK ? "ServoIntSX: ATTACCATO" : "ServoIntSX: STACCATO");
        b_int_sx_precedente = B_SERVO_INT_SX_OK;
    }
    if (B_SERVO_INT_DX_OK != b_int_dx_precedente) {
        if (B_SERVO_INT_DX_OK) servoInternoDX.attach(PIN_INT_DX); else servoInternoDX.detach();
        inviaMessaggioAvionica(B_SERVO_INT_DX_OK ? "ServoIntDX: ATTACCATO" : "ServoIntDX: STACCATO");
        b_int_dx_precedente = B_SERVO_INT_DX_OK;
    }
    if (B_SERVO_EST_SX_OK != b_est_sx_precedente) {
        if (B_SERVO_EST_SX_OK) servoEsternoSX.attach(PIN_EST_SX); else servoEsternoSX.detach();
        inviaMessaggioAvionica(B_SERVO_EST_SX_OK ? "ServoEstSX: ATTACCATO" : "ServoEstSX: STACCATO");
        b_est_sx_precedente = B_SERVO_EST_SX_OK;
    }
    if (B_SERVO_EST_DX_OK != b_est_dx_precedente) {
        if (B_SERVO_EST_DX_OK) servoEsternoDX.attach(PIN_EST_DX); else servoEsternoDX.detach();
        inviaMessaggioAvionica(B_SERVO_EST_DX_OK ? "ServoEstDX: ATTACCATO" : "ServoEstDX: STACCATO");
        b_est_dx_precedente = B_SERVO_EST_DX_OK;
    }

    posIntSX_deg = constrain(posIntSX_deg, 45, 135);
    posIntDX_deg = constrain(posIntDX_deg, 45, 135);
    posEstSX_deg = constrain(posEstSX_deg, 45, 135);
    posEstDX_deg = constrain(posEstDX_deg, 45, 135);

    // Scriviamo soltanto sui servi che risultano funzionanti.
    if (B_SERVO_INT_SX_OK) servoInternoSX.write(posIntSX_deg);
    if (B_SERVO_INT_DX_OK) servoInternoDX.write(posIntDX_deg);
    if (B_SERVO_EST_SX_OK) servoEsternoSX.write(posEstSX_deg);
    if (B_SERVO_EST_DX_OK) servoEsternoDX.write(posEstDX_deg);

}

bool letturaINAValida(float tensione_V) {
    return isfinite(tensione_V) &&
           tensione_V >= INA219_TENSIONE_MIN_PLAUSIBILE_V &&
           tensione_V <= INA219_TENSIONE_MAX_PLAUSIBILE_V;
}

void diagnosticaServi() {
    if (!B_servo_sicurezza) {
        B_SERVO_EST_SX_OK = B_SERVO_EST_DX_OK = B_SERVO_INT_SX_OK = B_SERVO_INT_DX_OK = true;
        return;
    }

    static int erroriConsecutivi[4] = {0, 0, 0, 0};

    float corrente_mA;
    float tensione_V;

    tensione_V = sensoreEstSX.getBusVoltage_V();
    B_INA219_EST_SX_OK = letturaINAValida(tensione_V);
    if (B_INA219_EST_SX_OK) {
        G_tensione_servo_est_sx_v = tensione_V;
        corrente_mA = sensoreEstSX.getCurrent_mA();
        if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
            erroriConsecutivi[0]++;
            if (erroriConsecutivi[0] > ERRORI_CONSECUTIVI_SERVO) B_SERVO_EST_SX_OK = false;
        } else {
            erroriConsecutivi[0] = 0;
            B_SERVO_EST_SX_OK = true;
        }
    }

    tensione_V = sensoreEstDX.getBusVoltage_V();
    B_INA219_EST_DX_OK = letturaINAValida(tensione_V);
    if (B_INA219_EST_DX_OK) {
        G_tensione_servo_est_dx_v = tensione_V;
        corrente_mA = sensoreEstDX.getCurrent_mA();
        if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
            erroriConsecutivi[1]++;
            if (erroriConsecutivi[1] > ERRORI_CONSECUTIVI_SERVO) B_SERVO_EST_DX_OK = false;
        } else {
            erroriConsecutivi[1] = 0;
            B_SERVO_EST_DX_OK = true;
        }
    }

    tensione_V = sensoreIntSX.getBusVoltage_V();
    B_INA219_INT_SX_OK = letturaINAValida(tensione_V);
    if (B_INA219_INT_SX_OK) {
        G_tensione_servo_int_sx_v = tensione_V;
        corrente_mA = sensoreIntSX.getCurrent_mA();
        if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
            erroriConsecutivi[2]++;
            if (erroriConsecutivi[2] > ERRORI_CONSECUTIVI_SERVO) B_SERVO_INT_SX_OK = false;
        } else {
            erroriConsecutivi[2] = 0;
            B_SERVO_INT_SX_OK = true;
        }
    }

    tensione_V = sensoreIntDX.getBusVoltage_V();
    B_INA219_INT_DX_OK = letturaINAValida(tensione_V);
    if (B_INA219_INT_DX_OK) {
        G_tensione_servo_int_dx_v = tensione_V;
        corrente_mA = sensoreIntDX.getCurrent_mA();
        if (corrente_mA < SERVO_mA_MIN || corrente_mA > SERVO_mA_MAX) {
            erroriConsecutivi[3]++;
            if (erroriConsecutivi[3] > ERRORI_CONSECUTIVI_SERVO) B_SERVO_INT_DX_OK = false;
        } else {
            erroriConsecutivi[3] = 0;
            B_SERVO_INT_DX_OK = true;
        }
    }

    B_INA219_OK = B_INA219_MOTORE_OK && B_INA219_TEENSY_OK &&
                  B_INA219_INT_SX_OK && B_INA219_INT_DX_OK &&
                  B_INA219_EST_SX_OK && B_INA219_EST_DX_OK;
}


void gestisciAlimentazione() {
    unsigned long tempo_attuale_ms = millis();
    float dt_ore = (tempo_attuale_ms - G_tempo_batteria_precedente_ms) / 3600000.0f;

    float tensioneTeensy_V = sensoreTeensy.getBusVoltage_V();
    float tensioneMotore_V = sensoreMotore.getBusVoltage_V();

    B_INA219_TEENSY_OK = B_INA219_INIZIALIZZATO && letturaINAValida(tensioneTeensy_V);
    B_INA219_MOTORE_OK = B_INA219_INIZIALIZZATO && letturaINAValida(tensioneMotore_V);

    if (B_INA219_TEENSY_OK) {
        G_tensione_teensy_v = tensioneTeensy_V;
        G_corrente_teensy_ma = sensoreTeensy.getCurrent_mA();
        G_carica_consumata_teensy += G_corrente_teensy_ma * dt_ore;

        G_carica_rimanente_teensy_percentuale =
            ((CAPACITA_TEENSY_mAh - G_carica_consumata_teensy) / CAPACITA_TEENSY_mAh) * 100.0f;

        G_autonomia_teensy_residua = G_corrente_teensy_ma > 0.0f
            ? (CAPACITA_TEENSY_mAh - G_carica_consumata_teensy) / G_corrente_teensy_ma
            : -1.0f;
    }

    if (B_INA219_MOTORE_OK) {
        G_tensione_motore_v = tensioneMotore_V;
        G_corrente_motore_ma = sensoreMotore.getCurrent_mA();
        G_carica_consumata_motore += G_corrente_motore_ma * dt_ore;

        G_carica_rimanente_motore_percentuale =
            ((CAPACITA_MOTORE_mAh - G_carica_consumata_motore) / CAPACITA_MOTORE_mAh) * 100.0f;

        G_autonomia_motore_residua = G_corrente_motore_ma > 0.0f
            ? (CAPACITA_MOTORE_mAh - G_carica_consumata_motore) / G_corrente_motore_ma
            : -1.0f;
    }

    B_INA219_OK = B_INA219_MOTORE_OK && B_INA219_TEENSY_OK &&
                  B_INA219_INT_SX_OK && B_INA219_INT_DX_OK &&
                  B_INA219_EST_SX_OK && B_INA219_EST_DX_OK;

    static bool b_corrente_motore_eccessiva_precedente = false;
    B_corrente_motore_eccessiva = B_INA219_MOTORE_OK &&
        (G_corrente_motore_ma > CORRENTE_MOTORE_MAX_PLAUSIBILE_mA);

    if (B_corrente_motore_eccessiva != b_corrente_motore_eccessiva_precedente) {
        inviaMessaggioAvionica(B_corrente_motore_eccessiva
            ? "ATTENZIONE: corrente motore eccessiva rilevata"
            : "Corrente motore rientrata nei limiti plausibili");
        b_corrente_motore_eccessiva_precedente = B_corrente_motore_eccessiva;
    }

    if (!B_alimentazione_sicurezza) {
        B_batteria_bassa_teensy = false;
        B_batteria_bassa_motore = false;
    } else {
        if (B_INA219_TEENSY_OK) {
            bool b_precedente = B_batteria_bassa_teensy;
            B_batteria_bassa_teensy = (G_tensione_teensy_v < VALORE_BATT_TEENSY_BASSA_V);

            if (B_batteria_bassa_teensy && !b_precedente) {
                inviaMessaggioAvionica("ATTENZIONE: batteria Teensy bassa, commutazione su batteria motore apertura rele");
            }

            if (B_batteria_bassa_teensy && !B_rele_attivato) {
                digitalWrite(PIN_RELE, HIGH);
                B_rele_attivato = true;
                inviaMessaggioAvionica("Rele' alimentazione ATTIVATO (failover su batteria motore)");
            }
        }

        if (B_INA219_MOTORE_OK) {
            bool b_precedente = B_batteria_bassa_motore;
            B_batteria_bassa_motore = (G_tensione_motore_v < VALORE_BATT_MOTORE_BASSA_V);

            if (B_batteria_bassa_motore && !b_precedente) {
                inviaMessaggioAvionica("ATTENZIONE: batteria motore bassa, considerare atterraggio");
            }
        }
    }

    G_tempo_batteria_precedente_ms = tempo_attuale_ms;
}

void gestisciSchianto() {
    if (!B_schianto_sicurezza) {
        B_stato_schianto_rilevato = false;
        return;
    }
    if (B_stato_schianto_rilevato) {
        scriviMotore(GAS_NEUTRO_us);
        return;
    }

    if (!B_drone_in_volo) return;
    if (!B_BNO055_OK) return;

    imu::Vector<3> accel = giroscopio.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    float accelerazioneTotale_ms2 = sqrt((accel.x() * accel.x()) + (accel.y() * accel.y()) + (accel.z() * accel.z()));
static unsigned long tempoInizioPicco_ms = 0;
    static bool b_picco_in_corso = false;
    unsigned long tempoAttuale_ms = millis();
    
    
    if (accelerazioneTotale_ms2 > SOGLIA_ACCELERAZIONE_SCHIANTO_ms2) {
        if (!b_picco_in_corso) {
            b_picco_in_corso = true;
            tempoInizioPicco_ms = tempoAttuale_ms;
        } else {
            if ((tempoAttuale_ms - tempoInizioPicco_ms) >= TEMPO_CONFERMA_SCHIANTO_ms) {
                if (G_velocita_aria_ms < SOGLIA_VELOCITA_CRITICA_SCHIANTO_ms) {
                    
                    inviaMessaggioAvionica("CRITICAL: SCHIANTO CONFERMATO ");
                    B_stato_schianto_rilevato = true;
                    B_schianto_bloccato = true;
                    inviaMessaggioAvionica("CRITICAL: DISATTIVO SERVI E MOTORE");
                    scriviMotore(GAS_NEUTRO_us);
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
        b_picco_in_corso = false;
    }
}

void verificaDroneInVolo() {
    if (!B_drone_in_volo) {
        bool b_velocita_sufficiente   = (G_velocita_aria_ms > SOGLIA_VELOCITA_DECOLLO_ms);
        bool b_altitudine_sufficiente = (G_altitudine_m > SOGLIA_ALTITUDINE_DECOLLO_m);

        if (b_velocita_sufficiente && b_altitudine_sufficiente) {
            if (TIMESTAMP_DECOLLO_ms == 0) {
                TIMESTAMP_DECOLLO_ms = millis();
            }
            if (millis() - TIMESTAMP_DECOLLO_ms >= TEMPO_DECOLLO_SICURO_ms) {
                inviaMessaggioAvionica("velocita e altitudine di decollo confermate");
                B_drone_in_volo = true;
                TIMESTAMP_DECOLLO_ms = 0;
                inviaMessaggioAvionica("DECOLLO CONFERMATO: drone in volo, monitoraggio schianto attivo");
            }
        } else {
            TIMESTAMP_DECOLLO_ms = 0;
        }
    }
}

void gestisciAllarmi() {
    if (!B_SERVO_EST_SX_OK || !B_SERVO_EST_DX_OK || !B_SERVO_INT_SX_OK || !B_SERVO_INT_DX_OK) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        digitalWrite(PIN_LED_VERDE_GPS, HIGH);
        digitalWrite(PIN_LED_BLU_PID, HIGH);
        return;
    }
    digitalWrite(PIN_LED_ROSSO_ALARM, (B_batteria_bassa_motore || B_failsafe) ? HIGH : LOW);

    digitalWrite(PIN_LED_VERDE_GPS, gpsValido() ? HIGH : LOW);

    digitalWrite(PIN_LED_BLU_PID, (G_modalita_volo == 2) ? HIGH : LOW);
}

void aggiornaDiagnosticaIMU() {
    if (!B_BNO055_INIZIALIZZATO) {
        G_imu_cal_sys = G_imu_cal_gyro = G_imu_cal_accel = G_imu_cal_mag = 0;
        return;
    }

    giroscopio.getCalibration(&G_imu_cal_sys, &G_imu_cal_gyro, &G_imu_cal_accel, &G_imu_cal_mag);
}


// Invia lo stato corrente alla stazione di terra.
// Non riceve parametri: legge direttamente lo stato globale aggiornato nel loop.
void inviaTelemetria() {
TELEMETRIA.print("$,");

    TELEMETRIA.print(G_modalita_volo); TELEMETRIA.print(",");
    TELEMETRIA.print(B_drone_in_volo ? "1" : "0"); TELEMETRIA.print(","); 
    TELEMETRIA.print(B_failsafe ? "1" : "0"); TELEMETRIA.print(","); 
    TELEMETRIA.print(B_stato_schianto_rilevato ? "1" : "0"); TELEMETRIA.print(","); 
    TELEMETRIA.print(B_BNO055_OK? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(B_PMW3901_OK? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(B_BMP390_OK? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(B_INA219_OK? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(B_GPS_OK? "1":"0");TELEMETRIA.print(",");
    TELEMETRIA.print(G_imu_cal_sys); TELEMETRIA.print(",");
    TELEMETRIA.print(G_imu_cal_gyro); TELEMETRIA.print(",");
    TELEMETRIA.print(G_imu_cal_accel); TELEMETRIA.print(",");
    TELEMETRIA.print(G_imu_cal_mag); TELEMETRIA.print(",");

    TELEMETRIA.print(G_tensione_motore_v, 2); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_tensione_teensy_v, 2); TELEMETRIA.print(","); 

    TELEMETRIA.print(G_carica_rimanente_teensy_percentuale, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_carica_rimanente_motore_percentuale, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_autonomia_teensy_residua, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_autonomia_motore_residua, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_corrente_teensy_ma, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_corrente_motore_ma, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(B_corrente_motore_eccessiva ? "1" : "0"); TELEMETRIA.print(",");
    TELEMETRIA.print(G_tensione_servo_int_sx_v, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_tensione_servo_int_dx_v, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_tensione_servo_est_sx_v, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_tensione_servo_est_dx_v, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(B_servo_sicurezza ? "1" : "0"); TELEMETRIA.print(",");

    TELEMETRIA.print(G_pitch_deg, 1); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_roll_deg, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_yaw_deg, 1);   TELEMETRIA.print(","); 
            
    TELEMETRIA.print(G_altitudine_m, 1); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_altitudine_lidar_m, 2); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_altitudine_baro_m, 2);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_altitudine_target_m, 1); TELEMETRIA.print(",");
    TELEMETRIA.print(G_velocita_verticale_ms, 2); TELEMETRIA.print(",");
    TELEMETRIA.print(B_sink_rate_eccessivo ? "1" : "0"); TELEMETRIA.print(",");

    TELEMETRIA.print((G_velocita_aria_ms * 3.6f), 1);      TELEMETRIA.print(","); 
    TELEMETRIA.print((G_velocita_suolo_ms * 3.6f), 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_vento_velocita_ms * 3.6f, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_vento_direzione_deg, 1);       TELEMETRIA.print(","); 
    TELEMETRIA.print(G_velocita_crociera_kmh, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_velocita_avvicinamento_kmh, 1);       TELEMETRIA.print(","); 


    TELEMETRIA.print(G_distanza_target_m, 0); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_rotta_target_deg, 1);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_roll_target_deg, 1);   TELEMETRIA.print(","); 
    TELEMETRIA.print(G_rotta_attuale_deg, 1);     TELEMETRIA.print(",");

    TELEMETRIA.print(G_canali_rc[1]); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_canali_rc[0]); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_canali_rc[2]); TELEMETRIA.print(","); 

    TELEMETRIA.print(G_comando_pitch_deg); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_comando_roll_deg);  TELEMETRIA.print(","); 
    TELEMETRIA.print(G_comando_gas_us);    TELEMETRIA.print(","); 

    TELEMETRIA.print(servoInternoSX.read()); TELEMETRIA.print(","); 
    TELEMETRIA.print(servoInternoDX.read()); TELEMETRIA.print(","); 
    TELEMETRIA.print(servoEsternoSX.read()); TELEMETRIA.print(","); 
    TELEMETRIA.print(servoEsternoDX.read()); TELEMETRIA.print(","); 

    TELEMETRIA.print(B_limitazione_termica_attiva ? "1" : "0"); TELEMETRIA.print(",");
    TELEMETRIA.print(G_temperatura_motore_c, 1);    TELEMETRIA.print(","); 
    TELEMETRIA.print(G_temperatura_fusoliera_c, 1); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_temperatura_esterna_c, 1);   TELEMETRIA.print(","); 
    TELEMETRIA.print(G_temperatura_esc_c, 1);       TELEMETRIA.print(","); 
    TELEMETRIA.print(G_gas_limite_termico_us);      TELEMETRIA.print(","); 

    if (gpsValido()) {
        TELEMETRIA.print(G_numero_satelliti); TELEMETRIA.print(","); 
        TELEMETRIA.print(G_drone_lat_deg, 6);  TELEMETRIA.print(","); 
        TELEMETRIA.print(G_drone_lon_deg, 6);  TELEMETRIA.print(","); 
        
        TELEMETRIA.print(G_errore_gps);     TELEMETRIA.print(","); 
        
        TELEMETRIA.print(G_errore_rotta_deg, 1);    TELEMETRIA.print(","); 
    } else {
        TELEMETRIA.print("0,0.0,0.0,-1.0,0.0,"); 
    }
    
    TELEMETRIA.print(B_alimentazione_sicurezza ? "1" : "0"); TELEMETRIA.print(",");
    TELEMETRIA.print(B_rele_attivato ? "1" : "0");           TELEMETRIA.print(",");
    TELEMETRIA.print(B_motore_disabilitato_da_terra ? "1" : "0"); TELEMETRIA.print(",");

    TELEMETRIA.print(G_velocita_ottica_x_ms, 2); TELEMETRIA.print(","); 
    TELEMETRIA.print(G_velocita_ottica_y_ms, 2);

    TELEMETRIA.println();

}

void comandiDaTerra() {
    static String buffer[2] = { "", "" };
    Stream* fonti[] = { &TELEMETRIA, &Serial };
    for (int i = 0; i < 2; i++) {
        Stream* porta = fonti[i];
        while (porta->available()) {
            char c = porta->read();
            if (c == '\n') {
                // Riga completa: la passiamo al parser dei comandi.
                elaboraComando(buffer[i]);

                // Svuotiamo il buffer per il comando successivo.
                buffer[i] = "";
            } else {
                buffer[i] += c;
                if (buffer[i].length() > 64) buffer[i] = "";
            }
        }
    }
}

// Interpreta un comando ricevuto da Serial o telemetria.
//
// Formato:
//     CMD:NOME_COMANDO:VALORE
//
// Non esistono piu' ACK/NACK e non e' possibile modificare Kp/Ki/Kd da terra.
// Ogni comando stampa semplicemente un messaggio leggibile con inviaMessaggioAvionica().
void elaboraComando(const String& cmd) {
    // Ignoriamo tutto cio' che non inizia con "CMD:".
    if (!cmd.startsWith("CMD:")) {
        return;
    }

    // Cerchiamo i due punti che separano nome e valore.
    int separatore = cmd.indexOf(':', 4);

    // Se manca il separatore il comando e' incompleto.
    if (separatore < 0) {
        inviaMessaggioAvionica("Comando non valido");
        return;
    }

    // Nome del comando.
    String campo = cmd.substring(4, separatore);

    // Parte dopo il secondo ':'.
    String valoreStr = cmd.substring(separatore + 1);

    // Versione intera, utile per servi, modalita' e gas.
    int valore = valoreStr.toInt();


    // ========================================================
    // SERVI
    // ========================================================

    if (campo == "SERVO_ISX") {
        servoInternoSX.write(constrain(valore, 45, 135));
        inviaMessaggioAvionica("Servo interno SX comandato");
    }
    else if (campo == "SERVO_IDX") {
        servoInternoDX.write(constrain(valore, 45, 135));
        inviaMessaggioAvionica("Servo interno DX comandato");
    }
    else if (campo == "SERVO_ESX") {
        servoEsternoSX.write(constrain(valore, 45, 135));
        inviaMessaggioAvionica("Servo esterno SX comandato");
    }
    else if (campo == "SERVO_EDX") {
        servoEsternoDX.write(constrain(valore, 45, 135));
        inviaMessaggioAvionica("Servo esterno DX comandato");
    }

    else if (campo == "SERVO_ISX_ATTACH") {
        servoInternoSX.attach(PIN_INT_SX);
        inviaMessaggioAvionica("Servo interno SX collegato");
    }
    else if (campo == "SERVO_IDX_ATTACH") {
        servoInternoDX.attach(PIN_INT_DX);
        inviaMessaggioAvionica("Servo interno DX collegato");
    }
    else if (campo == "SERVO_ESX_ATTACH") {
        servoEsternoSX.attach(PIN_EST_SX);
        inviaMessaggioAvionica("Servo esterno SX collegato");
    }
    else if (campo == "SERVO_EDX_ATTACH") {
        servoEsternoDX.attach(PIN_EST_DX);
        inviaMessaggioAvionica("Servo esterno DX collegato");
    }

    else if (campo == "SERVO_ISX_DETACH") {
        servoInternoSX.detach();
        inviaMessaggioAvionica("Servo interno SX scollegato");
    }
    else if (campo == "SERVO_IDX_DETACH") {
        servoInternoDX.detach();
        inviaMessaggioAvionica("Servo interno DX scollegato");
    }
    else if (campo == "SERVO_ESX_DETACH") {
        servoEsternoSX.detach();
        inviaMessaggioAvionica("Servo esterno SX scollegato");
    }
    else if (campo == "SERVO_EDX_DETACH") {
        servoEsternoDX.detach();
        inviaMessaggioAvionica("Servo esterno DX scollegato");
    }


    // ========================================================
    // RELE' E SICUREZZE
    // ========================================================

    else if (campo == "RELE_ON") {
        digitalWrite(PIN_RELE, HIGH);
        B_rele_attivato = true;
        inviaMessaggioAvionica("Rele' attivato");
    }
    else if (campo == "RELE_OFF") {
        digitalWrite(PIN_RELE, LOW);
        B_rele_attivato = false;
        inviaMessaggioAvionica("Rele' disattivato");
    }

    else if (campo == "SICUREZZA_SCHIANTO_ON") {
        B_schianto_sicurezza = true;
        inviaMessaggioAvionica("Sicurezza schianto attiva");
    }
    else if (campo == "SICUREZZA_SCHIANTO_OFF") {
        B_schianto_sicurezza = false;
        inviaMessaggioAvionica("Sicurezza schianto disattiva");
    }

    else if (campo == "SICUREZZA_ALIMENTAZIONE_ON") {
        B_alimentazione_sicurezza = true;
        inviaMessaggioAvionica("Sicurezza alimentazione attiva");
    }
    else if (campo == "SICUREZZA_ALIMENTAZIONE_OFF") {
        B_alimentazione_sicurezza = false;
        inviaMessaggioAvionica("Sicurezza alimentazione disattiva");
    }

    else if (campo == "SICUREZZA_SERVI_ON") {
        B_servo_sicurezza = true;
        inviaMessaggioAvionica("Sicurezza servi attiva");
    }
    else if (campo == "SICUREZZA_SERVI_OFF") {
        B_servo_sicurezza = false;
        inviaMessaggioAvionica("Sicurezza servi disattiva");
    }

    else if (campo == "SICUREZZA_TEMP_ON") {
        B_limitazione_termica_abilitata = true;
        inviaMessaggioAvionica("Protezione termica attiva");
    }
    else if (campo == "SICUREZZA_TEMP_OFF") {
        B_limitazione_termica_abilitata = false;
        inviaMessaggioAvionica("Protezione termica disattiva");
    }


    // ========================================================
    // MOTORE E MODALITA'
    // ========================================================

    else if (campo == "GAS") {
        // Comando pensato per prove manuali da banco.
        if (B_motore_disabilitato_da_terra) {
            inviaMessaggioAvionica("GAS rifiutato: motore disabilitato");
        }
        else if (G_modalita_volo != 1) {
            inviaMessaggioAvionica("GAS rifiutato: drone non in modalita' manuale");
        }
        else {
            int gas_us =
                constrain(valore, GAS_NEUTRO_us, GAS_MASSIMO_us);

            scriviMotore(gas_us);
            inviaMessaggioAvionica("Comando GAS eseguito");
        }
    }

    else if (campo == "MODO") {
        // Sono ammesse solo modalita' 1 e 2 e non si cambia modo in failsafe.
        if (!B_failsafe && valore >= 1 && valore <= 2) {
            G_modalita_volo = valore;
            resettaPID();
            inviaMessaggioAvionica("Modalita' di volo aggiornata");
        } else {
            inviaMessaggioAvionica("Cambio modalita' rifiutato");
        }
    }

    else if (campo == "STOP_MOTORE") {
        // Attiviamo il kill switch software.
        B_motore_disabilitato_da_terra = true;

        // Il motore viene portato immediatamente al neutro.
        scriviMotore(GAS_NEUTRO_us);

        inviaMessaggioAvionica("MOTORE DISABILITATO DA TERRA");
    }

    else if (campo == "RIPRISTINA_MOTORE") {
        B_motore_disabilitato_da_terra = false;
        inviaMessaggioAvionica("Motore ripristinato");
    }


    // ========================================================
    // TARGET DI NAVIGAZIONE
    // ========================================================

    else if (campo == "SET_LATITUDE") {
        float latitudine_deg = valoreStr.toFloat();

        if (latitudine_deg >= -90.0f && latitudine_deg <= 90.0f) {
            G_target_lat_deg = latitudine_deg;
            inviaMessaggioAvionica("Latitudine target aggiornata");
        } else {
            inviaMessaggioAvionica("Latitudine rifiutata: fuori limite");
        }
    }

    else if (campo == "SET_LONGITUDE") {
        float longitudine_deg = valoreStr.toFloat();

        if (longitudine_deg >= -180.0f && longitudine_deg <= 180.0f) {
            G_target_lon_deg = longitudine_deg;
            inviaMessaggioAvionica("Longitudine target aggiornata");
        } else {
            inviaMessaggioAvionica("Longitudine rifiutata: fuori limite");
        }
    }

    else if (campo == "SET_ALTITUDE") {
        float altitudine_m = valoreStr.toFloat();

        if (altitudine_m >= 0.0f && altitudine_m <= ALTITUDINE_MAX_m) {
            G_altitudine_target_m = altitudine_m;
            inviaMessaggioAvionica("Altitudine target aggiornata");
        } else {
            inviaMessaggioAvionica("Altitudine target rifiutata: fuori limite");
        }
    }

    else if (campo == "SET_VEL_CROCIERA") {
        float velocita_kmh = valoreStr.toFloat();

        if (velocita_kmh >= 20.0f && velocita_kmh <= 150.0f) {
            G_velocita_crociera_kmh = velocita_kmh;
            inviaMessaggioAvionica("Velocita' crociera aggiornata");
        } else {
            inviaMessaggioAvionica("Velocita' crociera rifiutata");
        }
    }

    else if (campo == "SET_VEL_AVVICINAMENTO") {
        float velocita_kmh = valoreStr.toFloat();

        if (velocita_kmh >= 15.0f && velocita_kmh <= 150.0f) {
            G_velocita_avvicinamento_kmh = velocita_kmh;
            inviaMessaggioAvionica("Velocita' avvicinamento aggiornata");
        } else {
            inviaMessaggioAvionica("Velocita' avvicinamento rifiutata");
        }
    }


    // ========================================================
    // CALIBRAZIONI
    // ========================================================

    else if (campo == "CALIBRA_IMU") {
        if (!calibrazioneConsentita()) {
            inviaMessaggioAvionica("Calibrazione IMU bloccata: drone in volo");
        }
        else if (calibraIMU()) {
            inviaMessaggioAvionica("Calibrazione IMU completata");
        }
        else {
            inviaMessaggioAvionica("Calibrazione IMU fallita o drone mosso");
        }
    }

    else if (campo == "CALIBRA_BARO") {
        if (!calibrazioneConsentita()) {
            inviaMessaggioAvionica("Calibrazione barometro bloccata: drone in volo");
        }
        else if (calibraBarometro()) {
            inviaMessaggioAvionica("Calibrazione barometro completata");
        }
        else {
            inviaMessaggioAvionica("Calibrazione barometro fallita");
        }
    }

    else if (campo == "CALIBRA_PITOT") {
        if (!calibrazioneConsentita()) {
            inviaMessaggioAvionica("Calibrazione Pitot bloccata: drone in volo");
        }
        else if (calibraPitot()) {
            inviaMessaggioAvionica("Calibrazione Pitot completata");
        }
        else {
            inviaMessaggioAvionica("Calibrazione Pitot fallita");
        }
    }

    else if (campo == "CALIBRA_POST_SCHIANTO") {
        if (calibraDopoSchianto()) {
            inviaMessaggioAvionica("Calibrazione post-schianto completata");
        }

 else {
            inviaMessaggioAvionica("Calibrazione post-schianto non consentita o fallita");
        }
    }


    // ========================================================
    // DIAGNOSTICA
    // ========================================================

    else if (campo == "REQ_DIAG") {
        B_forza_invio_diagnostica = true;
        inviaMessaggioAvionica("Diagnostica richiesta");
    }

    else if (campo == "RESET_PID") {
        resettaPID();
        inviaMessaggioAvionica("PID azzerati");
    }

    else {
        // I guadagni PID non sono modificabili da terra:
        // i loro valori esistono soltanto in config.h.
        inviaMessaggioAvionica("Comando sconosciuto o non consentito");
    }
}

// ============================================================
// SETUP MODULARE
// ============================================================

void setupSegnalazioni() {
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

    tone(PIN_BUZZER, 800, 100);  delay(150);
    tone(PIN_BUZZER, 1200, 100); delay(150);
    tone(PIN_BUZZER, 1600, 150); delay(300);
}

void setupComunicazioni() {
    Serial.begin(BAUD_RATE_DEBUG);

    Wire.begin();
    Wire.setClock(400000);
    Wire1.begin();
    Wire1.setClock(400000);

    ricevente.begin();
    TELEMETRIA.begin(BAUD_RATE_LORA);
    GPS_SERIAL.begin(BAUD_RATE_GPS);
    LIDAR_SERIAL.begin(BAUD_RATE_LIDAR);

    delay(100);
    while (LIDAR_SERIAL.available()) LIDAR_SERIAL.read();

    inviaMessaggioAvionica("     SISTEMA DRONE - AVVIO IN CORSO     ");
}

bool inizializzaFlussoOttico() {
    inviaMessaggioAvionica("[ ] Flusso Ottico PMW3901");

    B_PMW3901_INIZIALIZZATO = flussoOttico.begin();
    B_PMW3901_OK = B_PMW3901_INIZIALIZZATO;

    if (B_PMW3901_OK) {
        inviaMessaggioAvionica("[OK] Flusso Ottico PMW3901");
        segnalaOK();
    } else {
        inviaMessaggioAvionica("[WARN] Flusso Ottico PMW3901 non disponibile");
        segnalaErrore();
    }

    return B_PMW3901_OK;
}

bool inizializzaLidar() {
    inviaMessaggioAvionica("[ ] TF-Luna LIDAR");

    B_LIDAR_OK = false;
    unsigned long inizio_ms = millis();

    while (millis() - inizio_ms < TIMEOUT_INIT_LIDAR_ms) {
        aggiornaLidar();
        if (B_LIDAR_OK) break;
        delay(10);
    }

    if (B_LIDAR_OK) {
        B_LIDAR_INIZIALIZZATO = true;
        inviaMessaggioAvionica("[OK] TF-Luna LIDAR");
        segnalaOK();
    } else {
        inviaMessaggioAvionica("[WARN] TF-Luna LIDAR assente - continuo senza");
    }

    return B_LIDAR_OK;
}

bool inizializzaGPS() {
    inviaMessaggioAvionica("[ ] Modulo GPS");

    unsigned long inizio_ms = millis();

    while (millis() - inizio_ms < TIMEOUT_INIT_GPS_ms) {
        while (GPS_SERIAL.available() > 0) {
            gps.encode(GPS_SERIAL.read());
        }
    }

    aggiornaGPS();

    if (B_GPS_OK) {
        inviaMessaggioAvionica("[OK] Modulo GPS operativo, stato=" + String(G_errore_gps));
        segnalaOK();
    } else {
        inviaMessaggioAvionica("ERRORE GPS, stato=" + String(G_errore_gps));
        segnalaErrore();
    }

    return B_GPS_OK;
}

bool inizializzaIMU() {
    inviaMessaggioAvionica("[ ] IMU BNO055");

    if (!B_BNO055_INIZIALIZZATO) {
        if (!giroscopio.begin()) {
            B_BNO055_OK = false;
            inviaMessaggioAvionica("ERRORE IMU BNO055 (cavi I2C?)");
            segnalaErrore();
            return false;
        }

        giroscopio.setExtCrystalUse(true);
        B_BNO055_INIZIALIZZATO = true;
    }

    return calibraIMU();
}

bool calibraIMU() {
    if (!B_BNO055_INIZIALIZZATO) return false;

    bool b_stato_precedente_ok = B_BNO055_OK;
    float vecchioOffsetRoll_deg = G_offset_roll_deg;
    float vecchioOffsetPitch_deg = G_offset_pitch_deg;
    float vecchioOffsetYaw_deg = G_offset_yaw_deg;

    inviaMessaggioAvionica("Calibrazione IMU: tenere il drone fermo");

    uint8_t sys = 0, gyro = 0, accel = 0, mag = 0;
    unsigned long inizio_ms = millis();

    do {
        giroscopio.getCalibration(&sys, &gyro, &accel, &mag);
        delay(100);
        if (millis() - inizio_ms > TIMEOUT_CALIBRAZIONE_IMU_ms) break;
    } while (gyro < 2);

    double sommaRoll_deg = 0.0;
    double sommaPitch_deg = 0.0;
    double sommaYawSin = 0.0;
    double sommaYawCos = 0.0;

    float minRoll_deg = 10000.0f;
    float maxRoll_deg = -10000.0f;
    float minPitch_deg = 10000.0f;
    float maxPitch_deg = -10000.0f;

    for (int i = 0; i < IMU_CAMPIONI_TARA; i++) {
        sensors_event_t ev;
        giroscopio.getEvent(&ev);

        if (!isfinite(ev.orientation.x) ||
            !isfinite(ev.orientation.y) ||
            !isfinite(ev.orientation.z)) {
            B_BNO055_OK = b_stato_precedente_ok;
            digitalWrite(PIN_LED_BLU_PID, LOW);
            digitalWrite(PIN_BUZZER, LOW);
            inviaMessaggioAvionica("Calibrazione IMU annullata: lettura non valida");
            return false;
        }

        float roll_deg = ev.orientation.z;
        float pitch_deg = ev.orientation.y;

        sommaRoll_deg += roll_deg;
        sommaPitch_deg += pitch_deg;

        float yawRad = radians(ev.orientation.x);
        sommaYawSin += sin(yawRad);
        sommaYawCos += cos(yawRad);

        minRoll_deg = min(minRoll_deg, roll_deg);
        maxRoll_deg = max(maxRoll_deg, roll_deg);
        minPitch_deg = min(minPitch_deg, pitch_deg);
        maxPitch_deg = max(maxPitch_deg, pitch_deg);

        if (i % 20 == 0) segnalaCalibrazione(PIN_LED_BLU_PID);
        delay(10);
    }

    digitalWrite(PIN_LED_BLU_PID, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    bool b_drone_mosso =
        (maxRoll_deg - minRoll_deg > MAX_MOVIMENTO_CALIBRAZIONE_IMU_deg) ||
        (maxPitch_deg - minPitch_deg > MAX_MOVIMENTO_CALIBRAZIONE_IMU_deg);

    if (b_drone_mosso) {
        G_offset_roll_deg = vecchioOffsetRoll_deg;
        G_offset_pitch_deg = vecchioOffsetPitch_deg;
        G_offset_yaw_deg = vecchioOffsetYaw_deg;
        B_BNO055_OK = b_stato_precedente_ok;

        inviaMessaggioAvionica("Calibrazione IMU rifiutata: drone mosso durante la tara");
        segnalaErrore();
        return false;
    }

    G_offset_roll_deg = (float)(sommaRoll_deg / IMU_CAMPIONI_TARA);
    G_offset_pitch_deg = (float)(sommaPitch_deg / IMU_CAMPIONI_TARA);
    G_offset_yaw_deg = degrees(atan2(sommaYawSin, sommaYawCos));
    if (G_offset_yaw_deg < 0.0f) G_offset_yaw_deg += 360.0f;

    B_BNO055_OK = true;

    inviaMessaggioAvionica("IMU calibrata: roll=" + String(G_offset_roll_deg, 2) +
                           " pitch=" + String(G_offset_pitch_deg, 2) +
                           " yaw=" + String(G_offset_yaw_deg, 2));
    segnalaOK();
    return true;
}

bool inizializzaBarometro() {
    inviaMessaggioAvionica("[ ] Barometro BMP390");

    if (!B_BMP390_INIZIALIZZATO) {
        if (!barometro.begin_I2C()) {
            B_BMP390_OK = false;
            inviaMessaggioAvionica("ERRORE Barometro BMP390 (cavi I2C?)");
            segnalaErrore();
            return false;
        }

        barometro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        barometro.setPressureOversampling(BMP3_OVERSAMPLING_32X);
        barometro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        barometro.setOutputDataRate(BMP3_ODR_50_HZ);

        B_BMP390_INIZIALIZZATO = true;
        delay(100);

        for (int i = 0; i < 3; i++) {
            barometro.readAltitude(PRESSIONE_RIFERIMENTO_BARO_hPa);
            delay(25);
        }
    }

    return calibraBarometro();
}

bool calibraBarometro() {
    if (!B_BMP390_INIZIALIZZATO) return false;

    inviaMessaggioAvionica("Calibrazione barometro");

    float sommaAltitudine_m = 0.0f;

    for (int i = 0; i < BARO_CAMPIONI_TARA; i++) {
        float altitudineIstantanea_m = barometro.readAltitude(PRESSIONE_RIFERIMENTO_BARO_hPa);

        bool b_valida = isfinite(altitudineIstantanea_m) &&
                        altitudineIstantanea_m >= BARO_ALTITUDINE_MIN_PLAUSIBILE_m &&
                        altitudineIstantanea_m <= BARO_ALTITUDINE_MAX_PLAUSIBILE_m;

        if (!b_valida) {
            B_BMP390_OK = false;
            digitalWrite(PIN_LED_BLU_PID, LOW);
            inviaMessaggioAvionica("ERRORE: lettura barometrica impossibile");
            segnalaErrore();
            return false;
        }

        sommaAltitudine_m += altitudineIstantanea_m;
        segnalaCalibrazione(PIN_LED_BLU_PID);
        delay(25);
    }

    digitalWrite(PIN_LED_BLU_PID, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    float mediaBaroCalibrazione_m = sommaAltitudine_m / BARO_CAMPIONI_TARA;

    if (mediaBaroCalibrazione_m < 5.0f &&
        B_LIDAR_OK &&
        G_altitudine_lidar_m > 0.0f &&
        G_altitudine_lidar_m < 5.0f) {
        G_tara_altitudine_baro_m = mediaBaroCalibrazione_m - G_altitudine_lidar_m;
    } else {
        G_tara_altitudine_baro_m = mediaBaroCalibrazione_m;
    }

    B_BMP390_OK = true;
    inviaMessaggioAvionica("Barometro calibrato, tara=" + String(G_tara_altitudine_baro_m, 1) + " m");
    segnalaOK();
    return true;
}

bool inizializzaPitot() {
    B_PITOT_INIZIALIZZATO = true;
    return calibraPitot();
}

bool calibraPitot() {
    if (!B_PITOT_INIZIALIZZATO) return false;

    inviaMessaggioAvionica("Calibrazione pitot: nessun flusso d'aria sul sensore");

    long sommaLetture_adc = 0;

    for (int i = 0; i < PITOT_CAMPIONI_TARA; i++) {
        if (i % 10 == 0) segnalaCalibrazione(PIN_LED_VERDE_GPS);
        sommaLetture_adc += analogRead(PIN_ARIA);
        delay(10);
    }

    digitalWrite(PIN_LED_VERDE_GPS, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    float nuovoZero_adc = sommaLetture_adc / (float)PITOT_CAMPIONI_TARA;

    if (nuovoZero_adc <= PITOT_ADC_MIN_VALIDO || nuovoZero_adc >= PITOT_ADC_MAX_VALIDO) {
        B_PITOT_OK = false;
        inviaMessaggioAvionica("ERRORE Pitot: zero ADC anomalo " + String(nuovoZero_adc, 1));
        segnalaErrore();
        return false;
    }

    G_pitot_zero_adc = nuovoZero_adc;
    B_PITOT_OK = true;

    inviaMessaggioAvionica("Pitot calibrato, zero=" + String(G_pitot_zero_adc, 1));
    segnalaOK();
    return true;
}

bool inizializzaINA219() {
    inviaMessaggioAvionica("[ ] INA219 (6 sensori)");

    B_INA219_INIZIALIZZATO = true;

    B_INA219_MOTORE_OK = sensoreMotore.begin();
    B_INA219_TEENSY_OK = sensoreTeensy.begin();
    B_INA219_INT_SX_OK = sensoreIntSX.begin();
    B_INA219_INT_DX_OK = sensoreIntDX.begin();
    B_INA219_EST_SX_OK = sensoreEstSX.begin();
    B_INA219_EST_DX_OK = sensoreEstDX.begin();

    inviaMessaggioAvionica(String("INA219 Batteria motore: ") + (B_INA219_MOTORE_OK ? "OK" : "ERRORE"));
    inviaMessaggioAvionica(String("INA219 Batteria Teensy: ") + (B_INA219_TEENSY_OK ? "OK" : "ERRORE"));
    inviaMessaggioAvionica(String("INA219 Servo IntSX: ") + (B_INA219_INT_SX_OK ? "OK" : "ERRORE"));
    inviaMessaggioAvionica(String("INA219 Servo IntDX: ") + (B_INA219_INT_DX_OK ? "OK" : "ERRORE"));
    inviaMessaggioAvionica(String("INA219 Servo EstSX: ") + (B_INA219_EST_SX_OK ? "OK" : "ERRORE"));
    inviaMessaggioAvionica(String("INA219 Servo EstDX: ") + (B_INA219_EST_DX_OK ? "OK" : "ERRORE"));

    B_INA219_OK = B_INA219_MOTORE_OK && B_INA219_TEENSY_OK &&
                  B_INA219_INT_SX_OK && B_INA219_INT_DX_OK &&
                  B_INA219_EST_SX_OK && B_INA219_EST_DX_OK;

    if (B_INA219_OK) segnalaOK();
    else segnalaErrore();

    return B_INA219_OK;
}

bool sensoriCriticiOK() {
    return B_BNO055_OK &&
           B_BMP390_OK &&
           B_PITOT_OK &&
           B_INA219_OK &&
           B_GPS_OK;
}

void setupSensori() {
    inviaMessaggioAvionica("     INIZIALIZZAZIONE SENSORI     ");

    G_tentativi_init = 0;

    while (!sensoriCriticiOK() && G_tentativi_init < MAX_TENTATIVI_INIT) {
        G_tentativi_init++;
        inviaMessaggioAvionica("Tentativo inizializzazione " + String(G_tentativi_init));

        if (!B_PMW3901_INIZIALIZZATO) inizializzaFlussoOttico();
        if (!B_LIDAR_OK) inizializzaLidar();
        if (!B_GPS_OK) inizializzaGPS();
        if (!B_BNO055_OK) inizializzaIMU();
        if (!B_BMP390_OK) inizializzaBarometro();
        if (!B_PITOT_OK) inizializzaPitot();
        if (!B_INA219_OK) inizializzaINA219();

        if (!sensoriCriticiOK() && G_tentativi_init < MAX_TENTATIVI_INIT) {
            inviaMessaggioAvionica("Sensori critici mancanti. Nuovo tentativo...");
            digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
            delay(TEMPO_RITENTO_SENSORI_ms);
            digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        }
    }

    bloccaAvvioSeSensoriCriticiKO();

    inviaMessaggioAvionica("Sensori critici inizializzati correttamente");
}

void bloccaAvvioSeSensoriCriticiKO() {
    if (sensoriCriticiOK()) return;

    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    inviaMessaggioAvionica("[FATAL ERROR] Fallimento inizializzazione hardware critico. Sistema bloccato.");

    while (1) {
        tone(PIN_BUZZER, 2000, 300);
        delay(400);
    }
}

void setupAttuatori() {
    inizializzaServo();
    inizializzaMotore();
}

void finalizzaSetup() {
    tone(PIN_BUZZER, 800, 120);  delay(170);
    tone(PIN_BUZZER, 1200, 120); delay(170);
    tone(PIN_BUZZER, 1800, 200); delay(350);

    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    digitalWrite(PIN_LED_BLU_PID, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
    digitalWrite(PIN_LED_BLU_PID, LOW);

    G_tempo_pid_precedente_ms = millis();
    G_tempo_batteria_precedente_ms = millis();

    inviaMessaggioAvionica("SISTEMA PRONTO AL VOLO");
    delay(500);
}

bool calibrazioneConsentita() {
    // In volo normale la calibrazione e' bloccata.
    // Dopo uno schianto e con sistema bloccato e' consentita per il recupero.
    return !B_drone_in_volo || B_schianto_bloccato;
}

bool calibraDopoSchianto() {
    if (!calibrazioneConsentita()) return false;

    scriviMotore(GAS_NEUTRO_us);

    bool b_imu_ok = calibraIMU();
    bool b_baro_ok = calibraBarometro();
    bool b_pitot_ok = calibraPitot();

    resettaPID();

    return b_imu_ok && b_baro_ok && b_pitot_ok;
}

void setup() {
    setupComunicazioni();
    setupSegnalazioni();
    setupSensori();
    setupAttuatori();
    finalizzaSetup();
}

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


// Collega l'ESC alla Teensy e lo porta subito al comando neutro.
void inizializzaMotore() {
    inviaMessaggioAvionica("Inizializzazione motore in corso...");

    // Colleghiamo l'oggetto Servo al pin dell'ESC.
    motore.attach(PIN_MOTORE);

    // Tutte le scritture fisiche del motore passano da scriviMotore().
    scriviMotore(GAS_NEUTRO_us);

    inviaMessaggioAvionica("Motore inizializzato al neutro");
}


// UNICO punto del programma che chiama motore.writeMicroseconds().
// Se in futuro vuoi aggiungere un clamp, un log o un controllo hardware,
// lo devi fare soltanto qui.
void scriviMotore(int gas_us) {
    // Il comando non puo' uscire dal range previsto dall'ESC.
    int gasLimitato_us =
        constrain(gas_us, GAS_NEUTRO_us, GAS_MASSIMO_us);

    // Questa e' l'unica scrittura fisica verso l'ESC.
    motore.writeMicroseconds(gasLimitato_us);
}

// ============================================================
// STATO SENSORI E LOOP MODULARE
// ============================================================

void inviaDiagnosticaSensori() {
    inviaMessaggioAvionica(
        "SENSORI: IMU=" + String(B_BNO055_OK ? "OK" : "KO") +
        " BARO=" + String(B_BMP390_OK ? "OK" : "KO") +
        " LIDAR=" + String(B_LIDAR_OK ? "OK" : "KO") +
        " OTTICO=" + String(B_PMW3901_OK ? "OK" : "KO") +
        " PITOT=" + String(B_PITOT_OK ? "OK" : "KO") +
        " GPS=" + String(B_GPS_OK ? "OK" : "KO") +
        " INA219=" + String(B_INA219_OK ? "OK" : "KO")
    );
}

void aggiornaStatoSensori() {
    static bool b_prima_esecuzione = true;

    static bool b_imu_precedente = false;
    static bool b_baro_precedente = false;
    static bool b_lidar_precedente = false;
    static bool b_ottico_precedente = false;
    static bool b_pitot_precedente = false;
    static bool b_gps_precedente = false;
    static bool b_ina_precedente = false;

    B_INA219_OK = B_INA219_MOTORE_OK && B_INA219_TEENSY_OK &&
                  B_INA219_INT_SX_OK && B_INA219_INT_DX_OK &&
                  B_INA219_EST_SX_OK && B_INA219_EST_DX_OK;

    if (b_prima_esecuzione) {
        b_imu_precedente = B_BNO055_OK;
        b_baro_precedente = B_BMP390_OK;
        b_lidar_precedente = B_LIDAR_OK;
        b_ottico_precedente = B_PMW3901_OK;
        b_pitot_precedente = B_PITOT_OK;
        b_gps_precedente = B_GPS_OK;
        b_ina_precedente = B_INA219_OK;
        b_prima_esecuzione = false;
    } else {
        if (B_BNO055_OK != b_imu_precedente)
            inviaMessaggioAvionica(B_BNO055_OK ? "IMU tornata operativa" : "ATTENZIONE: IMU non affidabile");
        if (B_BMP390_OK != b_baro_precedente)
            inviaMessaggioAvionica(B_BMP390_OK ? "Barometro tornato operativo" : "ATTENZIONE: barometro non affidabile");
        if (B_LIDAR_OK != b_lidar_precedente)
            inviaMessaggioAvionica(B_LIDAR_OK ? "LIDAR tornato operativo" : "ATTENZIONE: LIDAR non affidabile");
        if (B_PMW3901_OK != b_ottico_precedente)
            inviaMessaggioAvionica(B_PMW3901_OK ? "Flusso ottico operativo" : "ATTENZIONE: flusso ottico non disponibile");
        if (B_PITOT_OK != b_pitot_precedente)
            inviaMessaggioAvionica(B_PITOT_OK ? "Pitot tornato operativo" : "ATTENZIONE: Pitot non affidabile");
        if (B_GPS_OK != b_gps_precedente)
            inviaMessaggioAvionica(B_GPS_OK ? "GPS tornato operativo" : "ATTENZIONE: GPS non affidabile");
        if (B_INA219_OK != b_ina_precedente)
            inviaMessaggioAvionica(B_INA219_OK ? "INA219 tutti operativi" : "ATTENZIONE: uno o piu INA219 non affidabili");

        b_imu_precedente = B_BNO055_OK;
        b_baro_precedente = B_BMP390_OK;
        b_lidar_precedente = B_LIDAR_OK;
        b_ottico_precedente = B_PMW3901_OK;
        b_pitot_precedente = B_PITOT_OK;
        b_gps_precedente = B_GPS_OK;
        b_ina_precedente = B_INA219_OK;
    }

    if (B_forza_invio_diagnostica) {
        inviaDiagnosticaSensori();
        B_forza_invio_diagnostica = false;
    }
}

void gestisciInizioCiclo() {
    comandiDaTerra();
    gestisciAlimentazione();
    gestisciSchianto();
    verificaDroneInVolo();
}


// Aggiorna tutti i sensori nell'ordine corretto.
// Questa funzione non fa controlli di volo: si occupa soltanto di acquisire
// e preparare i dati che verranno usati dopo.
void aggiornaSensori() {
    // Prima svuotiamo la seriale GPS nel parser TinyGPS++.
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    // Controlliamo lo stato dei quattro servi.
    diagnosticaServi();

    // Leggiamo l'assetto e aggiorniamo G_pitch/G_roll/G_yaw.
    leggiIMU();

    // Leggiamo lo stato di calibrazione interna del BNO055.
    aggiornaDiagnosticaIMU();

    // Sensori di temperatura.
    leggiTemperatura();

    // GPS.
    aggiornaGPS();

    // Ordine importante:
    // 1. barometro
    // 2. densita' aria
    // 3. Pitot
    // Il Pitot ha bisogno della densita' dell'aria aggiornata.
    leggiBarometro();
    aggiornaDensitaAria();
    leggiPitot();

    // Quota.
    aggiornaLidar();
    selezionaAltitudine();
    aggiornaVelocitaVerticale();

    // Velocita' al suolo.
    leggiVelocitaOttica();
    aggiornaVelocitaSuolo();
}


// Sceglie velocita' target e gas base in funzione della distanza dal waypoint.
// I risultati vengono salvati nello stato globale del controllo.
void scegliTargetVelocita() {
    // Memorizza la fascia precedente soltanto per evitare messaggi ripetuti.
    static int fasciaPrecedente = -1;

    // Target lontano: crociera.
    if (G_distanza_target_m > DISTANZA_FRENATA_m) {
        G_target_velocita_kmh = G_velocita_crociera_kmh;
        G_gas_base_us = GAS_CROCIERA_us;

        if (fasciaPrecedente != 1) {
            inviaMessaggioAvionica("Target lontano: velocita' crociera");
            fasciaPrecedente = 1;
        }

        return;
    }

    // Target vicino: avvicinamento.
    G_target_velocita_kmh = G_velocita_avvicinamento_kmh;
    G_gas_base_us = GAS_AVVICINAMENTO_us;

    if (fasciaPrecedente != 0) {
        inviaMessaggioAvionica("Target vicino: velocita' avvicinamento");
        fasciaPrecedente = 0;
    }
}

void aggiornaModalitaVoloDaRadio() {
    bool b_pacchetto_perso = false;

    if (!ricevente.read(&G_canali_rc[0], &B_failsafe, &b_pacchetto_perso)) return;

    if (B_stato_schianto_rilevato && G_canali_rc[4] < 992) {
        B_stato_schianto_rilevato = false;
        B_schianto_bloccato = false;
        B_drone_in_volo = false;

        inizializzaServo();

        noTone(PIN_BUZZER);
        tone(PIN_BUZZER, 1000, 100);
        delay(150);
        tone(PIN_BUZZER, 1500, 100);

        inviaMessaggioAvionica("SBLOCCO EMERGENZA ESEGUITO DA RADIO. Servi riarmati e centrati.");
        inviaMessaggioAvionica("Dopo uno schianto e' possibile eseguire CALIBRA_POST_SCHIANTO da terra.");
    }

    if (G_canali_rc[4] < 992) {
        G_modalita_volo = 1;
    } else if (B_drone_in_volo && !B_stato_schianto_rilevato) {
        G_modalita_volo = 2;
    }
}


// Decide chi comanda il drone in questo ciclo:
// manuale, PID automatico oppure protezione dell'inviluppo di volo.
void calcolaComandiVolo() {
    // Aggiorniamo velocita' target e gas base.
    scegliTargetVelocita();

    // Partiamo sempre da comandi neutri.
    G_comando_pitch_deg = 0;
    G_comando_roll_deg = 0;
    G_comando_gas_us = GAS_NEUTRO_us;

    // Stato 3 viene usato solo internamente per distinguere il failsafe.
    int statoAttuale =
        B_failsafe ? 3 : G_modalita_volo;

    // Quando cambia modalita' cancelliamo la memoria dei PID.
    static int statoPrecedente = -1;

    if (statoAttuale != statoPrecedente) {
        resettaPID();
        inviaMessaggioAvionica("Cambio modalita' di volo: PID azzerati");
        statoPrecedente = statoAttuale;
    }


    // ========================================================
    // VOLO MANUALE
    // ========================================================

    if (!B_schianto_bloccato &&
        G_modalita_volo == 1 &&
        !B_failsafe) {

        // Gas dal radiocomando.
        G_comando_gas_us =
            constrain(
                map(G_canali_rc[2],
                    172, 1811,
                    GAS_NEUTRO_us, GAS_MASSIMO_us),
                GAS_NEUTRO_us,
                GAS_MASSIMO_us
            );

        // Roll dal radiocomando.
        G_comando_roll_deg =
            constrain(
                map(G_canali_rc[0],
                    172, 1811,
                    -MAX_ROLL_deg, MAX_ROLL_deg),
                -MAX_ROLL_deg,
                MAX_ROLL_deg
            );

        // Pitch dal radiocomando.
        G_comando_pitch_deg =
            constrain(
                map(G_canali_rc[1],
                    172, 1811,
                    MAX_PITCH_deg, -MAX_PITCH_deg),
                -MAX_PITCH_deg,
                MAX_PITCH_deg
            );
    }


    // ========================================================
    // VOLO AUTOMATICO / FAILSAFE
    // ========================================================

    else if (!B_schianto_bloccato &&
             (G_modalita_volo == 2 || B_failsafe) &&
             (B_drone_in_volo || B_failsafe)) {

        // Prima del PID controlliamo l'inviluppo di volo.
        int protezione = verificaProtezioniVolo();

        // Se tutto e' normale usiamo il PID classico.
        if (protezione == PROTEZIONE_NESSUNA) {
            calcolaPID();
        }
        // Se esiste una condizione critica la safety prende il controllo.
        else {
            calcolaComandiProtezione(protezione);
        }
    }


    // ========================================================
    // KILL SWITCH SOFTWARE
    // ========================================================

    // Il kill switch ha priorita' su qualsiasi comando calcolato sopra.
    if (B_motore_disabilitato_da_terra) {
        G_comando_gas_us = GAS_NEUTRO_us;
    }
}


// Applica realmente i comandi a servi e motore.
// Tutti i limiti finali vengono applicati qui, immediatamente prima dell'hardware.
void aggiornaAttuatori() {
    // Dopo uno schianto il motore deve restare neutro.
    if (B_stato_schianto_rilevato) {
        G_comando_gas_us = GAS_NEUTRO_us;
        scriviMotore(G_comando_gas_us);
        return;
    }

    // Aggiorniamo LED/allarmi.
    gestisciAllarmi();

    // Il mixer trasforma pitch/roll nei quattro angoli servo.
    applicaMixer4Servi();

    // Calcoliamo il massimo gas consentito dalle temperature.
    G_gas_limite_termico_us = gasMaxTermico();

    // Stato precedente soltanto per non ripetere il messaggio.
    static bool b_limitazione_precedente = false;

    // Se la protezione termica e' abilitata, limitiamo il gas.
    if (B_limitazione_termica_abilitata &&
        G_comando_gas_us > G_gas_limite_termico_us) {

        G_comando_gas_us = G_gas_limite_termico_us;
        B_limitazione_termica_attiva = true;
    } else {
        B_limitazione_termica_attiva = false;
    }

    // Messaggio soltanto quando la limitazione entra o esce.
    if (B_limitazione_termica_attiva != b_limitazione_precedente) {
        inviaMessaggioAvionica(
            B_limitazione_termica_attiva
                ? "Limitazione termica gas ATTIVA"
                : "Limitazione termica gas rientrata"
        );

        b_limitazione_precedente = B_limitazione_termica_attiva;
    }

    // UNICA chiamata che porta il comando finale all'ESC.
    scriviMotore(G_comando_gas_us);
}


// Il loop descrive soltanto l'ordine logico del flight controller.
// I dettagli sono nascosti nelle funzioni dedicate.
void loop() {
    // Comandi da terra, alimentazione, schianto e stato volo.
    gestisciInizioCiclo();

    // Acquisizione di tutti i sensori.
    aggiornaSensori();

    // Aggiorna i flag B_SENSOR_OK.
    aggiornaStatoSensori();

    // Elaborazioni derivate dai sensori.
    stimaVento();
    aggiornaNavigazione();

    // Legge il radiocomando e decide manuale/automatico.
    aggiornaModalitaVoloDaRadio();

    // Calcola pitch, roll e gas.
    calcolaComandiVolo();

    // Scrive servi e motore.
    aggiornaAttuatori();

    // Invia lo stato alla stazione di terra.
    inviaTelemetria();
}

