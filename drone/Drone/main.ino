#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "SBUS.h"
#include <Adafruit_INA219.h>
#include <Adafruit_BMP3XX.h>

// ============================================================
//  SENSORI
// ============================================================
TinyGPSPlus gps;
Adafruit_BNO055  giroscopio = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX barometro;

const int PIN_T_motore=A1;
const int PIN_T_teensy=A2;
float Global_Temperatura_motore = 0.0;
float Global_Temperatura_teensy = 0.0;

// ============================================================
//  CONFIGURAZIONE GPS
// ============================================================
#define GPS_SERIAL    Serial1
#define BAUD_RATE_GPS 9600
// ============================================================
//  TELEMETRIA LORA 
// ============================================================
#define TELEMETRIA Serial4         
unsigned long timerTelemetria = 0;
// ============================================================
//  RICEVENTE SBUS (FrSky)
// ============================================================
SBUS     ricevente(Serial3);
uint16_t canaliRC[16];
bool failsafe       = false;
bool pacchettoPerso = false;
// ============================================================
//  PITOT (VELOCITÀ ARIA)
// ============================================================
const int   PIN_ARIA    = A0;
const float DENSITA_ARIA = 1.225;  // kg/m³
float       VALORE_ZERO  = 0.0;    // calibrazione pitot
// ============================================================
//  SERVO
// ============================================================
const int pinIntSX  = 6;
const int pinIntDX  = 7;
const int pinEstSX  = 8;
const int pinEstDX  = 9;
const int pinMotore = 10;

Servo servoInternoSX;  // flap interno sinistro  (pitch)
Servo servoInternoDX;  // flap interno destro    (pitch)
Servo servoEsternoSX;  // flap esterno sinistro  (pitch + roll)
Servo servoEsternoDX;  // flap esterno destro    (pitch + roll)
Servo motore;
// ============================================================
//  SENSORE CORRENTE
// ============================================================
Adafruit_INA219 sensoreBatt(0x40);  // Batteria principale
Adafruit_INA219 sensoreIntSX(0x41);
Adafruit_INA219 sensoreIntDX(0x42);
Adafruit_INA219 sensoreEstSX(0x43);
Adafruit_INA219 sensoreEstDX(0x44);
// ============================================================
//  COSTANTI
// ============================================================
const int CENTRO      = 90;
const int MAX_ROLL    = 35;
const int MAX_PITCH   = 20;
const int ALTEZZA_MAX = 120;
const int ALTEZZA_MIN = 10;
const int GAS_MINIMO   = 25;
const int GAS_CROCIERA = 70;   // regime di crociera
const int GAS_MASSIMO  = 130;  // massima apertura
const float VELOCITA_CROCIERA      = 60.0;   // km/h
const float VELOCITA_AVVICINAMENTO = 45.0;   // km/h (sopra la velocità di stallo)
const float DISTANZA_FRENATA       = 150.0;  // m — distanza per iniziare a rallentare
// ============================================================
//  NAVIGAZIONE
// ============================================================
const double TARGET_LAT        = 41.902782;
const double TARGET_LON        = 12.496366;
const float  ALTITUDINE_TARGET = 40.0;

float global_altitudineDiPartenza = 0.0;
float global_targetRoll           = 0.0;
float global_distanzaDalTarget    = 0.0;
float global_rottaVersoTarget     = 0.0;
float global_errore_rotta         = 0.0;
float global_altitudineDalSuolo   = 0.0;
float latPrecedente = 0.0;
float lonPrecedente = 0.0;
float velocitaGPS   = 0.0;
float alpha_vel     = 0.7; // 70% fiducia nel Pitot, 30% nel GPS
float Velocita_stimata_Ms = 0.0;
// ============================================================
//  PID — GUADAGNI
// ============================================================
float Kp_vel = 1.5;
float Ki_vel = 0.1;
float Kd_vel = 0.5;

float Kp_roll = 1.2;
float Ki_roll = 0.05;
float Kd_roll = 0.5;

float Kp_pitch = 1.2;   
float Ki_pitch = 0.05;
float Kd_pitch = 0.5;

float Kp_alt = 0.5;
float Ki_alt = 0.05;
float Kd_alt = 0.2;

unsigned long tempoPassatoPID = 0;

// ============================================================
//  VARIABILI DI STATO PID GLOBALI
// ============================================================
float pid_sommaErroriAlt     = 0.0;
float pid_errorePassatoAlt   = 0.0;

float pid_sommaErroriPitch   = 0.0;
float pid_errorePassatoPitch = 0.0;

float pid_sommaErroriRoll    = 0.0;
float pid_errorePassatoRoll  = 0.0;

float pid_sommaErroriVel     = 0.0;
float pid_errorePassatoVel   = 0.0;

// ============================================================
//  LED DI STATO E ALLARMI
// ============================================================
const int PIN_LED_ROSSO_ALARM = 2; // Allarmi / Batteria
const int PIN_LED_VERDE_GPS = 3; // GPS Fix
const int PIN_LED_BLU_PID   = 4; // Modalità AUTO (PID)

// ============================================================
//  FLAGS STATO SISTEMA
// ============================================================
bool imuPronto      = false;
bool baroPronto     = false;
bool pitotCalibrato = false;
bool Voltaggio      = true;
int  tentativi      = 0;
const int MAX_TENTATIVI = 3;

// batteria
bool  batteriaBassa          = false;
bool  batteriaBassaPrecedente = false;
float global_fattoreReattivita = 1.0;

// Stati di salute dei servi
bool estSX_Ok        = true;
bool estDX_Ok        = true;
bool intSX_Ok        = true;
bool intDX_Ok        = true;
bool interni_staccati = false;
int  global_modalitaVolo = 1;
bool statoPrecedenteInterni = true;
bool statoPrecedenteEsterni = true;

// ============================================================
//  PROTOTIPI
// ============================================================
void applicaMixer4Servi(int pitch, int roll);
void aggiornaNavigazione(float angoloYaw);
void diagnosticaServi();
void inviaTelemetria(float pitch, float roll, float yaw,  
                     float velPitotKmh, float velGpsKmh, float velStimataKmh,
                     int outPitch, int outRoll, int outGas);
void calcolaPID(float targetAltitudine, float targetRoll,
                float pitchReale, float rollReale,
                float velocitaAttuale, float targetVelocita,
                int gasDiBase,
                int &comandoPitchOut, int &comandoRollOut, int &comandoGasOut);
void gestisciLuci();

// ============================================================
//  SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    Serial.println("In attesa di segnale dal Radiocomando FrSky...");
    ricevente.begin();

    Serial.println("\n=== INIZIALIZZAZIONE SISTEMA DRONE ===");

    Serial.println("Inizializzazione LoRa per Telemetria...");
    TELEMETRIA.begin(9600); 
    Serial.println("Radio LoRa attivata su Serial4");

    Serial.println("Inizializzazione GPS...");
    GPS_SERIAL.begin(BAUD_RATE_GPS);
    Serial.println("GPS: Seriale aperta, in attesa di satelliti...");



    while ((!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) && tentativi < MAX_TENTATIVI) {
        Serial.println("\n--- Controllo Sensori in corso... ---");
        Serial.print("\nTentativo numero: ");
        Serial.println(tentativi + 1);
        tentativi++;

        // 1. controllo giroscopio (IMU)
        if (!imuPronto) {
            Serial.print("IMU (BNO055)....... ");
            if (giroscopio.begin()) {
                giroscopio.setExtCrystalUse(true);
                imuPronto = true;
                Serial.println("OK! (Calibrato)");
            } else {
                Serial.println("FALLITO! (Controllo cavi I2C)");
            }
        } else {
            Serial.println("IMU (BNO055)....... [GIA' OK]");
        }

        // 2. controllo barometro
        if (!baroPronto) {
            Serial.print("Barometro (BMP390).. ");
            if (barometro.begin_I2C()) {
                barometro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
                barometro.setPressureOversampling(BMP3_OVERSAMPLING_32X);
                barometro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
                barometro.setOutputDataRate(BMP3_ODR_50_HZ); 
                delay(100); // stabilizzare il sensore
                //tara 
                global_altitudineDiPartenza = barometro.readAltitude(1013.25);
                baroPronto = true;
                Serial.print("OK! (Tara: ");
                Serial.print(global_altitudineDiPartenza);
                Serial.println("m)");
            } else {
                Serial.println("FALLITO! (Controllo cavi I2C)");
            }
        } else {
            Serial.println("Barometro (BMP390).. [GIA' OK]");
        }

        // 3. calibrazione pitot
        if (!pitotCalibrato) {
            Serial.print("Pitot (Velocita')... ");
            long sommaLetture = 0;
            for (int i = 0; i < 100; i++) {
                sommaLetture += analogRead(PIN_ARIA);
                delay(5);
            }
            VALORE_ZERO = sommaLetture / 100.0;

            if (VALORE_ZERO > 5 && VALORE_ZERO < 1020) {
                pitotCalibrato = true;
                Serial.print("OK! (Zero a: ");
                Serial.print(VALORE_ZERO);
                Serial.println(")");
            } else {
                Serial.println("ANOMALIA! (valore fuori range)");
            }
        } else {
            Serial.println("Pitot (Velocita')... [GIA' OK]");
        }

        // 4. controllo sensori di corrente (INA219)
        Voltaggio = true;
        if (!sensoreBatt.begin()) {
            Serial.println("ERRORE: INA219 Batteria non trovato");
            Voltaggio = false;
        }
        if (!sensoreIntSX.begin()) {
            Serial.println("ERRORE: INA219 IntSX non trovato");
            Voltaggio = false;
        }
        if (!sensoreIntDX.begin()) {
            Serial.println("ERRORE: INA219 IntDX non trovato");
            Voltaggio = false;
        }
        if (!sensoreEstSX.begin()) {
            Serial.println("ERRORE: INA219 EstSX non trovato");
            Voltaggio = false;
        }
        if (!sensoreEstDX.begin()) {
            Serial.println("ERRORE: INA219 EstDX non trovato");
            Voltaggio = false;
        }

        if (!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) {
            Serial.println("ATTENZIONE: Sensori mancanti o errati. Ritento tra 2 secondi...");
            delay(2000);
        }
    }

    if (!imuPronto || !baroPronto || !pitotCalibrato) {
        Serial.println("\nERRORE CRITICO");
        Serial.println("AVVIO BLOCCATO. Controllare l'hardware.");
        while (1);
    }

    Serial.println("\n=== TUTTI I SENSORI OPERATIVI! ===");
    Serial.println("Inizializzazione Servomotori...");
    servoInternoSX.attach(pinIntSX);
    servoInternoDX.attach(pinIntDX);
    servoEsternoSX.attach(pinEstSX);
    servoEsternoDX.attach(pinEstDX);
    Serial.print("Inizializzazione servo del motore... ");
    motore.attach(pinMotore);
    Serial.println("Tutti i servomotori pronti");

    Serial.println("Settando i flap dritti e motore a zero...");
    servoInternoSX.write(CENTRO);
    servoInternoDX.write(CENTRO);
    servoEsternoSX.write(CENTRO);
    servoEsternoDX.write(CENTRO);
    motore.write(GAS_MINIMO);

    Serial.println("Inizializzazione LED di stato...");
    pinMode(PIN_LED_ROSSO_ALARM, OUTPUT);
    pinMode(PIN_LED_VERDE_GPS, OUTPUT);
    pinMode(PIN_LED_BLU_PID, OUTPUT);
    
    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    digitalWrite(PIN_LED_BLU_PID, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
    digitalWrite(PIN_LED_BLU_PID, LOW);

    tempoPassatoPID = millis();

    Serial.println("SISTEMA PRONTO");
    delay(1000);
}

// ============================================================
//  LOOP
// ============================================================
void loop()
{
    // 1. lettura GPS
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    // monitoraggio voltaggio
    float vBatt = sensoreBatt.getBusVoltage_V();
    if (vBatt < 11.1) {
        global_fattoreReattivita = 0.6;
        batteriaBassa = true;
        Serial.println("BATTERIA BASSA: Risparmio energetico attivo.");
    } else {
        global_fattoreReattivita = 1.0;
        batteriaBassa = false;
    }

    // FIX typo: diagrnosticaServi → diagnosticaServi
    diagnosticaServi();

    // 2. lettura giroscopio (IMU)
    sensors_event_t event;
    giroscopio.getEvent(&event);
    float angoloPitch = event.orientation.y;
    float angoloRoll  = event.orientation.z;
    float angoloYaw   = event.orientation.x;

    // 3. aggiorna navigazione
    aggiornaNavigazione(angoloYaw);

    // 4. lettura barometro
    float altitudineAttuale  = barometro.readAltitude(1013.25);
    global_altitudineDalSuolo = altitudineAttuale - global_altitudineDiPartenza;

    // controlloo temperatura
    float voltaggio_Sensore_motore = analogRead(PIN_T_motore) * (3.3 / 1023.0);
    Global_Temperatura_motore = (voltaggio_Sensore_motore - 0.5) * 100.0;
    float voltaggio_Sensore_teensy = analogRead(PIN_T_teensy) * (3.3 / 1023.0);
    Global_Temperatura_teensy = (voltaggio_Sensore_teensy - 0.5) * 100.0;

    // 5. pitot – lettura velocità aria
    float Velocita_pitot_Ms=0.0;
    int lettura_dal_pin_pitot= analogRead(PIN_ARIA);
    float differenza= lettura_dal_pin_pitot - VALORE_ZERO;
    if (differenza < 0) {
        differenza = 0;
    }
    float pressionePascal = differenza * 1.5;
    Velocita_pitot_Ms= sqrt((2.0 * pressionePascal) / DENSITA_ARIA);

    //6. gps calcolo velocità (groundspeed)
    float velocita_gps_Ms = 0.0;
    if (gps.speed.isValid()) {
        velocita_gps_Ms = gps.speed.kmph() / 3.6;
        if (latPrecedente == 0.0 && lonPrecedente == 0.0) {
            latPrecedente = gps.location.lat();
            lonPrecedente = gps.location.lng();
        }
    }
    Velocita_stimata_Ms = (alpha_vel * Velocita_pitot_Ms) + ((1.0 - alpha_vel) * velocita_gps_Ms);

    // 6. PREPARAZIONE DATI MOTORE
    float targetVelocita = 0.0;
    int gasDiBase = 0;
    if (global_distanzaDalTarget > DISTANZA_FRENATA) {
        targetVelocita = VELOCITA_CROCIERA;
        gasDiBase= GAS_CROCIERA;
    } else {
        targetVelocita = VELOCITA_AVVICINAMENTO;
        gasDiBase= 45;
    }

    int correzionePitch  = 0;
    int correzioneRoll   = 0;
    int comandoGasFinale = 0;

    // 1 = Manuale, 2 = Auto, 3 = Failsafe
    if (ricevente.read(&canaliRC[0], &failsafe, &pacchettoPerso)) {
        if (canaliRC[4] < 992) {
            global_modalitaVolo = 1; // Levetta in basso -> Volo Manuale
        } else {
            global_modalitaVolo = 2; // Levetta in alto -> Volo Automatico (PID)
        }
    }
    
    int statoAttuale;
    static int ultimoStatoStampato = 0;
    if(failsafe){
        statoAttuale = 3;
    } else {
        statoAttuale = global_modalitaVolo;
    }

    if (global_modalitaVolo == 1 && !failsafe) {
        // Gas: da 172-1811 a GAS_MINIMO-GAS_MASSIMO (limiti meccanici motore)
        comandoGasFinale = constrain(map(canaliRC[2], 172, 1811, GAS_MINIMO, GAS_MASSIMO), GAS_MINIMO, GAS_MASSIMO);
        correzioneRoll   = constrain(map(canaliRC[0], 172, 1811, -MAX_ROLL,   MAX_ROLL),   -MAX_ROLL,  MAX_ROLL);
        correzionePitch  = constrain(map(canaliRC[1], 172, 1811,  MAX_PITCH, -MAX_PITCH),  -MAX_PITCH, MAX_PITCH);
        //stampa stato solo se è cambiato (per evitare spam in console)
        if (statoAttuale != ultimoStatoStampato) {
            Serial.println("Volo: MANUALE (Comandi diretti dal radiocomando)");
            ultimoStatoStampato = statoAttuale;
        }
    } else if (global_modalitaVolo == 2 || failsafe) {
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
        calcolaPID(ALTITUDINE_TARGET, global_targetRoll,angoloPitch, angoloRoll,Velocita_stimata_Ms *3.6, targetVelocita,gasDiBase,correzionePitch, correzioneRoll, comandoGasFinale);
    }
    gestisciLuci();
    applicaMixer4Servi(correzionePitch, correzioneRoll);
    motore.write(comandoGasFinale);

    if (millis() > 10000 && gps.charsProcessed() < 10) {
        Serial.println("ATTENZIONE: Nessun dato dal GPS. Controlla i cavi TX e RX!");
    }

    // Invia il pacchetto scatola nera via LoRa (Convertendo le velocità in km/h)
    inviaTelemetria(
    angoloPitch, angoloRoll, angoloYaw,
    Velocita_pitot_Ms * 3.6f,
    velocita_gps_Ms   * 3.6f,
    Velocita_stimata_Ms * 3.6f,
    correzionePitch, correzioneRoll, comandoGasFinale);
    delay(50);
}

// ============================================================
//  MIXER 4 SERVI
// ============================================================
void applicaMixer4Servi(int pitch, int roll)
{
    int posIntSX = CENTRO;
    int posIntDX = CENTRO;
    int posEstSX = CENTRO;
    int posEstDX = CENTRO;

    bool esterniAttivi = estSX_Ok && estDX_Ok && !batteriaBassa;
    bool interniAttivi = intSX_Ok && intDX_Ok;

    if (esterniAttivi && interniAttivi) {
        // Caso C: tutto OK → funzionamento standard
        posIntSX = CENTRO + pitch;
        posIntDX = CENTRO + pitch;
        posEstSX = CENTRO + pitch + roll;
        posEstDX = CENTRO + pitch - roll;
    } else if (!esterniAttivi) {
        // Caso A: esterni rotti o batteria bassa → interni fanno tutto
        posIntSX = CENTRO + pitch + roll;
        posIntDX = CENTRO + pitch - roll;
    } else {
        // Caso B: interni rotti → esterni fanno tutto
        posEstSX = CENTRO + pitch + roll;
        posEstDX = CENTRO + pitch - roll;
    }
    // Attach/detach automatico basato sullo stato attuale
    if (interniAttivi != statoPrecedenteInterni) { 
        if (interniAttivi) {
            servoInternoSX.attach(pinIntSX); 
            servoInternoDX.attach(pinIntDX); 
        } else {
            servoInternoSX.detach();
            servoInternoDX.detach(); 
        }
        statoPrecedenteInterni = interniAttivi;
    }

    if (esterniAttivi != statoPrecedenteEsterni) { 
        if (esterniAttivi) {
            servoEsternoSX.attach(pinEstSX);  
            servoEsternoDX.attach(pinEstDX);
        } else {
            servoEsternoSX.detach();           
            servoEsternoDX.detach(); 
        }
        statoPrecedenteEsterni = esterniAttivi;
    }

    // Limiti di sicurezza (45° – 135°)
    posIntSX = constrain(posIntSX, 45, 135);
    posIntDX = constrain(posIntDX, 45, 135);
    posEstSX = constrain(posEstSX, 45, 135);
    posEstDX = constrain(posEstDX, 45, 135);

    // Comando fisico ai servomotori
    if (interniAttivi) { 
        servoInternoSX.write(posIntSX); 
        servoInternoDX.write(posIntDX); 
    }
    if (esterniAttivi) { 
        servoEsternoSX.write(posEstSX); 
        servoEsternoDX.write(posEstDX); 
    }
}
// ============================================================
//  NAVIGAZIONE GPS
// ============================================================
void aggiornaNavigazione(float angoloYaw)
{
    if (gps.location.isValid()) {
        global_distanzaDalTarget = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);

        // Angolo in GRADI (0=Nord, 90=Est, 180=Sud, 270=Ovest) verso il target
        global_rottaVersoTarget = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);

        // Differenza tra direzione target e direzione attuale (yaw)
        global_errore_rotta = global_rottaVersoTarget - angoloYaw;

        // Via più breve per girare (normalizzazione ±180°)
        if      (global_errore_rotta >  180.0) {
            global_errore_rotta -= 360.0;
        }else if (global_errore_rotta < -180.0) {
            global_errore_rotta += 360.0;
        }

        // L1 Calcola l'accelerazione laterale necessaria per curvare dolcemente verso la rotta: a_lat = 2*V^2/l1 *sin(eta)
        float L1 = max(Velocita_stimata_Ms * 4.0f, 1.0f); // minimo 1 metro per evitare divisione per zero mettere tutti float
        float eta = radians(global_errore_rotta);
        float a_lat = (2 * Velocita_stimata_Ms * Velocita_stimata_Ms / L1) * sin(eta);
        float rollRad = atan(a_lat / 9.81);
        global_targetRoll = constrain(degrees(rollRad), -MAX_ROLL, MAX_ROLL);

        Serial.print("Target a: ");
        Serial.print(global_distanzaDalTarget);
        Serial.print(" metri | Direzione bussola: ");
        Serial.print(global_rottaVersoTarget);
        Serial.print(" gradi | Errore rotta: ");
        Serial.print(global_errore_rotta);
        Serial.print(" gradi | Target Roll calcolato: ");
        Serial.println(global_targetRoll);
    } else {
        Serial.println("GPS: In attesa di segnale valido (FIX)...");
    }
}

// ============================================================
//  DIAGNOSTICA SERVI
// ============================================================
void diagnosticaServi()
{
    float mA;

    mA = sensoreEstSX.getCurrent_mA();
    estSX_Ok = (mA > 1 && mA < 2500.0);
    if (!estSX_Ok) Serial.println("WARN: ServoEstSX anomalia corrente!");

    mA = sensoreEstDX.getCurrent_mA();
    estDX_Ok = (mA > 1.0 && mA < 2500.0);
    if (!estDX_Ok) Serial.println("WARN: ServoEstDX anomalia corrente!");

    mA = sensoreIntSX.getCurrent_mA();
    intSX_Ok = (mA > 1.0 && mA < 2500.0);
    if (!intSX_Ok) Serial.println("WARN: ServoIntSX anomalia corrente!");

    mA = sensoreIntDX.getCurrent_mA();
    intDX_Ok = (mA > 1.0 && mA < 2500.0);
    if (!intDX_Ok) Serial.println("WARN: ServoIntDX anomalia corrente!");
}

// ============================================================
//  GESTIONE LUCI DI STATO
// ============================================================
void gestisciLuci() {
    
    // 1. EMERGENZA CRITICA: Guasto Servi (Tutti accesi fissi)
    if (!estSX_Ok || !estDX_Ok || !intSX_Ok || !intDX_Ok) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH); 
        digitalWrite(PIN_LED_VERDE_GPS, HIGH);
        digitalWrite(PIN_LED_BLU_PID,   HIGH);
        return; 
    }

    // 2. LED ROSSO: Allarmi (Batteria o Failsafe)
    if (batteriaBassa || failsafe) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH); 
    } else {
        digitalWrite(PIN_LED_ROSSO_ALARM, LOW);  
    }

    // 3. LED VERDE: Stato GPS
    if (gps.location.isValid()) {
        digitalWrite(PIN_LED_VERDE_GPS, HIGH); 
    } else {
        digitalWrite(PIN_LED_VERDE_GPS, LOW);  
    }

    // 4. LED BLU: Modalità di volo
    if (global_modalitaVolo == 2) {
        digitalWrite(PIN_LED_BLU_PID, HIGH);   
    } else {
        digitalWrite(PIN_LED_BLU_PID, LOW);    
    }
}

// ============================================================
//  INVIO TELEMETRIA COMPLETA (Formato CSV)
// ============================================================
void inviaTelemetria(float pitch, float roll, float yaw, float velPitotKmh, float velGpsKmh, float velStimataKmh, int outPitch, int outRoll, int outGas) {
    
    unsigned long tempoAttuale = millis();

    // Invio a 2 Hz (ogni 500 ms) per non saturare la banda radio LoRa
    if (tempoAttuale - timerTelemetria > 500) {
        timerTelemetria = tempoAttuale;

        // --- LETTURE VOLTAGGI IN TEMPO REALE ---
        float vBatt  = sensoreBatt.getBusVoltage_V();
        float vIntSX = sensoreIntSX.getBusVoltage_V();
        float vIntDX = sensoreIntDX.getBusVoltage_V();
        float vEstSX = sensoreEstSX.getBusVoltage_V();
        float vEstDX = sensoreEstDX.getBusVoltage_V();

        // --- CODICE ALLARMI ---
        int codiceAllarme = 0;
        if (failsafe) codiceAllarme += 1;
        if (batteriaBassa) codiceAllarme += 2;

        // ==========================================
        //  COMPOSIZIONE TRENO DATI CSV
        // ==========================================
        TELEMETRIA.print("$,"); // 0. Start
        
        // --- STATO E ALLARMI ---
        TELEMETRIA.print(global_modalitaVolo);     
        TELEMETRIA.print(","); // 1
        TELEMETRIA.print(codiceAllarme);           
        TELEMETRIA.print(","); // 2
        
        // --- ALIMENTAZIONE ---
        TELEMETRIA.print(vBatt, 2);                
        TELEMETRIA.print(","); // 3
        TELEMETRIA.print(vIntSX, 2);               
        TELEMETRIA.print(","); // 4
        TELEMETRIA.print(vIntDX, 2);               
        TELEMETRIA.print(","); // 5
        TELEMETRIA.print(vEstSX, 2);               
        TELEMETRIA.print(","); // 6
        TELEMETRIA.print(vEstDX, 2);               
        TELEMETRIA.print(","); // 7
        
        // --- STATO SERVI (Stringa es. "1111") ---
        TELEMETRIA.print(intSX_Ok ? "1" : "0");
        TELEMETRIA.print(intDX_Ok ? "1" : "0");
        TELEMETRIA.print(estSX_Ok ? "1" : "0");
        TELEMETRIA.print(estDX_Ok ? "1" : "0");    
        TELEMETRIA.print(","); // 8

        // --- ASSETTO E QUOTA (IMU + Baro) ---
        TELEMETRIA.print(pitch, 1);                
        TELEMETRIA.print(","); // 9
        TELEMETRIA.print(roll, 1);                 
        TELEMETRIA.print(","); // 10
        TELEMETRIA.print(yaw, 1);                  
        TELEMETRIA.print(","); // 11
        TELEMETRIA.print(global_altitudineDalSuolo, 1); 
        TELEMETRIA.print(","); // 12
        
        // --- VELOCITÀ ---
        TELEMETRIA.print(velPitotKmh, 1);          
        TELEMETRIA.print(","); // 13
        TELEMETRIA.print(velGpsKmh, 1);            
        TELEMETRIA.print(","); // 14
        TELEMETRIA.print(velStimataKmh, 1);        
        TELEMETRIA.print(","); // 15
        
        // --- NAVIGAZIONE ---
        TELEMETRIA.print(global_distanzaDalTarget, 0); 
        TELEMETRIA.print(","); // 16
        TELEMETRIA.print(global_rottaVersoTarget, 1);  
        TELEMETRIA.print(","); // 17
        TELEMETRIA.print(global_targetRoll, 1);        
        TELEMETRIA.print(","); // 18
        
        // --- INPUT RADIOCOMANDO (Grezzi 172-1811) ---
        TELEMETRIA.print(canaliRC[1]);             
        TELEMETRIA.print(","); // 19 (RC Pitch)
        TELEMETRIA.print(canaliRC[0]);             
        TELEMETRIA.print(","); // 20 (RC Roll)
        TELEMETRIA.print(canaliRC[2]);             
        TELEMETRIA.print(","); // 21 (RC Gas)
        
        // --- OUTPUT PID/MIXER ---
        TELEMETRIA.print(outPitch);                
        TELEMETRIA.print(","); // 22 (PID Pitch)
        TELEMETRIA.print(outRoll);                 
        TELEMETRIA.print(","); // 23 (PID Roll)
        TELEMETRIA.print(outGas);                  
        TELEMETRIA.print(","); // 24 (PID Gas)
        
        // --- POSIZIONE FISICA ATTUALE SERVI (Gradi) ---
        TELEMETRIA.print(servoInternoSX.read());   
        TELEMETRIA.print(","); // 25
        TELEMETRIA.print(servoInternoDX.read());   
        TELEMETRIA.print(","); // 26
        TELEMETRIA.print(servoEsternoSX.read());   
        TELEMETRIA.print(","); // 27
        TELEMETRIA.print(servoEsternoDX.read());   
        TELEMETRIA.print(","); // 28

        // --- TEMPERATURE ---
        TELEMETRIA.print(Global_Temperatura_motore, 1);
        TELEMETRIA.print(","); // 29
        TELEMETRIA.print(Global_Temperatura_teensy, 1);
        TELEMETRIA.print(","); // 30
        
        // --- SATELLITI GPS ---
        if (gps.location.isValid()) {
            TELEMETRIA.print(gps.satellites.value());                    
        } else {
            TELEMETRIA.print("0");
        }

        // CHIUSURA PACCHETTO
        TELEMETRIA.println(); 
    }
}
// ============================================================
//  CALCOLO PID
// ============================================================
void calcolaPID(float targetAltitudine, float targetRoll,
                float pitchReale, float rollReale,
                float velocitaAttuale, float targetVelocita,
                int gasDiBase,
                int &comandoPitchOut, int &comandoRollOut, int &comandoGasOut)
{
    // ----------------------------------------------------------
    // 1. CALCOLO DEL TEMPO
    // ----------------------------------------------------------
    unsigned long tempoAttuale = millis();
    float dt = (tempoAttuale - tempoPassatoPID) / 1000.0;

    if (dt <= 0.0) return;
    tempoPassatoPID = tempoAttuale;

    float kp_dinamico_roll  = Kp_roll  * global_fattoreReattivita;
    float kp_dinamico_pitch = Kp_pitch * global_fattoreReattivita;

    // ----------------------------------------------------------
    // 2. PID ALTITUDINE
    // ----------------------------------------------------------
    float targetPitch_Auto = 0.0;

    if (global_altitudineDalSuolo > ALTEZZA_MAX) {
        targetPitch_Auto      = -8.0;
        pid_sommaErroriAlt    =  0.0;  // reset integrale in saturazione
        pid_errorePassatoAlt  =  0.0;
    } else if (global_altitudineDalSuolo < ALTEZZA_MIN) {
        targetPitch_Auto      = 12.0;
        pid_sommaErroriAlt    =  0.0;
        pid_errorePassatoAlt  =  0.0;
    } else {
        float erroreAltitudine = targetAltitudine - global_altitudineDalSuolo;

        float P_alt = Kp_alt * erroreAltitudine;

        pid_sommaErroriAlt += erroreAltitudine * dt;
        pid_sommaErroriAlt  = constrain(pid_sommaErroriAlt, -20.0, 20.0);
        float I_alt = Ki_alt * pid_sommaErroriAlt;

        float D_alt = Kd_alt * ((erroreAltitudine - pid_errorePassatoAlt) / dt);
        pid_errorePassatoAlt = erroreAltitudine;

        targetPitch_Auto = constrain(P_alt + I_alt + D_alt, -10.0, 15.0);
    }

    // ----------------------------------------------------------
    // 3. PID PITCH
    // ----------------------------------------------------------
    float errorePitch = targetPitch_Auto - pitchReale;

    float P_Pitch = kp_dinamico_pitch * errorePitch;

    pid_sommaErroriPitch += errorePitch * dt;
    pid_sommaErroriPitch  = constrain(pid_sommaErroriPitch, -40.0, 40.0);
    float I_Pitch = Ki_pitch * pid_sommaErroriPitch;

    float D_Pitch = Kd_pitch * ((errorePitch - pid_errorePassatoPitch) / dt);
    pid_errorePassatoPitch = errorePitch;

    comandoPitchOut = (int)(P_Pitch + I_Pitch + D_Pitch);
    comandoPitchOut = constrain(comandoPitchOut, -MAX_PITCH, MAX_PITCH);

    // ----------------------------------------------------------
    // 4. PID ROLL
    // ----------------------------------------------------------
    float erroreRoll = targetRoll - rollReale;  

    float P_Roll = kp_dinamico_roll * erroreRoll;

    pid_sommaErroriRoll += erroreRoll * dt;
    pid_sommaErroriRoll  = constrain(pid_sommaErroriRoll, -40.0, 40.0);
    float I_Roll = Ki_roll * pid_sommaErroriRoll;

    float D_Roll = Kd_roll * ((erroreRoll - pid_errorePassatoRoll) / dt);
    pid_errorePassatoRoll = erroreRoll;

    comandoRollOut = (int)(P_Roll + I_Roll + D_Roll);
    comandoRollOut  = constrain(comandoRollOut,  -MAX_ROLL,  MAX_ROLL);

    // ----------------------------------------------------------
    // 5. AUTOTHROTTLE (PID velocità)
    // ----------------------------------------------------------
    float erroreVel = targetVelocita - velocitaAttuale;

    float P_vel = Kp_vel * erroreVel;

    pid_sommaErroriVel += erroreVel * dt;
    pid_sommaErroriVel  = constrain(pid_sommaErroriVel, -30.0, 30.0);
    float I_vel = Ki_vel * pid_sommaErroriVel;

    float D_vel = Kd_vel * ((erroreVel - pid_errorePassatoVel) / dt);
    pid_errorePassatoVel = erroreVel;

    int gasCalcolato = gasDiBase + (int)(P_vel + I_vel + D_vel);
    comandoGasOut = constrain(gasCalcolato, GAS_MINIMO, GAS_MASSIMO);
}