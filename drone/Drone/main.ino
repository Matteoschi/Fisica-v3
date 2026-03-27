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
float temp_aria_barometro = 0.0; 

// ============================================================
//  CONFIGURAZIONE GPS
// ============================================================
#define GPS_SERIAL  Serial1
#define BAUD_RATE_GPS 9600
// ============================================================
//  TELEMETRIA LORA 
// ============================================================
#define TELEMETRIA Serial4         
unsigned long timerTelemetria = 0;
// ============================================================
//  RICEVENTE SBUS (FrSky)
// ============================================================
SBUS   ricevente(Serial3);
uint16_t canaliRC[16];
bool failsafe   = false;
bool pacchettoPerso = false;
// ============================================================
//  PITOT (VELOCITÀ ARIA)
// ============================================================
const int   PIN_ARIA  = A0;
const float DENSITA_ARIA = 1.225;  // kg/m³
float VALORE_ZERO  = 0.0;    
const float FATTORE_CONVERSIONE_PA = 3.22;
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
Adafruit_INA219 sensoreMotore(0x40);  
Adafruit_INA219 sensoreIntSX(0x41);
Adafruit_INA219 sensoreIntDX(0x42);
Adafruit_INA219 sensoreEstSX(0x43);
Adafruit_INA219 sensoreEstDX(0x44);
Adafruit_INA219 sensoreTeensy(0x45); 
// ============================================================
//  COSTANTI
// ============================================================
const int VALORE_BATT_MOTORE_BASSA = 11.8;
const int VALORE_BATT_TEENSY_BASSA = 4.9;
const int SOGLIA_G_SCHIANTO=25;
const int CENTRO_SERVO  = 90;
const int MAX_ROLL = 35;
const int MAX_PITCH  = 20;
const int ALTEZZA_MAX = 120;
const int ALTEZZA_MIN = 10;
const int GAS_MINIMO = 25;
const int GAS_CROCIERA= 70;   
const int GAS_MASSIMO = 130;  
const float VELOCITA_CROCIERA  = 60.0;  
const float VELOCITA_AVVICINAMENTO= 45.0;   
const float DISTANZA_FRENATA = 150.0;  
// ============================================================
//  NAVIGAZIONE
// ============================================================
const double TARGET_LAT = 41.902782;
const double TARGET_LON = 12.496366;
const float  ALTITUDINE_TARGET = 40.0;

float global_altitudineDiPartenza = 0.0;
float global_targetRoll  = 0.0;
float global_distanzaDalTarget = 0.0;
float global_rottaVersoTarget = 0.0;
float global_errore_rotta  = 0.0;
float global_altitudineDalSuolo = 0.0;
float latPrecedente = 0.0;
float lonPrecedente= 0.0;
float velocitaGPS = 0.0;
float alpha_vel= 0.7; 
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
float pid_sommaErroriAlt   = 0.0;
float pid_errorePassatoAlt = 0.0;

float pid_sommaErroriPitch = 0.0;
float pid_errorePassatoPitch = 0.0;

float pid_sommaErroriRoll  = 0.0;
float pid_errorePassatoRoll = 0.0;

float pid_sommaErroriVel   = 0.0;
float pid_errorePassatoVel = 0.0;

// ============================================================
//  LED DI STATO E ALLARMI
// ============================================================
const int PIN_LED_ROSSO_ALARM = 2; // Allarmi come moduli mancanti / Batteria
const int PIN_LED_VERDE_GPS = 3; // GPS Fix e settaggio pitot
const int PIN_LED_BLU_PID  = 4; // Modalità AUTO pid e settaggio barometro
const int PIN_BUZZER = 5;
const int PIN_RELE = 20;


// ============================================================
//  FLAGS STATO SISTEMA
// ============================================================
bool imuPronto = false;
bool baroPronto= false;
bool pitotCalibrato = false;
bool Voltaggio = true;
int  tentativi = 0;
const int MAX_TENTATIVI = 3;

bool statoSchiantoRilevato = false; 
bool droneInVolo = false;
// batteria
bool  batteriaBassa_motore= false;
bool  batteriaBassa_teensy= false;
bool relèAttivato = false;

// Stati di salute dei servi
bool estSX_Ok = true;
bool estDX_Ok = true;
bool intSX_Ok = true;
bool intDX_Ok = true;
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
void inviaTelemetria(float pitch, float roll, float yaw, float velPitotKmh, float velGpsKmh, float velStimataKmh, int outPitch, int outRoll, int outGas);
void calcolaPID(float targetAltitudine, float targetRoll, float pitchReale, float rollReale, float velocitaAttuale, float targetVelocita, int gasDiBase, int &comandoPitchOut, int &comandoRollOut, int &comandoGasOut);
void gestisci_allarmi();
void segnalaOK();
void segnalaErrore();
void segnalaCalibrazione(int pin_led);
void gestisciSchianto();
void gestisciAlimentazione();

void segnalaOK() {
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    tone(PIN_BUZZER, 1200, 150);
    delay(300);
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
    digitalWrite(PIN_BUZZER, !digitalRead(PIN_BUZZER));
}


void setup()
{
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);

    pinMode(PIN_LED_ROSSO_ALARM, OUTPUT);
    pinMode(PIN_LED_VERDE_GPS, OUTPUT);
    pinMode(PIN_LED_BLU_PID, OUTPUT);
    pinMode(PIN_BUZZER,  OUTPUT);
    pinMode(PIN_RELE, OUTPUT);

    digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
    digitalWrite(PIN_LED_BLU_PID, LOW);
    digitalWrite(PIN_BUZZER, LOW);
    
    digitalWrite(PIN_RELE, LOW);

    // Bip di accensione
    tone(PIN_BUZZER, 800,  100); 
    delay(150);
    tone(PIN_BUZZER, 1200, 100); 
    delay(150);
    tone(PIN_BUZZER, 1600, 150); 
    delay(300);

    Serial.println("\n=========================================");
    Serial.println("     SISTEMA DRONE — AVVIO IN CORSO     ");
    Serial.println("=========================================");

    ricevente.begin();
    TELEMETRIA.begin(9600);
    GPS_SERIAL.begin(BAUD_RATE_GPS);

    // INIZIALIZZAZIONE SENSORI
    while ((!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) && tentativi < MAX_TENTATIVI) {
        tentativi++;
        Serial.println("\n-----------------------------------------");
        Serial.print  ("  Tentativo ");
        Serial.print  (tentativi);
        Serial.print  (" / ");
        Serial.println(MAX_TENTATIVI);
        Serial.println("-----------------------------------------");

        // 1. IMU 
        if (!imuPronto) {
            Serial.print("[ ] IMU BNO055 ............. ");
            if (giroscopio.begin()) {
                giroscopio.setExtCrystalUse(true);
                imuPronto = true;
                Serial.println("OK");
                segnalaOK();
            } else {
                Serial.println("ERRORE (cavi I2C?)");
                segnalaErrore();
            }
        } else {
            Serial.println("[OK] IMU BNO055");
        }

        //2. BAROMETRO 
        if (!baroPronto) {
            Serial.print("[ ] Barometro BMP390 ....... ");
            if (barometro.begin_I2C()) {
                barometro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
                barometro.setPressureOversampling(BMP3_OVERSAMPLING_32X);
                barometro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
                barometro.setOutputDataRate(BMP3_ODR_50_HZ);
                delay(100);

                float sommaAlt = 0.0;
                for (int i = 0; i < 20; i++) {
                    segnalaCalibrazione(PIN_LED_BLU_PID);
                    sommaAlt += barometro.readAltitude(1013.25);
                    delay(50);
                }
                digitalWrite(PIN_LED_BLU_PID,LOW);
                digitalWrite(PIN_BUZZER,LOW);

                baroPronto = true;
                global_altitudineDiPartenza = sommaAlt / 20.0;
                Serial.print("OK (tara: ");
                Serial.print(global_altitudineDiPartenza, 1);
                Serial.println(" m)");
                segnalaOK();
            } else {
                Serial.println("ERRORE (cavi I2C?)");
                segnalaErrore();
            }
        } else {
            Serial.println("[OK] Barometro BMP390");
        }

        // 3. PITOT 
        if (!pitotCalibrato) {
            Serial.print("[ ] Pitot (velocita') ...... ");
            long sommaLetture = 0;
            for (int i = 0; i < 100; i++) {
                if (i % 10 == 0) {
                    segnalaCalibrazione(PIN_LED_VERDE_GPS);
                }
                sommaLetture += analogRead(PIN_ARIA);
                delay(10);
            }
            digitalWrite(PIN_LED_VERDE_GPS, LOW);
            digitalWrite(PIN_BUZZER,LOW);

            VALORE_ZERO = sommaLetture / 100.0;
            if (VALORE_ZERO > 5 && VALORE_ZERO < 1020) {
                pitotCalibrato = true;
                Serial.print("OK (zero: ");
                Serial.print(VALORE_ZERO, 1);
                Serial.println(")");
                segnalaOK();
            } else {
                Serial.print("ERRORE (valore anomalo: ");
                Serial.print(VALORE_ZERO);
                Serial.println(")");
                segnalaErrore();
            }
        } else {
            Serial.println("[OK] Pitot");
        }

        // 4. INA219 
        Voltaggio = true;

        Serial.print("[ ] INA219 Batteria motore........ ");
        if (sensoreMotore.begin()) {
            Serial.println("OK");
        } else {
            Serial.println("ERRORE");
            Voltaggio = false;
        }
        Serial.print("[ ] INA219 Batteria Teensy........ ");
        if (sensoreTeensy.begin()) {
            Serial.println("OK");
        } else {
            Serial.println("ERRORE");
            Voltaggio = false;
        }

        Serial.print("[ ] INA219 Servo IntSX ..... ");
        if (sensoreIntSX.begin()) {
            Serial.println("OK");
        } else {
            Serial.println("ERRORE");
            Voltaggio = false;
        }

        Serial.print("[ ] INA219 Servo IntDX ..... ");
        if (sensoreIntDX.begin()) {
            Serial.println("OK");
        } else {
            Serial.println("ERRORE");
            Voltaggio = false;
        }

        Serial.print("[ ] INA219 Servo EstSX ..... ");
        if (sensoreEstSX.begin()) {
            Serial.println("OK");
        } else {
            Serial.println("ERRORE");
            Voltaggio = false;
        }

        Serial.print("[ ] INA219 Servo EstDX ..... ");
        if (sensoreEstDX.begin()) {
            Serial.println("OK");
        } else {
            Serial.println("ERRORE");
            Voltaggio = false;
        }
        if (Voltaggio) {
            segnalaOK();
        } else {
            segnalaErrore();
        }

        //  Riepilogo tentativo 
        if (!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) {
            Serial.println("\n  >> Sensori mancanti. Nuovo tentativo tra 2s...");
            digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
            delay(2000);
            digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        }
    }
    //  ERRORE CRITICO 
    if (!imuPronto || !baroPronto || !pitotCalibrato) {
        Serial.println("\n!!! ERRORE CRITICO — AVVIO BLOCCATO !!!");
        Serial.println("    Controlla l'hardware e riavvia.");
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        while (1) {
            tone(PIN_BUZZER, 2000, 300);
            delay(400);
        }
    }
    
    //  TUTTO OK — INIT SERVO
    Serial.println("\n=========================================");
    Serial.println("     TUTTI I SENSORI OPERATIVI          ");
    Serial.println("=========================================");
    Serial.println("Inizializzazione servomotori...");

    servoInternoSX.attach(pinIntSX);
    servoInternoDX.attach(pinIntDX);
    servoEsternoSX.attach(pinEstSX);
    servoEsternoDX.attach(pinEstDX);
    motore.attach(pinMotore);

    servoInternoSX.write(CENTRO_SERVO);
    servoInternoDX.write(CENTRO_SERVO);
    servoEsternoSX.write(CENTRO_SERVO);
    servoEsternoDX.write(CENTRO_SERVO);
    motore.write(GAS_MINIMO);
    Serial.println("Servomotori pronti — flap neutri, gas minimo");

    // Jingle avvio riuscito
    tone(PIN_BUZZER, 800,  120); delay(170);
    tone(PIN_BUZZER, 1200, 120); delay(170);
    tone(PIN_BUZZER, 1800, 200); delay(350);

    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    digitalWrite(PIN_LED_VERDE_GPS,HIGH);
    digitalWrite(PIN_LED_BLU_PID, HIGH);
    delay(800);
    digitalWrite(PIN_LED_ROSSO_ALARM,LOW);
    digitalWrite(PIN_LED_VERDE_GPS,LOW);
    digitalWrite(PIN_LED_BLU_PID,LOW);

    tempoPassatoPID = millis();
    Serial.println("\n  >> SISTEMA PRONTO AL VOLO\n");
    delay(500);
}

// ============================================================
//  LOOP
// ============================================================
void loop()
{
    gestisciAlimentazione();
    gestisciSchianto();

    // 1. lettura GPS
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    // 2. monitoraggio voltaggio teensy e motore

    diagnosticaServi();

    // 3. lettura giroscopio (IMU)
    sensors_event_t event;
    giroscopio.getEvent(&event);
    float angoloPitch = event.orientation.y;
    float angoloRoll  = event.orientation.z;
    float angoloYaw   = event.orientation.x;

    // 4. aggiorna navigazione
    aggiornaNavigazione(angoloYaw);

    // 5. lettura barometro
    float altitudineAttuale  = barometro.readAltitude(1013.25);
    global_altitudineDalSuolo = altitudineAttuale - global_altitudineDiPartenza;

    // 6. controllo temperature
    temp_aria_barometro = barometro.temperature;
    float voltaggio_Sensore_motore = analogRead(PIN_T_motore) * (3.3 / 1023.0);
    Global_Temperatura_motore = (voltaggio_Sensore_motore - 0.5) * 100.0;

    // 7. pitot – lettura velocità aria
    float Velocita_pitot_Ms=0.0;
    int lettura_dal_pin_pitot= analogRead(PIN_ARIA);
    lettura_dal_pin_pitot = constrain(lettura_dal_pin_pitot, 0, 1023);
    float differenza= (float)lettura_dal_pin_pitot - VALORE_ZERO;
    if (differenza < 0) {
        differenza = 0;
    }
    float pressionePascal = differenza * FATTORE_CONVERSIONE_PA;
    if (pressionePascal >0){
        Velocita_pitot_Ms= sqrt((2.0 * pressionePascal) / DENSITA_ARIA);
    }

    //8. gps calcolo velocità (groundspeed)
    float velocita_gps_Ms = 0.0;
    if (gps.speed.isValid()) {
        velocita_gps_Ms = gps.speed.kmph() / 3.6;
        if (latPrecedente == 0.0 && lonPrecedente == 0.0) {
            latPrecedente = gps.location.lat();
            lonPrecedente = gps.location.lng();
        }
    }
    Velocita_stimata_Ms = (alpha_vel * Velocita_pitot_Ms) + ((1.0 - alpha_vel) * velocita_gps_Ms);

    // 9. PREPARAZIONE DATI MOTORE
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
            global_modalitaVolo = 1; 
        } else {
            global_modalitaVolo = 2; 
        }
    }
    
    int statoAttuale;
    static int ultimoStatoStampato = 0;
    if(failsafe){
        statoAttuale = 3;
    } else {
        statoAttuale = global_modalitaVolo;
    }

    // 10. RESET PID AL CAMBIO DI MODALITÀ 
    static int modalitaPrecedente = 1;
    if (statoAttuale != modalitaPrecedente) {

        if (statoSchiantoRilevato == true) {
            statoSchiantoRilevato = false; 
            droneInVolo = false;           
    
            noTone(PIN_BUZZER); 
        
            tone(PIN_BUZZER, 1000, 100); delay(150);
            tone(PIN_BUZZER, 1500, 100);
            
            Serial.println("!!! SBLOCCO EMERGENZA ESEGUITO DA RADIO !!! Pronto al riarmo.");
        }

        pid_sommaErroriAlt  = 0.0;  pid_errorePassatoAlt = 0.0;
        pid_sommaErroriPitch= 0.0;  pid_errorePassatoPitch= 0.0;
        pid_sommaErroriRoll = 0.0;  pid_errorePassatoRoll = 0.0;
        pid_sommaErroriVel = 0.0;  pid_errorePassatoVel = 0.0;
        tempoPassatoPID = millis();
        Serial.print(">> Reset PID: modalita' ");
        Serial.print(modalitaPrecedente);
        Serial.print(" -> ");
        Serial.println(statoAttuale);           
        modalitaPrecedente = statoAttuale;      
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
    if (statoSchiantoRilevato == true) {
        motore.write(0);
    } else {
        gestisci_allarmi();
        applicaMixer4Servi(correzionePitch, correzioneRoll);
        motore.write(comandoGasFinale);
    }

    if (millis() > 10000 && gps.charsProcessed() < 10) {
        Serial.println("ATTENZIONE: Nessun dato dal GPS. Controlla i cavi TX e RX!");
    }
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
    int posIntSX = CENTRO_SERVO;
    int posIntDX = CENTRO_SERVO;
    int posEstSX = CENTRO_SERVO;
    int posEstDX = CENTRO_SERVO;

    // Batteria Teensy bassa → forza solo esterni per risparmiare corrente
    if (batteriaBassa_teensy) {
        intSX_Ok = false;
        intDX_Ok = false;
    }

    bool esterniAttivi = estSX_Ok && estDX_Ok;
    bool interniAttivi = intSX_Ok && intDX_Ok;

    if (esterniAttivi && interniAttivi) {
        // Caso A: tutto OK → interni = SOLO PITCH, esterni = SOLO ROLL
        posIntSX = CENTRO_SERVO + pitch;
        posIntDX = CENTRO_SERVO + pitch;
        posEstSX = CENTRO_SERVO + roll;
        posEstDX = CENTRO_SERVO - roll;
    } else if (esterniAttivi && !interniAttivi) {
        // Caso B: interni rotti → esterni fanno pitch + roll
        posEstSX = CENTRO_SERVO + pitch + roll;
        posEstDX = CENTRO_SERVO + pitch - roll;
    } else if (!esterniAttivi && interniAttivi) {
        // Caso C: esterni rotti → interni fanno pitch + roll
        posIntSX = CENTRO_SERVO + pitch + roll;
        posIntDX = CENTRO_SERVO + pitch - roll;
    } else {
        // Caso D: tutto rotto
        Serial.println("CRITICO: Nessun servo disponibile!");
        return;
    }

    // ── Attach/detach automatico ──────────────────────────
    if (interniAttivi != statoPrecedenteInterni) {
        if (interniAttivi) {
            servoInternoSX.attach(pinIntSX);
            servoInternoDX.attach(pinIntDX);
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
            servoEsternoSX.attach(pinEstSX);
            servoEsternoDX.attach(pinEstDX);
            Serial.println("Servi esterni: ATTIVATI");
        } else {
            servoEsternoSX.detach();
            servoEsternoDX.detach();
            Serial.println("Servi esterni: STACCATI");
        }
        statoPrecedenteEsterni = esterniAttivi;
    }

    // ── Limiti di sicurezza ───────────────────────────────
    posIntSX = constrain(posIntSX, 45, 135);
    posIntDX = constrain(posIntDX, 45, 135);
    posEstSX = constrain(posEstSX, 45, 135);
    posEstDX = constrain(posEstDX, 45, 135);

    // ── Comando fisico ────────────────────────────────────
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
        if(global_errore_rotta >  180.0) {
            global_errore_rotta -= 360.0;
        }else if (global_errore_rotta < -180.0) {
            global_errore_rotta += 360.0;
        }

        // L1 Calcola l'accelerazione laterale necessaria per curvare  verso la rotta: a_lat = 2*V^2/l1 *sin(eta)
        Velocita_stimata_Ms = max(Velocita_stimata_Ms, 1.0f); // Mai sotto 1 m/s nei calcoli
        float L1 = max(Velocita_stimata_Ms * 4.0f, 1.0f); 
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
void gestisci_allarmi() {

    // 1. EMERGENZA CRITICA: Guasto Servi (Tutti accesi fissi)
    if (!estSX_Ok || !estDX_Ok || !intSX_Ok || !intDX_Ok) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH); 
        digitalWrite(PIN_LED_VERDE_GPS, HIGH);
        digitalWrite(PIN_LED_BLU_PID,   HIGH);
        return; 
    }

    // 2. LED ROSSO: Allarmi (Batteria o Failsafe)
    if (batteriaBassa_motore || failsafe) {
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
// ============================================================
//  INVIO TELEMETRIA COMPLETA (Formato CSV)
// ============================================================
void inviaTelemetria(float pitch, float roll, float yaw, float velPitotKmh, float velGpsKmh, float velStimataKmh, int outPitch, int outRoll, int outGas) {
    
    unsigned long tempoAttuale = millis();

    // Invio a 2 Hz (ogni 500 ms) per non saturare la banda radio LoRa
    if (tempoAttuale - timerTelemetria > 500) {
        timerTelemetria = tempoAttuale;

        // --- LETTURE VOLTAGGI IN TEMPO REALE ---
        float vBatt  = sensoreMotore.getBusVoltage_V();
        float vTeensy = sensoreTeensy.getBusVoltage_V();
        float vIntSX = sensoreIntSX.getBusVoltage_V();
        float vIntDX = sensoreIntDX.getBusVoltage_V();
        float vEstSX = sensoreEstSX.getBusVoltage_V();
        float vEstDX = sensoreEstDX.getBusVoltage_V();


        int codiceAllarme = 0;
        if (failsafe)             codiceAllarme += 1;  // Bit 0
        if (batteriaBassa_motore) codiceAllarme += 2;  // Bit 1
        if (relèAttivato)         codiceAllarme += 4;  // Bit 2
        if (batteriaBassa_teensy) codiceAllarme += 8;  // Bit 3
        if (statoSchiantoRilevato)codiceAllarme += 16; // Bit 4
        if (droneInVolo)          codiceAllarme += 32; // Bit 5 


        TELEMETRIA.print("$,"); // 0. Start indicatore
        
        // --- STATO E ALLARMI ---
        TELEMETRIA.print(global_modalitaVolo);     TELEMETRIA.print(","); // 1. (1=Manuale, 2=Auto, 3=Failsafe)
        TELEMETRIA.print(codiceAllarme);           TELEMETRIA.print(","); // 2. Bitmask allarmi globali
        
        // --- ALIMENTAZIONE ---
        TELEMETRIA.print(vBatt, 2);                TELEMETRIA.print(","); // 3. V Motore
        TELEMETRIA.print(vTeensy, 2);              TELEMETRIA.print(","); // 4. V Teensy
        TELEMETRIA.print(vIntSX, 2);               TELEMETRIA.print(","); // 5. V Servo Int SX
        TELEMETRIA.print(vIntDX, 2);               TELEMETRIA.print(","); // 6. V Servo Int DX
        TELEMETRIA.print(vEstSX, 2);               TELEMETRIA.print(","); // 7. V Servo Est SX
        TELEMETRIA.print(vEstDX, 2);               TELEMETRIA.print(","); // 8. V Servo Est DX
        
        // --- STATO SALUTE SERVI 
        TELEMETRIA.print(intSX_Ok ? "1" : "0");
        TELEMETRIA.print(intDX_Ok ? "1" : "0");
        TELEMETRIA.print(estSX_Ok ? "1" : "0");
        TELEMETRIA.print(estDX_Ok ? "1" : "0");    TELEMETRIA.print(","); // 9. Salute Servi

        // --- ASSETTO E QUOTA (IMU + Baro) ---
        TELEMETRIA.print(pitch, 1);                TELEMETRIA.print(","); // 10. Pitch reale
        TELEMETRIA.print(roll, 1);                 TELEMETRIA.print(","); // 11. Roll reale
        TELEMETRIA.print(yaw, 1);                  TELEMETRIA.print(","); // 12. Yaw reale (Bussola)
        TELEMETRIA.print(global_altitudineDalSuolo, 1); TELEMETRIA.print(","); // 13. Altitudine relativa
        
        // --- VELOCITÀ ---
        TELEMETRIA.print(velPitotKmh, 1);          TELEMETRIA.print(","); // 14. Velocità Aria (Pitot)
        TELEMETRIA.print(velGpsKmh, 1);            TELEMETRIA.print(","); // 15. Velocità Suolo (GPS)
        TELEMETRIA.print(velStimataKmh, 1);        TELEMETRIA.print(","); // 16. Velocità Fusa (Pitot+GPS)
        
        // --- NAVIGAZIONE ---
        TELEMETRIA.print(global_distanzaDalTarget, 0); TELEMETRIA.print(","); // 17. Distanza target (m)
        TELEMETRIA.print(global_rottaVersoTarget, 1);  TELEMETRIA.print(","); // 18. Rotta target (Gradi)
        TELEMETRIA.print(global_targetRoll, 1);        TELEMETRIA.print(","); // 19. Rollio comandato da L1
        
        // --- INPUT RADIOCOMANDO (Grezzi 172-1811) ---
        TELEMETRIA.print(canaliRC[1]);             TELEMETRIA.print(","); // 20. RC Pitch
        TELEMETRIA.print(canaliRC[0]);             TELEMETRIA.print(","); // 21. RC Roll
        TELEMETRIA.print(canaliRC[2]);             TELEMETRIA.print(","); // 22. RC Gas
        
        // --- OUTPUT PID/MIXER ---
        TELEMETRIA.print(outPitch);                TELEMETRIA.print(","); // 23. PID Pitch Out
        TELEMETRIA.print(outRoll);                 TELEMETRIA.print(","); // 24. PID Roll Out
        TELEMETRIA.print(outGas);                  TELEMETRIA.print(","); // 25. PID Gas Out
        
        // --- POSIZIONE FISICA ATTUALE SERVI (Gradi 45-135) ---
        TELEMETRIA.print(servoInternoSX.read());   TELEMETRIA.print(","); // 26. Pos Servo Int SX
        TELEMETRIA.print(servoInternoDX.read());   TELEMETRIA.print(","); // 27. Pos Servo Int DX
        TELEMETRIA.print(servoEsternoSX.read());   TELEMETRIA.print(","); // 28. Pos Servo Est SX
        TELEMETRIA.print(servoEsternoDX.read());   TELEMETRIA.print(","); // 29. Pos Servo Est DX

        // --- TEMPERATURE ---
        TELEMETRIA.print(Global_Temperatura_motore, 1); TELEMETRIA.print(","); // 30. Temp Motore
        TELEMETRIA.print(temp_aria_barometro, 1);       TELEMETRIA.print(","); // 31. Temp Avionica/Teensy
        
        // --- SATELLITI E COORDINATE GPS ---
        if (gps.location.isValid()) {
            TELEMETRIA.print(gps.satellites.value());  TELEMETRIA.print(","); // 32. Numero Satelliti
            TELEMETRIA.print(gps.location.lat(), 6);   TELEMETRIA.print(","); // 33. Latitudine
            TELEMETRIA.print(gps.location.lng(), 6);   // 34. Longitudine 
        } else {
            TELEMETRIA.print("0,0.000000,0.000000"); // Satelliti=0, Lat=0, Lon=0
        }
        TELEMETRIA.print(",");
        TELEMETRIA.print(relèAttivato ? "1" : "0"); 
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

    if (dt <= 0.001) return; // Evita divisioni per zero
    if (dt > 0.5) dt = 0.5;  // Evita lag improvvisi
    tempoPassatoPID = tempoAttuale;

    // ----------------------------------------------------------
    // 2. PID ALTITUDINE
    // ----------------------------------------------------------
    float targetPitch_Auto = 0.0;
    int gasCorrente = gasDiBase; 

    if (global_altitudineDalSuolo > ALTEZZA_MAX) {
        targetPitch_Auto = -8.0;
        gasCorrente = GAS_MINIMO + 5; // Taglia il gas in picchiata!
        pid_sommaErroriAlt =  0.0;  
        pid_errorePassatoAlt=  0.0;
    } else if (global_altitudineDalSuolo < ALTEZZA_MIN) {
        targetPitch_Auto = 12.0;
        gasCorrente = GAS_MASSIMO - 10; // Dai gas per risalire!
        pid_sommaErroriAlt =  0.0;
        pid_errorePassatoAlt =  0.0;
    } else {
        float erroreAltitudine = targetAltitudine - global_altitudineDalSuolo;
        erroreAltitudine = constrain(erroreAltitudine, -20.0, 20.0);

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

    float P_Pitch = Kp_pitch * errorePitch;

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

    float P_Roll = Kp_roll * erroreRoll;

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

    // Usa gasCorrente calcolato dalla sezione Altitudine!
    int gasCalcolato = gasCorrente + (int)(P_vel + I_vel + D_vel);
    comandoGasOut = constrain(gasCalcolato, GAS_MINIMO, GAS_MASSIMO);
}
void gestisciSchianto() {
    if (statoSchiantoRilevato) {
        motore.write(GAS_MINIMO); 
        return; 
    }

    // 2. Controllo decollo (Usa la nuova variabile globale)
    if (Velocita_stimata_Ms > 5.0 || global_altitudineDalSuolo > 5.0) {
        droneInVolo = true;
    }

    if (!droneInVolo) return; 

    // 3. Lettura G-Force
    imu::Vector<3> accel = giroscopio.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    float accelerazioneTotale = sqrt((accel.x() * accel.x()) + (accel.y() * accel.y()) + (accel.z() * accel.z()));

    if (accelerazioneTotale > SOGLIA_G_SCHIANTO) {
        statoSchiantoRilevato = true;
        motore.write(GAS_MINIMO); 
        
        Serial.println("!!! IMPATTO RILEVATO DALL'IMU !!!");
        Serial.print("Forza: "); 
        Serial.print(accelerazioneTotale); 
        Serial.println(" m/s^2");
        
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        digitalWrite(PIN_LED_VERDE_GPS, HIGH);
        digitalWrite(PIN_LED_BLU_PID, HIGH);
        tone(PIN_BUZZER, 2000); 
    }
}

void gestisciAlimentazione() {
    float vTeensy = sensoreTeensy.getBusVoltage_V();
    float vMotore = sensoreMotore.getBusVoltage_V();

    // ── BATTERIA TEENSY 
    batteriaBassa_teensy = (vTeensy < VALORE_BATT_TEENSY_BASSA);
    if (batteriaBassa_teensy) {
        Serial.println("WARN: Batteria Teensy bassa");
        if (!relèAttivato) {
            digitalWrite(PIN_RELE, HIGH);
            relèAttivato = true;
            Serial.println(">>> FAILOVER: Rele' attivato, subentra batteria motore");
        }
    }
    // ── BATTERIA MOTORE 
    batteriaBassa_motore = (vMotore < VALORE_BATT_MOTORE_BASSA);
}