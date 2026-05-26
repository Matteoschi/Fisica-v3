#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include "SBUS.h"
#include <Adafruit_INA219.h>
#include <Adafruit_BMP3XX.h>

//  SENSORI
TinyGPSPlus gps;
Adafruit_BNO055  giroscopio = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX barometro;

const int PIN_T_motore=A1;
float Global_Temperatura_motore = 0.0;
float temp_aria_barometro = 0.0; 

//  CONFIGURAZIONE GPS
#define GPS_SERIAL  Serial1
#define BAUD_RATE_GPS 9600
#define BAUD_RATE_LORA 57600
#define BAUD_RATE_LIDAR 115200

//  TELEMETRIA LORA 
#define TELEMETRIA Serial4         
unsigned long timerTelemetria = 0;

//  RICEVENTE SBUS (FrSky)
SBUS   ricevente(Serial3);
uint16_t canaliRC[16];
bool failsafe   = false;
bool pacchettoPerso = false;

//  PITOT (VELOCITÀ ARIA)
const int   PIN_ARIA  = A0;
const float DENSITA_ARIA = 1.225;  
float VALORE_ZERO  = 0.0;    
const float FATTORE_CONVERSIONE_PA = 3.22;

//  SERVO
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

//  SENSORE CORRENTE
Adafruit_INA219 sensoreMotore(0x40);  
Adafruit_INA219 sensoreIntSX(0x41);
Adafruit_INA219 sensoreIntDX(0x42);
Adafruit_INA219 sensoreEstSX(0x43);
Adafruit_INA219 sensoreEstDX(0x44);
Adafruit_INA219 sensoreTeensy(0x45); 

//  COSTANTI
const float VALORE_BATT_MOTORE_BASSA = 11.8f;
const float VALORE_BATT_TEENSY_BASSA = 4.9f;

const int SOGLIA_G_SCHIANTO=50;
const int SEMPLE_VALORI_SCHIANTO=3;

const int CENTRO_SERVO  = 90;
const int MAX_ROLL = 35;
const int MAX_PITCH  = 20;

const int ALTEZZA_MAX = 120;
const int ALTEZZA_MIN = 10;

const int GAS_NEUTRO = 1000;
const int GAS_MASSIMO = 2000;
const int GAS_MINIMO = 1200;   
const int GAS_CROCIERA= 1450; 

const float VELOCITA_CROCIERA  = 60.0;  
const float VELOCITA_AVVICINAMENTO= 45.0;   
const float DISTANZA_FRENATA = 150.0; 

const int   IMU_CAMPIONI_TARA = 200;

const float T_MOTORE_THROTTLE_START = 70.0f;  
const float T_MOTORE_THROTTLE_END  = 90.0f; 

const float ALPHA_LIDAR = 0.25f;
const float ALTEZZA_MAX_LIDAR = 6.0f;

const float  alpha_vel= 0.7; 

//  NAVIGAZIONE
double TARGET_LAT = 41.902782;
double TARGET_LON = 12.496366;
float  ALTITUDINE_TARGET = 40.0;

float G_altitudine_lidar = -1.0;  
float set_up_lidar_alt = 0.0;            // Dal LIDAR in setup
float G_altitudine_baro = 0.0;   
float set_up_gps_alt = 0.0;              // Dal barometro in setup  
float G_altitudine = 0.0;          // Fuso LIDAR + Baro

float G_tara_altitudine = 0.0;
float G_targetRoll  = 0.0;
float G_distanzaDalTarget = 0.0;
float G_rottaVersoTarget = 0.0;
float G_errore_rotta  = 0.0;

float G_Velocità_MS = 0.0;

float offsetRoll  = 0.0f;
float offsetPitch = 0.0f;
float offsetyaw   = 0.0f;

//  PID — GUADAGNI
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

//  VARIABILI DI STATO PID GLOBALI
float pid_sommaErroriAlt   = 0.0;
float pid_errorePassatoAlt = 0.0;

float pid_sommaErroriPitch = 0.0;
float pid_errorePassatoPitch = 0.0;

float pid_sommaErroriRoll  = 0.0;
float pid_errorePassatoRoll = 0.0;

float pid_sommaErroriVel   = 0.0;
float pid_errorePassatoVel = 0.0;


const unsigned long TEMPO_DECOLLO_SICURO_MS = 1500;
const float SOGLIA_VELO_DECOLLO_MS = 5.0;
const float SOGLIA_ALT_DECOLLO_M = 5.0;
unsigned long timestampDecollo = 0;
static int contatoreImpatto = 0;

//  LED DI STATO E ALLARMI
const int PIN_LED_ROSSO_ALARM = 2; // Allarmi come moduli mancanti / Batteria
const int PIN_LED_VERDE_GPS = 3; // GPS Fix e settaggio pitot
const int PIN_LED_BLU_PID  = 4; // Modalità AUTO pid e settaggio barometro
const int PIN_BUZZER = 12;     
const int PIN_RELE = 20;

//  FLAGS STATO SISTEMA
bool imuPronto       = false;
bool lidarOk = false;
bool baroPronto      = false;
bool pitotCalibrato  = false;
bool Voltaggio       = true;
int  tentativi       = 0;
const int MAX_TENTATIVI = 3;

bool servo_sicurezza         = true;
bool alimentazione_sicurezza = true;   
bool schianto_sicurezza      = true;

bool statoSchiantoRilevato   = false;
bool droneInVolo             = false;
bool schiantoBloccato        = false;

bool batteriaBassa_motore    = false;
bool batteriaBassa_teensy    = false;
bool relèAttivato            = false;

bool estSX_Ok = true, estDX_Ok = true;
bool intSX_Ok = true, intDX_Ok = true;
int  global_modalitaVolo       = 1;
bool statoPrecedenteInterni    = true;
bool statoPrecedenteEsterni    = true;

bool sistema_sicurezza_temp = true;

//  PROTOTIPI
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
void verifica_drone_in_volo();
void inizializzazione_servo();
void inizializzazione_motore();
void comandi_da_terra();                       
void elaboraComando(const String& cmd);     
void aggiornaLidar();
void altitudine();
int  gasMaxTermico();   

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
    tone(PIN_BUZZER, 1000, 30);
}


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

    Serial.println("     SISTEMA DRONE — AVVIO IN CORSO     ");;

    ricevente.begin();
    TELEMETRIA.begin(BAUD_RATE_LORA);
    GPS_SERIAL.begin(BAUD_RATE_GPS);
    Serial2.begin(BAUD_RATE_LIDAR); 
    delay(100);

    while (Serial2.available()) Serial2.read();
    
    // INIZIALIZZAZIONE SENSORI
    while ((!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) && tentativi < MAX_TENTATIVI) {
        tentativi++;
        Serial.println("\n-----------------------------------------");
        Serial.print  ("  Tentativo ");
        Serial.print  (tentativi);
        Serial.print  (" / ");
        Serial.println(MAX_TENTATIVI);
        Serial.println("-----------------------------------------");

        if (!lidarOk) {
            Serial.println("[ ] TF-Luna LIDAR .............. ");
            unsigned long t0 = millis();
            while (millis() - t0 < 3000) {
                if (Serial2.available() >= 9) {
                    if (Serial2.read() == 0x59 && Serial2.read() == 0x59) {
                        for (int i = 0; i < 7; i++) {
                            Serial2.read();
                        }
                        lidarOk = true;
                        break;
                    }
                }
            }
            if (lidarOk) {
                for (int i = 0; i < 10; i++) {
                    aggiornaLidar();
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

        // 1. IMU 
        if (!imuPronto) {
        Serial.print("[ ] IMU BNO055 ................. ");
        if (giroscopio.begin()) {
            giroscopio.setExtCrystalUse(true);
            Serial.println("OK");
            // 1. calibrazione interna giroscopio
            Serial.print("   Calibrazione interna (non muovere)");
            uint8_t sys, gyro, accel, mag;
            unsigned long timeout = millis();
            do {
                giroscopio.getCalibration(&sys, &gyro, &accel, &mag);
                Serial.print(".");
                delay(100);
                if (millis() - timeout > 10000) {
                    Serial.println(" timeout, continuo");
                    break;
                }
            } while (gyro < 2);

            // 2. tara
            Serial.println("\n   Tara offset in corso...");
            double sommaRoll  = 0.0;
            double sommaPitch = 0.0;
            double sommaYaw   = 0.0;
            for (int i = 0; i < IMU_CAMPIONI_TARA; i++) {
                sensors_event_t ev;
                giroscopio.getEvent(&ev);
                sommaRoll  += ev.orientation.z;
                sommaPitch += ev.orientation.y;
                sommaYaw   += ev.orientation.x;
                delay(10);
            }
            offsetRoll  = (float)(sommaRoll  / IMU_CAMPIONI_TARA);
            offsetPitch = (float)(sommaPitch / IMU_CAMPIONI_TARA);
            offsetyaw   = (float)(sommaYaw   / IMU_CAMPIONI_TARA);

            imuPronto = true; 
            segnalaOK();
            Serial.print(" imu offsets: roll= ");
            Serial.print(offsetRoll, 2);
            Serial.print(" ; pitch= ");
            Serial.print(offsetPitch, 2);
            Serial.print(" ; yaw= ");
            Serial.println(offsetyaw, 2);


        } else {
            Serial.println("\n ERRORE (cavi I2C?)");
            segnalaErrore();
        }
        } else {
            Serial.println(" \n [OK] IMU BNO055");
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

                // --- LETTURE A VUOTO per scartare ---
                for(int j=0; j<3; j++) {
                    barometro.readAltitude(1013.25);
                    delay(25);
                }
                
                float sommaAlt = 0.0;
                bool erroreCalibrazione = false;

                for (int i = 0; i < 20; i++) {
                    segnalaCalibrazione(PIN_LED_BLU_PID);
                    
                    float altIstantanea = barometro.readAltitude(1013.25);
                    
                    // VALIDAZIONE HARDWARE (Limiti ASL estremi)
                    if (altIstantanea < -500.0 || altIstantanea > 8000.0) {
                        Serial.println("\n ERRORE: Lettura barometrica impossibile");
                        Serial.print(" Altitudine letta: ");
                        Serial.print(altIstantanea);
                        Serial.println(" m");
                        
                        digitalWrite(PIN_LED_BLU_PID, LOW);
                        erroreCalibrazione = true;
                        break; 
                    }
                    sommaAlt += altIstantanea;
                    delay(25);
                }
                
                if (erroreCalibrazione) {
                    segnalaErrore();
                    continue; 
                }

                digitalWrite(PIN_LED_BLU_PID, LOW);
                digitalWrite(PIN_BUZZER, LOW);

                baroPronto = true;
                float mediaBaroASL = sommaAlt / 20.0;
                set_up_gps_alt = mediaBaroASL;  // Salva per uso in setup
                
                if (set_up_gps_alt < 5.0 && set_up_lidar_alt < 5.0 && set_up_lidar_alt > 0.0 && lidarOk) {
                    
                    G_tara_altitudine = mediaBaroASL - set_up_lidar_alt;
                    Serial.print("OK (Tara ASL corretta da LIDAR: ");
                } else {
                    G_tara_altitudine = mediaBaroASL;
                    Serial.print("OK (Tara ASL standard: ");
                }
                
                Serial.print(G_tara_altitudine, 1);
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
    Serial.println("     TUTTI I SENSORI OPERATIVI          ");
    Serial.println("Inizializzazione servomotori...");

    inizializzazione_servo();
    inizializzazione_motore();
    Serial.println("Settati flap neutri e gas al minimo");

    // Jingle avvio riuscito
    tone(PIN_BUZZER, 800,  120); delay(170);
    tone(PIN_BUZZER, 1200, 120); delay(170);
    tone(PIN_BUZZER, 1800, 200); delay(350);

    Serial.println("Verifica oculare luci di stato...");
    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    digitalWrite(PIN_LED_VERDE_GPS,HIGH);
    digitalWrite(PIN_LED_BLU_PID, HIGH);
    delay(1000);
    digitalWrite(PIN_LED_ROSSO_ALARM,LOW);
    digitalWrite(PIN_LED_VERDE_GPS,LOW);
    digitalWrite(PIN_LED_BLU_PID,LOW);

    tempoPassatoPID = millis();
    Serial.println("\n  >> SISTEMA PRONTO AL VOLO\n");
    delay(500);
}

void  inizializzazione_servo(){
    servoInternoSX.attach(pinIntSX);
    servoInternoDX.attach(pinIntDX);
    servoEsternoSX.attach(pinEstSX);
    servoEsternoDX.attach(pinEstDX);
    servoInternoSX.write(CENTRO_SERVO);
    servoInternoDX.write(CENTRO_SERVO);
    servoEsternoSX.write(CENTRO_SERVO);
    servoEsternoDX.write(CENTRO_SERVO);
}

void inizializzazione_motore(){
    motore.attach(pinMotore);
    motore.writeMicroseconds(GAS_NEUTRO);
}

void loop()
{
    comandi_da_terra(); 
    gestisciAlimentazione();
    gestisciSchianto();
    verifica_drone_in_volo();

    // 1. lettura GPS
    while (GPS_SERIAL.available() > 0) {
        gps.encode(GPS_SERIAL.read());
    }

    // 2. monitoraggio voltaggio teensy e motore
    diagnosticaServi();

    // 3. lettura giroscopio (IMU)
    sensors_event_t event;
    giroscopio.getEvent(&event);
    float angoloPitch = event.orientation.y - offsetPitch;
    float angoloRoll  = event.orientation.z - offsetRoll;
    float angoloYaw   = event.orientation.x;

    // 5. lettura barometro
    G_altitudine_baro = barometro.readAltitude(1013.25) - G_tara_altitudine;
    
    // 5b. AGGIORNA LIDAR E ALTITUDINE FUSA
    aggiornaLidar();
    altitudine();

    // 6. controllo temperature
    temp_aria_barometro = barometro.temperature;
    float voltaggio_Sensore_motore = analogRead(PIN_T_motore) * (3.3 / 1023.0);
    Global_Temperatura_motore = (voltaggio_Sensore_motore - 0.5) * 100.0;


    // 7. pitot – lettura velocità aria
    int lettura_dal_pin_pitot= analogRead(PIN_ARIA);
    lettura_dal_pin_pitot = constrain(lettura_dal_pin_pitot, 0, 1023);
    float Velocita_pitot_Ms = 0.0;
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
    bool gpsSpeedValido = gps.speed.isValid();
    if (gpsSpeedValido) {
        velocita_gps_Ms = gps.speed.kmph() / 3.6;
    }

    if (gpsSpeedValido && Velocita_pitot_Ms > 0.1f) {
        G_Velocità_MS = (alpha_vel * Velocita_pitot_Ms) + ((1.0 - alpha_vel) * velocita_gps_Ms);
    } else if (gpsSpeedValido) {
        G_Velocità_MS = velocita_gps_Ms;
    } else {
        G_Velocità_MS = Velocita_pitot_Ms; 
    }

    aggiornaNavigazione(angoloYaw);

    // 9. PREPARAZIONE DATI MOTORE
    float targetVelocita = 0.0;
    int gasDiBase = 0;
    if (G_distanzaDalTarget > DISTANZA_FRENATA) {
        targetVelocita = VELOCITA_CROCIERA;
        gasDiBase= GAS_CROCIERA;
    } else {
        targetVelocita = VELOCITA_AVVICINAMENTO;
        gasDiBase= 1250;
    }

    int correzionePitch  = 0;
    int correzioneRoll   = 0;
    int comandoGasFinale = GAS_NEUTRO;

    // 1 = Manuale, 2 = Auto, 3 = Failsafe
    if (ricevente.read(&canaliRC[0], &failsafe, &pacchettoPerso)) {
        if (statoSchiantoRilevato && canaliRC[4] < 992) {
            statoSchiantoRilevato = false;
            schiantoBloccato = false;
            droneInVolo = false;           
            inizializzazione_servo();

            statoPrecedenteInterni = true;
            statoPrecedenteEsterni = true;

            noTone(PIN_BUZZER); 
            tone(PIN_BUZZER, 1000, 100); 
            delay(150);
            tone(PIN_BUZZER, 1500, 100);
            Serial.println("!!! SBLOCCO EMERGENZA ESEGUITO DA RADIO !!! Servi Riarmati e centrati.");
        }

        if (canaliRC[4] < 992) {
            global_modalitaVolo = 1; 
        } else {
            if(droneInVolo  && statoSchiantoRilevato == false){
                global_modalitaVolo = 2; 
            }
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

    if (!schiantoBloccato && global_modalitaVolo == 1 && !failsafe) {
        // Gas: da 172-1811 a GAS_NEUTRO-GAS_MASSIMO (limiti meccanici motore)
        comandoGasFinale = constrain(map(canaliRC[2], 172, 1811, GAS_NEUTRO, GAS_MASSIMO), GAS_NEUTRO, GAS_MASSIMO);
        correzioneRoll   = constrain(map(canaliRC[0], 172, 1811, -MAX_ROLL,   MAX_ROLL),   -MAX_ROLL,  MAX_ROLL);
        correzionePitch  = constrain(map(canaliRC[1], 172, 1811,  MAX_PITCH, -MAX_PITCH),  -MAX_PITCH, MAX_PITCH);
        if (statoAttuale != ultimoStatoStampato) {
            Serial.println("Volo: MANUALE (Comandi diretti dal radiocomando)");
            ultimoStatoStampato = statoAttuale;
        }
    } else if (!schiantoBloccato && (global_modalitaVolo == 2 || failsafe)) {
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
        calcolaPID(ALTITUDINE_TARGET, G_targetRoll,angoloPitch, angoloRoll,G_Velocità_MS *3.6, targetVelocita,gasDiBase,correzionePitch, correzioneRoll, comandoGasFinale);
    }
    int gasEffettivo = GAS_NEUTRO;  

    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO);
        gasEffettivo = GAS_NEUTRO;
    } else {
        gestisci_allarmi();
        applicaMixer4Servi(correzionePitch, correzioneRoll);

        int limiteTermico = gasMaxTermico();
        if (comandoGasFinale > limiteTermico && sistema_sicurezza_temp) {
            comandoGasFinale = limiteTermico;
        }
        motore.writeMicroseconds(comandoGasFinale);
        gasEffettivo = comandoGasFinale;
    }
    inviaTelemetria(
    angoloPitch, angoloRoll, angoloYaw,
    Velocita_pitot_Ms * 3.6f,
    velocita_gps_Ms   * 3.6f,
    G_Velocità_MS * 3.6f,
    correzionePitch, correzioneRoll, gasEffettivo);
}
//  MIXER 4 SERVI
void applicaMixer4Servi(int pitch, int roll){
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
        return;
    }
    // ── Attach/detach automatico 
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

    // ── Limiti di sicurezza 
    posIntSX = constrain(posIntSX, 45, 135);
    posIntDX = constrain(posIntDX, 45, 135);
    posEstSX = constrain(posEstSX, 45, 135);
    posEstDX = constrain(posEstDX, 45, 135);

    // ── Comando fisico 
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
        G_distanzaDalTarget = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);

        // Angolo in GRADI (0=Nord, 90=Est, 180=Sud, 270=Ovest) verso il target
        G_rottaVersoTarget = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);

        // Differenza tra direzione target e direzione attuale (yaw)
        G_errore_rotta = G_rottaVersoTarget - angoloYaw;

        // Via più breve per girare (normalizzazione ±180°)
        if(G_errore_rotta >  180.0) {
            G_errore_rotta -= 360.0;
        }else if (G_errore_rotta < -180.0) {
            G_errore_rotta += 360.0;
        }
        // L1 Calcola l'accelerazione laterale necessaria per curvare  verso la rotta: a_lat = 2*V^2/l1 *sin(eta)
        float velPerCalcolo = max(G_Velocità_MS, 1.0f);
        float L1 = max(velPerCalcolo * 4.0f, 1.0f); 
        float eta = radians(G_errore_rotta);
        float a_lat = (2 * velPerCalcolo * velPerCalcolo / L1) * sin(eta);
        float rollRad = atan(a_lat / 9.81);
        G_targetRoll = constrain(degrees(rollRad), -MAX_ROLL, MAX_ROLL);

        Serial.print("Target a: ");
        Serial.print(G_distanzaDalTarget);
        Serial.print(" metri | Direzione bussola: ");
        Serial.print(G_rottaVersoTarget);
        Serial.print(" gradi | Errore rotta: ");
        Serial.print(G_errore_rotta);
        Serial.print(" gradi | Target Roll calcolato: ");
        Serial.println(G_targetRoll);
    } else {
        Serial.println("GPS: In attesa di segnale valido (FIX)...");
    }
}

//  DIAGNOSTICA SERVI
void diagnosticaServi(){
    if (!servo_sicurezza) {
        estSX_Ok = estDX_Ok = intSX_Ok = intDX_Ok = true;
        return;
    }
    static int consecutiveErrors[4] = {0, 0, 0, 0};
    float mA = 0.0f;

    mA = sensoreEstSX.getCurrent_mA();
    if (mA < 0.5f || mA > 2500.0f) {
        consecutiveErrors[0]++;
        if (consecutiveErrors[0] > 5) {
            estSX_Ok = false;
            Serial.println("WARN: ServoEstSX anomalia corrente persistente!");
        }
    } else {
        consecutiveErrors[0] = 0;
        estSX_Ok = true;
    }
    // Servo Esterno DX
    mA = sensoreEstDX.getCurrent_mA();
    if (mA < 0.5f || mA > 2500.0f) {
        consecutiveErrors[1]++;
        if (consecutiveErrors[1] > 5) {
            estDX_Ok = false;
            Serial.println("WARN: ServoEstDX anomalia corrente persistente!");
        }
    } else {
        consecutiveErrors[1] = 0;
        estDX_Ok = true;
    }
    // Servo Interno SX
    mA = sensoreIntSX.getCurrent_mA();
    if (mA < 0.5f || mA > 2500.0f) {
        consecutiveErrors[2]++;
        if (consecutiveErrors[2] > 5) {
            intSX_Ok = false;
            Serial.println("WARN: ServoIntSX anomalia corrente persistente!");
        }
    } else {
        consecutiveErrors[2] = 0;
        intSX_Ok = true;
    }
    // Servo Interno DX
    mA = sensoreIntDX.getCurrent_mA();
    if (mA < 0.5f || mA > 2500.0f) {
        consecutiveErrors[3]++;
        if (consecutiveErrors[3] > 5) {
            intDX_Ok = false;
            Serial.println("WARN: ServoIntDX anomalia corrente persistente!");
        }
    } else {
        consecutiveErrors[3] = 0;
        intDX_Ok = true;
    }
}

//  GESTIONE LUCI DI STATO
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

//  INVIO TELEMETRIA COMPLETA (Formato CSV)
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
        TELEMETRIA.print(G_altitudine, 1); TELEMETRIA.print(","); // 13. Altitudine relativa
        
        // --- VELOCITÀ ---
        TELEMETRIA.print(velPitotKmh, 1);          TELEMETRIA.print(","); // 14. Velocità Aria (Pitot)
        TELEMETRIA.print(velGpsKmh, 1);            TELEMETRIA.print(","); // 15. Velocità Suolo (GPS)
        TELEMETRIA.print(velStimataKmh, 1);        TELEMETRIA.print(","); // 16. Velocità Fusa (Pitot+GPS)
        
        // --- NAVIGAZIONE ---
        TELEMETRIA.print(G_distanzaDalTarget, 0); TELEMETRIA.print(","); // 17. Distanza target (m)
        TELEMETRIA.print(G_rottaVersoTarget, 1);  TELEMETRIA.print(","); // 18. Rotta target (Gradi)
        TELEMETRIA.print(G_targetRoll, 1);        TELEMETRIA.print(","); // 19. Rollio comandato da L1
        
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
//  CALCOLO PID
void calcolaPID(float targetAltitudine, float targetRoll,
                float pitchReale, float rollReale,
                float velocitaAttuale, float targetVelocita,
                int gasDiBase,
                int &comandoPitchOut, int &comandoRollOut, int &comandoGasOut)
{
    // 1. CALCOLO DEL TEMPO
    unsigned long tempoAttuale = millis();
    float dt = (tempoAttuale - tempoPassatoPID) / 1000.0;

    if (dt <= 0.001) return; // Evita divisioni per zero
    if (dt > 0.5) dt = 0.5;  // Evita lag improvvisi
    tempoPassatoPID = tempoAttuale;

    // 2. PID ALTITUDINE
    float targetPitch_Auto = 0.0;
    int gasCorrente = gasDiBase; 

    if (G_altitudine > ALTEZZA_MAX) {
        targetPitch_Auto = -8.0;
        gasCorrente = GAS_MINIMO; 
        pid_sommaErroriAlt =  0.0;  
        pid_errorePassatoAlt=  0.0;
    } else if (G_altitudine < ALTEZZA_MIN) {
        targetPitch_Auto = 12.0;
        gasCorrente = GAS_MASSIMO - 10; 
        pid_sommaErroriAlt =  0.0;
        pid_errorePassatoAlt =  0.0;
    } else {
        float erroreAltitudine = targetAltitudine - G_altitudine;
        erroreAltitudine = constrain(erroreAltitudine, -20.0, 20.0);

        float P_alt = Kp_alt * erroreAltitudine;

        pid_sommaErroriAlt += erroreAltitudine * dt;
        pid_sommaErroriAlt  = constrain(pid_sommaErroriAlt, -20.0, 20.0);
        float I_alt = Ki_alt * pid_sommaErroriAlt;

        float D_alt = Kd_alt * ((erroreAltitudine - pid_errorePassatoAlt) / dt);
        pid_errorePassatoAlt = erroreAltitudine;

        targetPitch_Auto = constrain(P_alt + I_alt + D_alt, -10.0, 15.0);
    }

    // 3. PID PITCH
    float errorePitch = targetPitch_Auto - pitchReale;

    float P_Pitch = Kp_pitch * errorePitch;

    pid_sommaErroriPitch += errorePitch * dt;
    pid_sommaErroriPitch  = constrain(pid_sommaErroriPitch, -40.0, 40.0);
    float I_Pitch = Ki_pitch * pid_sommaErroriPitch;

    float D_Pitch = Kd_pitch * ((errorePitch - pid_errorePassatoPitch) / dt);
    pid_errorePassatoPitch = errorePitch;

    comandoPitchOut = (int)(P_Pitch + I_Pitch + D_Pitch);
    comandoPitchOut = constrain(comandoPitchOut, -MAX_PITCH, MAX_PITCH);

    // 4. PID ROLL
    float erroreRoll = targetRoll - rollReale;  

    float P_Roll = Kp_roll * erroreRoll;

    pid_sommaErroriRoll += erroreRoll * dt;
    pid_sommaErroriRoll  = constrain(pid_sommaErroriRoll, -40.0, 40.0);
    float I_Roll = Ki_roll * pid_sommaErroriRoll;

    float D_Roll = Kd_roll * ((erroreRoll - pid_errorePassatoRoll) / dt);
    pid_errorePassatoRoll = erroreRoll;

    comandoRollOut = (int)(P_Roll + I_Roll + D_Roll);
    comandoRollOut  = constrain(comandoRollOut,  -MAX_ROLL,  MAX_ROLL);

    // 5. AUTOTHROTTLE (PID velocità)
    float erroreVel = targetVelocita - velocitaAttuale;

    float P_vel = Kp_vel * erroreVel;

    pid_sommaErroriVel += erroreVel * dt;
    pid_sommaErroriVel  = constrain(pid_sommaErroriVel, -30.0, 30.0);
    float I_vel = Ki_vel * pid_sommaErroriVel;

    float D_vel = Kd_vel * ((erroreVel - pid_errorePassatoVel) / dt);
    pid_errorePassatoVel = erroreVel;

    int gasCalcolato = gasCorrente + (int)(P_vel + I_vel + D_vel);
    comandoGasOut = constrain(gasCalcolato, GAS_MINIMO, GAS_MASSIMO);
}


void verifica_drone_in_volo() {          
    if (!droneInVolo) {
        bool velocitaSufficiente   = (G_Velocità_MS > SOGLIA_VELO_DECOLLO_MS);
        bool altitudineSufficiente = (G_altitudine > SOGLIA_ALT_DECOLLO_M);
        bool decolloValido         = altitudineSufficiente && velocitaSufficiente;

        if (decolloValido) {
            if (timestampDecollo == 0) {
                timestampDecollo = millis();
            }
            if (millis() - timestampDecollo >= TEMPO_DECOLLO_SICURO_MS) {
                droneInVolo = true;
                timestampDecollo = 0;   
                Serial.println("STATO: Decollo confermato, drone in volo.");
            }
        } else {
            timestampDecollo = 0;       
        }
    }
}

void gestisciSchianto() {
    if (!schianto_sicurezza) {
        statoSchiantoRilevato = false;   
        return;
    }
    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO); 
        return; 
    }

    if (!droneInVolo) return; 

    // 3. Lettura G-Force
    imu::Vector<3> accel = giroscopio.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    float accelerazioneTotale = sqrt((accel.x() * accel.x()) + (accel.y() * accel.y()) + (accel.z() * accel.z()));

    
    if (accelerazioneTotale > SOGLIA_G_SCHIANTO) {
        contatoreImpatto++;
        if (contatoreImpatto >= SEMPLE_VALORI_SCHIANTO) {   
                statoSchiantoRilevato = true;
                schiantoBloccato = true;
                contatoreImpatto = 0;
                motore.writeMicroseconds(GAS_NEUTRO); 
                servoInternoSX.detach();
                servoInternoDX.detach();
                servoEsternoSX.detach();
                servoEsternoDX.detach();
                Serial.println("!!! IMPATTO RILEVATO DALL'IMU !!!");
                Serial.print("Forza: "); 
                Serial.print(accelerazioneTotale); 
                Serial.println(" m/s^2");
                
                digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
                digitalWrite(PIN_LED_VERDE_GPS, HIGH);
                digitalWrite(PIN_LED_BLU_PID, HIGH);
                tone(PIN_BUZZER, 2000); 
            }
    } else {
        contatoreImpatto = 0;  
    }
}

void aggiornaLidar() {
  // Solo se sotto margine di sicurezza (atterraggio/volo basso)
  if (G_altitudine_baro > ALTEZZA_MAX_LIDAR) {
    while (Serial2.available()) {
      Serial2.read();  // Svuota buffer
    }
    // Mantieni ultimo valore valido per continuità
    return;
  }

  static uint8_t buffer[9];
  
  while (Serial2.available() >= 9) {
    if (Serial2.read() == 0x59 && Serial2.read() == 0x59) {
      buffer[0] = 0x59;
      buffer[1] = 0x59;
      for (int i = 2; i < 9; i++) {
        buffer[i] = Serial2.read();
      }

      // Checksum
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) {
        checksum += buffer[i];
      }
      if (checksum != buffer[8]) continue;

      uint16_t distanza_cm = buffer[2] | ((uint16_t)buffer[3] << 8);
      float distanza_m = distanza_cm / 100.0f;
      
      // Filtro passa-basso: inizializza correttamente al primo valore
      if (G_altitudine_lidar < 0.0f) {
        G_altitudine_lidar = distanza_m;  // Prima lettura
      } else {
        G_altitudine_lidar = ALPHA_LIDAR * distanza_m + (1.0f - ALPHA_LIDAR) * G_altitudine_lidar;
      }
      set_up_lidar_alt = G_altitudine_lidar;  // Salva per setup
      return;
    }
  }
}

void altitudine() {
  // Usa LIDAR in volo basso, fallback a barometro
  if (lidarOk && G_altitudine_lidar > 0.0f && G_altitudine_baro < ALTEZZA_MAX_LIDAR) {
    G_altitudine = G_altitudine_lidar;  // LIDAR ha priorità in basso
  } else {
    G_altitudine = G_altitudine_baro;    // Barometro come default
  }
}

void gestisciAlimentazione() {
    if (!alimentazione_sicurezza) {   
        batteriaBassa_teensy = false;
        batteriaBassa_motore = false;
        return;
    }
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

int gasMaxTermico() {
    if (Global_Temperatura_motore <= T_MOTORE_THROTTLE_START) {
        return GAS_MASSIMO;   
    }
    if (Global_Temperatura_motore >= T_MOTORE_THROTTLE_END) {
        return GAS_MINIMO;
    }

    float t = (Global_Temperatura_motore - T_MOTORE_THROTTLE_START) /
              (T_MOTORE_THROTTLE_END   - T_MOTORE_THROTTLE_START);  
    int limite = (int)(GAS_MASSIMO - t * (GAS_MASSIMO - GAS_MINIMO));
    return limite;
}

void comandi_da_terra() {
    static String buffer = "";  

    HardwareSerial* fonti[] = { &TELEMETRIA, &Serial };
    for (auto* porta : fonti) {
        while (porta->available()) {
            char c = porta->read();
            if (c == '\n') {
                segnalaOK();
                elaboraComando(buffer);
                buffer = "";
            } else {
                buffer += c;
                if (buffer.length() > 64) buffer = "";  
            }
        }
    }
}

//  PARSER COMANDI
void elaboraComando(const String& cmd) {
    if (!cmd.startsWith("CMD:")) return;

    int sep = cmd.indexOf(':', 4);
    if (sep < 0) return;

    String campo     = cmd.substring(4, sep);
    String valoreStr = cmd.substring(sep + 1);   
    int    val       = valoreStr.toInt();        

    //Servi: posizione 
    if (campo == "SERVO_ISX") {
        servoInternoSX.write(constrain(val, 45, 135));

    } else if (campo == "SERVO_IDX") {
        servoInternoDX.write(constrain(val, 45, 135));

    } else if (campo == "SERVO_ESX") {
        servoEsternoSX.write(constrain(val, 45, 135));

    } else if (campo == "SERVO_EDX") {
        servoEsternoDX.write(constrain(val, 45, 135));

    //Servi: attach/detach
    } else if (campo == "SERVO_ISX_ATTACH") {
        servoInternoSX.attach(pinIntSX);

    } else if (campo == "SERVO_IDX_ATTACH") {
        servoInternoDX.attach(pinIntDX);

    } else if (campo == "SERVO_ESX_ATTACH") {
        servoEsternoSX.attach(pinEstSX);  

    } else if (campo == "SERVO_EDX_ATTACH") {
        servoEsternoDX.attach(pinEstDX);  

    } else if (campo == "SERVO_ISX_DETACH") {
        servoInternoSX.detach();

    } else if (campo == "SERVO_IDX_DETACH") {
        servoInternoDX.detach();

    } else if (campo == "SERVO_ESX_DETACH") {
        servoEsternoSX.detach();

    } else if (campo == "SERVO_EDX_DETACH") {
        servoEsternoDX.detach();

    } else if (campo == "RELE_ON") {
        digitalWrite(PIN_RELE, HIGH);
        relèAttivato = true;

    } else if (campo == "RELE_OFF") {
        digitalWrite(PIN_RELE, LOW);
        relèAttivato = false;

    } else if (campo == "SICUREZZA_SCHIANTO_ON") {
        schianto_sicurezza = true;

    } else if (campo == "SICUREZZA_SCHIANTO_OFF") {
        schianto_sicurezza = false;

    } else if (campo == "SICUREZZA_ALIMENTAZIONE_ON") {
        alimentazione_sicurezza = true;   

    } else if (campo == "SICUREZZA_ALIMENTAZIONE_OFF") {
        alimentazione_sicurezza = false;   

    } else if (campo == "SICUREZZA_SERVI_ON") {
        servo_sicurezza = true;

    } else if (campo == "SICUREZZA_SERVI_OFF") {
        servo_sicurezza = false;
    }else if (campo == "SICUREZZA_TEMP_ON") {
        sistema_sicurezza_temp = true;

    } else if (campo == "SICUREZZA_TEMP_OFF") {
        sistema_sicurezza_temp = false;

    // Gas 
    } else if (campo == "GAS") {
        if (global_modalitaVolo == 1) {
            motore.writeMicroseconds(constrain(val, GAS_NEUTRO, GAS_MASSIMO));   
        }

    //Modalità di volo
    } else if (campo == "MODO") {
        if (!failsafe && val >= 1 && val <= 2) {
            global_modalitaVolo = val;
        }
    }else if (campo == "SET_LATITUDE") {
        float lat = valoreStr.toFloat();
        if (lat >= -90.0 && lat <= 90.0) {
            TARGET_LAT = lat;
        }

    } else if (campo == "SET_LONGITUDE") {
        float lon = valoreStr.toFloat();
        if (lon >= -180.0 && lon <= 180.0) {
            TARGET_LON = lon;
        }

    }else if (campo == "SET_ALTITUDE") {
        float alt = valoreStr.toFloat();
        if (alt >= 0.0 && alt <= 500.0) {
            ALTITUDINE_TARGET = alt;
        }
    }
}