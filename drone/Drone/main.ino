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

//  SENSORI
TinyGPSPlus gps;
Adafruit_BNO055  giroscopio = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX barometro;
Bitcraze_PMW3901 flusso_ottico(25);

const int PIN_T_motore=A12;
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
unsigned long timerTelemetriaDiag = 0;
unsigned long numeroPacchettoTEL1 = 0;
uint8_t contatorePacchettoDiag = 0;   // cicla 0..2 tra TEL2 / TEL3 / TEL4
bool forzaInvioDiagnostica = false;   // impostato da REQ_DIAG

//  RICEVENTE SBUS (FrSky)
SBUS   ricevente(Serial7);
uint16_t canaliRC[16];
bool failsafe   = false;
bool pacchettoPerso = false;

//  PITOT (VELOCITÀ ARIA)
const int   PIN_ARIA  = A0;
const float DENSITA_ARIA = 1.225;  
float VALORE_ZERO  = 0.0;    
const float FATTORE_CONVERSIONE_PA = 3.22;

// --- Diagnostica pitot (per telemetria) ---
int   G_pitot_raw = 0;
float G_pitot_differenza = 0.0f;
bool  G_pitot_valido = false;

//  SERVO
const int pinIntSX  = 6;
const int pinIntDX  = 22;
const int pinEstSX  = 23;
const int pinEstDX  = 24;
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

// --- Diagnostica corrente servi (per telemetria, salvata da diagnosticaServi()) ---
float G_correnteIntSX = 0.0f;
float G_correnteIntDX = 0.0f;
float G_correnteEstSX = 0.0f;
float G_correnteEstDX = 0.0f;

//  COSTANTI
const float VALORE_BATT_MOTORE_BASSA = 11.8f;
const float VALORE_BATT_TEENSY_BASSA = 4.9f;

const int SOGLIA_G_SCHIANTO=50;
const int SEMPLE_VALORI_SCHIANTO=3;

const int CENTRO_SERVO  = 90;
const int MAX_ROLL = 35;
const int MAX_PITCH  = 20;

// Questi limiti erano "const": resi modificabili da terra (entro validazione),
// come richiesto nella sezione "PARAMETRI OPERATIVI".
float ALTEZZA_MAX = 120;
float ALTEZZA_MIN = 10;

const int GAS_NEUTRO = 1000;
const int GAS_MASSIMO = 2000;
const int GAS_MINIMO = 1200;   
const int GAS_CROCIERA= 1450; 

const float MAX_AIRSPEED_X8 = 45.0f;
float VELOCITA_CROCIERA  = 60.0;       // km/h, ora configurabile da terra
float VELOCITA_AVVICINAMENTO= 45.0;    // km/h, ora configurabile da terra
const float DISTANZA_FRENATA = 150.0; 
float RAGGIO_ACCETTAZIONE_MINIMO = 25.0f; // ora configurabile da terra

const int   IMU_CAMPIONI_TARA = 200;

const float T_MOTORE_THROTTLE_START = 70.0f;  
float T_MOTORE_THROTTLE_END  = 90.0f;   // ora configurabile da terra (limite termico)

int G_limiteGasTermico = GAS_MASSIMO;
int  G_comandoGasPreLimite = GAS_NEUTRO;
bool G_limitazioneTermicaAttiva = false;

const float ALPHA_LIDAR = 0.25f;
const float ALTEZZA_MAX_LIDAR = 6.0f;
const float ALTEZZA_MAX_SENSORE_OTTICO = 4.0f;

const float  alpha_vel= 0.7; 
const float COSTANTE_OTTICA = 0.0012;

//  NAVIGAZIONE
double TARGET_LAT = 41.902782;
double TARGET_LON = 12.496366;
float  ALTITUDINE_TARGET = 40.0;

float G_altitudine_lidar = -1.0;  
float set_up_lidar_alt = 0.0;           
float G_altitudine_baro = 0.0;   
float set_up_gps_alt = 0.0;              
float G_altitudine = 0.0;          

float G_tara_altitudine = 0.0;
float G_targetRoll  = 0.0;
float G_distanzaDalTarget = 0.0;
float G_rottaVersoTarget = 0.0;
float G_errore_rotta  = 0.0;

float G_Velocità_MS = 0.0;
float G_Airspeed_MS = 0.0;   
float G_Groundspeed_MS = 0.0; 

float G_vel_x_optical_sensor = 0.0;
float G_vel_y_optical_sensor = 0.0;
int   G_flow_dx = 0;   // conteggio grezzo flusso ottico (per telemetria)
int   G_flow_dy = 0;

float offsetRoll  = 0.0f;
float offsetPitch = 0.0f;
float offsetyaw   = 0.0f;

// --- Diagnostica IMU estesa (accelerazione lineare, giroscopio, calibrazione) ---
float G_accelX = 0.0f, G_accelY = 0.0f, G_accelZ = 0.0f, G_accelTotale = 0.0f;
float G_gyroX = 0.0f, G_gyroY = 0.0f, G_gyroZ = 0.0f;
uint8_t G_imuCalSys = 0, G_imuCalGyro = 0, G_imuCalAccel = 0, G_imuCalMag = 0;

// --- Diagnostica barometro estesa ---
float G_pressione_baro = 0.0f;

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

// Limiti di validazione per i guadagni PID ricevuti da terra (sicurezza)
const float LIMITE_KP_MAX = 10.0f;
const float LIMITE_KI_MAX = 2.0f;
const float LIMITE_KD_MAX = 5.0f;

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

// --- Diagnostica PID (salvata ad ogni ciclo calcolaPID, per telemetria TEL3) ---
float G_pid_altErrore=0, G_pid_altP=0, G_pid_altI=0, G_pid_altD=0;
float G_pid_targetPitchAuto=0;
float G_pid_pitchErrore=0, G_pid_pitchP=0, G_pid_pitchI=0, G_pid_pitchD=0;
float G_pid_rollErrore=0, G_pid_rollP=0, G_pid_rollI=0, G_pid_rollD=0;
float G_pid_velErrore=0, G_pid_velP=0, G_pid_velI=0, G_pid_velD=0;
float G_targetVelocitaAttuale = 0.0f;


const unsigned long TEMPO_DECOLLO_SICURO_MS = 1500;
const float SOGLIA_VELO_DECOLLO_MS = 5.0;
const float SOGLIA_ALT_DECOLLO_M = 5.0;
unsigned long timestampDecollo = 0;
static int contatoreImpatto = 0;

//  LED DI STATO E ALLARMI
const int PIN_LED_ROSSO_ALARM = 2; // Allarmi come moduli mancanti / Batteria
const int PIN_LED_VERDE_GPS = 3; // GPS Fix e settaggio pitot
const int PIN_LED_BLU_PID  = 4; // Modalità AUTO pid e settaggio barometro
const int PIN_BUZZER = 33;     
const int PIN_RELE = 20;

//  FLAGS STATO SISTEMA
bool imuPronto       = false;
bool flusso_otticoOK = false;
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

bool estSX_Ok = true, 
estDX_Ok = true;
bool intSX_Ok = true, 
intDX_Ok = true;
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
void aggiorna_altitudine();
void aggiorna_velocita(float Velocita_pitot_Ms, float velocita_gps_Ms);
int  gasMaxTermico();   
void velocità_flusso_ottico(float angoloYaw);
void resettaPID();
void aggiornaDiagnosticaIMU();
void inviaAck(const String& campo, const String& valore);
void inviaNack(const String& campo, const String& motivo);

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


        if (!flusso_otticoOK){
            Serial.println("[ ] Flusso Ottico PMW3901 ........... ");
            if (flusso_ottico.begin()) {
                Serial.println("[OK] Flusso Ottico PMW3901");
                flusso_otticoOK = true; 
            } else {
                Serial.println("ERRORE sensore flusso ottico (cavi SPI?)");
                segnalaErrore();
            }
        } else {
            Serial.println("[OK] Flusso Ottico PMW3901");
        }

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
                set_up_gps_alt = mediaBaroASL;  
                
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

    diagnosticaServi();

    sensors_event_t event;
    giroscopio.getEvent(&event);
    float angoloPitch = event.orientation.y - offsetPitch;
    float angoloRoll  = event.orientation.z - offsetRoll;
    float angoloYaw   = event.orientation.x;

    // 2. temperatura motore e barometro
    temp_aria_barometro = barometro.temperature;
    float voltaggio_Sensore_motore = analogRead(PIN_T_motore) * (3.3 / 1023.0);
    Global_Temperatura_motore = (voltaggio_Sensore_motore - 0.5) * 100.0;

    // 3. pitot – lettura velocità aria
    int lettura_dal_pin_pitot = constrain(analogRead(PIN_ARIA), 0, 1023);
    float Velocita_pitot_Ms = 0.0f;
    float differenza= (float)lettura_dal_pin_pitot - VALORE_ZERO;
    // --- salvataggio dati grezzi pitot per diagnostica/telemetria ---
    G_pitot_raw = lettura_dal_pin_pitot;
    G_pitot_differenza = differenza;
    G_pitot_valido = (differenza > 0.0f);
    if (differenza > 0.0f) {
        float pressionePascal = differenza * FATTORE_CONVERSIONE_PA;
        Velocita_pitot_Ms = sqrtf((2.0f * pressionePascal) / DENSITA_ARIA);
    }
    // 4. GPS – lettura velocità GPS
    float velocita_gps_Ms = 0.0f;
    if (gps.speed.isValid()) {
        velocita_gps_Ms = gps.speed.mps();
    } else {
        velocita_gps_Ms = 0.0f;
    }

    // 5. Barometro – lettura altitudine
    G_altitudine_baro = barometro.readAltitude(1013.25f) - G_tara_altitudine;
    G_pressione_baro = barometro.pressure; // Pa, letta dallo stesso ciclo di readAltitude()

    // ─── CORREZIONE ERRORE EVIDENTE ────────────────────────────────────────
    // "altitudine_velocità(...)" era dichiarata/chiamata ma MAI implementata:
    // il codice non avrebbe compilato. Le due funzioni che già esistevano
    // (aggiorna_altitudine / aggiorna_velocita) non venivano mai chiamate,
    // quindi G_altitudine e le velocità fuse non si aggiornavano mai.
    // Ordine corretto: LIDAR -> fusione altitudine -> flusso ottico (che usa
    // l'altitudine per la sua validità) -> fusione velocità (che usa il
    // flusso ottico) -> navigazione (che usa la velocità fusa).
    aggiornaLidar();
    aggiorna_altitudine();
    velocità_flusso_ottico(angoloYaw);
    aggiorna_velocita(Velocita_pitot_Ms, velocita_gps_Ms);
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

    //RESET PID AL CAMBIO DI MODALITÀ 
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
        calcolaPID(ALTITUDINE_TARGET, G_targetRoll,angoloPitch, angoloRoll,G_Airspeed_MS *3.6, targetVelocita,gasDiBase,correzionePitch, correzioneRoll, comandoGasFinale);
    }
    int gasEffettivo = GAS_NEUTRO;  

    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO);
        gasEffettivo = GAS_NEUTRO;
    } else {
        gestisci_allarmi();
        applicaMixer4Servi(correzionePitch, correzioneRoll);

        int limiteTermico = gasMaxTermico();
        G_limiteGasTermico = limiteTermico; 
        G_comandoGasPreLimite = comandoGasFinale;
        if (comandoGasFinale > limiteTermico && sistema_sicurezza_temp) {
            comandoGasFinale = limiteTermico;
            G_limitazioneTermicaAttiva = true;
        } else {
            G_limitazioneTermicaAttiva = false;
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
//  NAVIGAZIONE GPS
void aggiornaNavigazione(float angoloYaw)
{
    if (gps.location.isValid()) {
        // geometria targhet: distanza e rotta verso il waypoint
        G_distanzaDalTarget = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);
        // Angolo in GRADI (0=Nord, 90=Est, 180=Sud, 270=Ovest) verso il target
        G_rottaVersoTarget = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LAT, TARGET_LON);

        float velPerCalcolo = max(G_Groundspeed_MS, 1.0f); // Previene divisioni per zero
        float L1 = max(velPerCalcolo * 4.0f, 1.0f); 
        // arrivo al waypoint se entro un raggio dinamico (minimo 5m, massimo 75% di L1)
        float raggio_accettazione_dinamico = max(RAGGIO_ACCETTAZIONE_MINIMO, L1 * 0.75f);

        if (G_distanzaDalTarget <= raggio_accettazione_dinamico) {
            Serial.println("WAYPOINT RAGGIUNTO! Inizializza passaggio al prossimo target...");
            G_distanzaDalTarget = 0.0f;
            return;
        }
        // calcolo deriva vento
        float rotta_attuale;
        // GPS per capire la vera direzione del moto. 
        // Se la velocità è < 2 m/s il GPS è impreciso sulla direzione, quindi usiamo la bussola (Yaw)
        if (gps.course.isValid() && G_Groundspeed_MS > 2.0f) {
            rotta_attuale = gps.course.deg();
        } else {
            rotta_attuale = angoloYaw;
        }
        // Differenza tra direzione target e direzione attuale (yaw)
        G_errore_rotta = G_rottaVersoTarget - rotta_attuale;

        // ERRORE ROTTA: Via più breve per girare (normalizzazione ±180°)
        if(G_errore_rotta >  180.0) {
            G_errore_rotta -= 360.0;
        }else if (G_errore_rotta < -180.0) {
            G_errore_rotta += 360.0;
        }
        // L1 Calcola l'accelerazione laterale necessaria per curvare  verso la rotta: a_lat = 2*V^2/l1 *sin(eta)
        float eta = radians(G_errore_rotta);
        float a_lat = (2.0f * velPerCalcolo * velPerCalcolo / L1) * sin(eta);
        float rollRad = atan(a_lat / 9.81f);
        G_targetRoll = constrain(degrees(rollRad), -MAX_ROLL, MAX_ROLL);
        //telemetria
        Serial.print("Dist WP: ");
        Serial.print(G_distanzaDalTarget);
        Serial.print("m | Rotta Target: ");
        Serial.print(G_rottaVersoTarget);
        Serial.print("° | Err: ");
        Serial.print(G_errore_rotta);
        Serial.print("° | Target Roll: ");
        Serial.println(G_targetRoll);

    } else {
        Serial.println("GPS: In attesa di segnale valido (FIX 3D)...");
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
    G_correnteEstSX = mA;
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
    G_correnteEstDX = mA;
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
    G_correnteIntSX = mA;
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
    G_correnteIntDX = mA;
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

// Legge calibrazione e giroscopio dal BNO055. Chiamata solo a bassa frequenza
// (dal pacchetto diagnostico TEL4) per non appesantire il loop principale.
void aggiornaDiagnosticaIMU() {
    giroscopio.getCalibration(&G_imuCalSys, &G_imuCalGyro, &G_imuCalAccel, &G_imuCalMag);
    imu::Vector<3> gyro = giroscopio.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    G_gyroX = gyro.x();
    G_gyroY = gyro.y();
    G_gyroZ = gyro.z();
}

void inviaAck(const String& campo, const String& valore) {
    TELEMETRIA.print("ACK:"); TELEMETRIA.print(campo); TELEMETRIA.print(":"); TELEMETRIA.println(valore);
    Serial.print("ACK:");     Serial.print(campo);     Serial.print(":");     Serial.println(valore);
}

void inviaNack(const String& campo, const String& motivo) {
    TELEMETRIA.print("NACK:"); TELEMETRIA.print(campo); TELEMETRIA.print(":"); TELEMETRIA.println(motivo);
    Serial.print("NACK:");     Serial.print(campo);     Serial.print(":");     Serial.println(motivo);
}

void resettaPID() {
    pid_sommaErroriAlt  = 0.0;  pid_errorePassatoAlt = 0.0;
    pid_sommaErroriPitch= 0.0;  pid_errorePassatoPitch= 0.0;
    pid_sommaErroriRoll = 0.0;  pid_errorePassatoRoll = 0.0;
    pid_sommaErroriVel = 0.0;  pid_errorePassatoVel = 0.0;
    tempoPassatoPID = millis();
}

//  INVIO TELEMETRIA COMPLETA (Formato CSV)
//
// TEL1 ("$,")  -> stato + assetto + navigazione + batterie + servi (FORMATO ORIGINALE,
//                 INVARIATO, stessa frequenza di prima ~2Hz, per non rompere il parser
//                 GCS esistente). In coda sono stati aggiunti SOLO 2 campi nuovi
//                 (timestamp millis, contatore pacchetto) — vedi sezione 5 della richiesta.
// TEL2 ("$2,") -> correnti/potenze batterie + dettaglio flusso ottico grezzo + gas pre-limite
// TEL3 ("$3,") -> diagnostica PID completa
// TEL4 ("$4,") -> GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC estesi (4-16)
//
// TEL2/3/4 vengono inviati in round-robin ogni 2 secondi circa (uno alla volta,
// alternati), per non appesantire la banda LoRa: sono dati diagnostici che non
// servono ad alta frequenza. REQ_DIAG forza l'invio immediato del prossimo del ciclo.
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
        TELEMETRIA.print(G_altitudine, 1);         TELEMETRIA.print(","); // 13. Altitudine relativa (fusa)
        
        // --- VELOCITÀ ---
        TELEMETRIA.print(velPitotKmh, 1);             TELEMETRIA.print(","); // 14. Velocità Aria (Pitot grezza)
        TELEMETRIA.print(velGpsKmh, 1);               TELEMETRIA.print(","); // 15. Velocità Suolo (GPS grezza)
        TELEMETRIA.print(G_Airspeed_MS * 3.6f, 1);    TELEMETRIA.print(","); // 16. Airspeed fusa (usata dal PID)
        TELEMETRIA.print(G_Groundspeed_MS * 3.6f, 1); TELEMETRIA.print(","); // 17. Groundspeed fusa (flusso ottico/GPS)
        
        // --- NAVIGAZIONE ---
        TELEMETRIA.print(G_distanzaDalTarget, 0); TELEMETRIA.print(","); // 18. Distanza target (m)
        TELEMETRIA.print(G_rottaVersoTarget, 1);  TELEMETRIA.print(","); // 19. Rotta target (Gradi)
        TELEMETRIA.print(G_targetRoll, 1);        TELEMETRIA.print(","); // 20. Rollio comandato da L1
        
        // --- INPUT RADIOCOMANDO (Grezzi 172-1811) ---
        TELEMETRIA.print(canaliRC[1]);             TELEMETRIA.print(","); // 21. RC Pitch
        TELEMETRIA.print(canaliRC[0]);             TELEMETRIA.print(","); // 22. RC Roll
        TELEMETRIA.print(canaliRC[2]);             TELEMETRIA.print(","); // 23. RC Gas
        
        // --- OUTPUT PID/MIXER ---
        TELEMETRIA.print(outPitch);                TELEMETRIA.print(","); // 24. PID Pitch Out
        TELEMETRIA.print(outRoll);                 TELEMETRIA.print(","); // 25. PID Roll Out
        TELEMETRIA.print(outGas);                  TELEMETRIA.print(","); // 26. PID Gas Out (effettivo, post-limite termico)
        
        // --- POSIZIONE FISICA ATTUALE SERVI (Gradi 45-135) ---
        TELEMETRIA.print(servoInternoSX.read());   TELEMETRIA.print(","); // 27. Pos Servo Int SX
        TELEMETRIA.print(servoInternoDX.read());   TELEMETRIA.print(","); // 28. Pos Servo Int DX
        TELEMETRIA.print(servoEsternoSX.read());   TELEMETRIA.print(","); // 29. Pos Servo Est SX
        TELEMETRIA.print(servoEsternoDX.read());   TELEMETRIA.print(","); // 30. Pos Servo Est DX

        // --- TEMPERATURE ---
        TELEMETRIA.print(Global_Temperatura_motore, 1); TELEMETRIA.print(","); // 31. Temp Motore
        TELEMETRIA.print(temp_aria_barometro, 1);       TELEMETRIA.print(","); // 32. Temp Avionica/Teensy
        
        // --- SATELLITI E COORDINATE GPS ---
        if (gps.location.isValid()) {
            TELEMETRIA.print(gps.satellites.value());  TELEMETRIA.print(","); // 33. Numero Satelliti
            TELEMETRIA.print(gps.location.lat(), 6);   TELEMETRIA.print(","); // 34. Latitudine
            TELEMETRIA.print(gps.location.lng(), 6);   // 35. Longitudine 
        } else {
            TELEMETRIA.print("0,0.000000,0.000000"); // Satelliti=0, Lat=0, Lon=0
        }
        TELEMETRIA.print(",");
        TELEMETRIA.print(relèAttivato ? "1" : "0");   // 36. Relè attivato
        TELEMETRIA.print(",");

        // --- FLUSSO OTTICO (velocità stimata) ---
        TELEMETRIA.print(G_vel_x_optical_sensor, 2);  TELEMETRIA.print(","); // 37. Vel X flusso ottico [m/s]
        TELEMETRIA.print(G_vel_y_optical_sensor, 2);  TELEMETRIA.print(","); // 38. Vel Y flusso ottico [m/s]

        // --- STATO SENSORI (bitmask: b0=flusso ottico OK, b1=LIDAR OK, b2=pacchetto SBUS perso) ---
        int statoSensori = 0;
        if (flusso_otticoOK)  statoSensori += 1;
        if (lidarOk)          statoSensori += 2;
        if (pacchettoPerso)   statoSensori += 4;
        TELEMETRIA.print(statoSensori);               TELEMETRIA.print(","); // 39. Bitmask stato sensori

        // --- NAVIGAZIONE (errore rotta) ---
        TELEMETRIA.print(G_errore_rotta, 1);          TELEMETRIA.print(","); // 40. Errore rotta [°]

        // --- PROTEZIONE TERMICA MOTORE ---
        TELEMETRIA.print(G_limiteGasTermico);         TELEMETRIA.print(","); // 41. Limite gas termico [µs]

        // --- ALTITUDINI GREZZE (per diagnostica sensori, prima della fusione) ---
        TELEMETRIA.print(G_altitudine_lidar, 2);      TELEMETRIA.print(","); // 42. Altitudine LIDAR grezza [m]
        TELEMETRIA.print(G_altitudine_baro, 2);       TELEMETRIA.print(","); // 43. Altitudine Baro grezza [m]

        // --- NUOVO: timestamp e contatore pacchetto (per rilevare pacchetti persi lato GCS) ---
        TELEMETRIA.print(tempoAttuale);               TELEMETRIA.print(","); // 44. millis() al momento dell'invio
        TELEMETRIA.print(numeroPacchettoTEL1);                                // 45. Numero progressivo pacchetto TEL1

        TELEMETRIA.println(); 
        numeroPacchettoTEL1++;

        // ── Pacchetti diagnostici supplementari (round-robin ogni ~2s, o forzati da REQ_DIAG) ──
        if (forzaInvioDiagnostica || (tempoAttuale - timerTelemetriaDiag > 2000)) {
            timerTelemetriaDiag = tempoAttuale;
            forzaInvioDiagnostica = false;

            if (contatorePacchettoDiag == 0) {
                // TEL2: correnti/potenze batterie + flusso ottico grezzo + gas pre-limite
                float pMotore  = sensoreMotore.getPower_mW();
                float pTeensy  = sensoreTeensy.getPower_mW();
                TELEMETRIA.print("$2,");
                TELEMETRIA.print(sensoreMotore.getCurrent_mA(), 1); TELEMETRIA.print(","); // corrente motore mA
                TELEMETRIA.print(sensoreTeensy.getCurrent_mA(), 1); TELEMETRIA.print(","); // corrente Teensy mA
                TELEMETRIA.print(G_correnteIntSX, 1);               TELEMETRIA.print(","); // corrente servo Int SX mA
                TELEMETRIA.print(G_correnteIntDX, 1);               TELEMETRIA.print(","); // corrente servo Int DX mA
                TELEMETRIA.print(G_correnteEstSX, 1);               TELEMETRIA.print(","); // corrente servo Est SX mA
                TELEMETRIA.print(G_correnteEstDX, 1);               TELEMETRIA.print(","); // corrente servo Est DX mA
                TELEMETRIA.print(pMotore, 1);                       TELEMETRIA.print(","); // potenza motore mW
                TELEMETRIA.print(pTeensy, 1);                       TELEMETRIA.print(","); // potenza Teensy mW
                TELEMETRIA.print(batteriaBassa_motore ? "1" : "0"); TELEMETRIA.print(","); // batteria motore bassa
                TELEMETRIA.print(batteriaBassa_teensy ? "1" : "0"); TELEMETRIA.print(","); // batteria Teensy bassa
                TELEMETRIA.print(G_flow_dx);                        TELEMETRIA.print(","); // flusso ottico dx grezzo
                TELEMETRIA.print(G_flow_dy);                        TELEMETRIA.print(","); // flusso ottico dy grezzo
                TELEMETRIA.print(G_comandoGasPreLimite);            TELEMETRIA.print(","); // gas comandato pre-limite termico
                TELEMETRIA.print(G_limitazioneTermicaAttiva ? "1" : "0"); // limitazione termica attiva
                TELEMETRIA.println();

            } else if (contatorePacchettoDiag == 1) {
                // TEL3: diagnostica PID completa
                TELEMETRIA.print("$3,");
                TELEMETRIA.print(G_pid_altErrore, 2);    TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_altP, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_altI, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_altD, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_targetPitchAuto, 2); TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_pitchErrore, 2);  TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_pitchP, 2);       TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_pitchI, 2);       TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_pitchD, 2);       TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_rollErrore, 2);   TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_rollP, 2);        TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_rollI, 2);        TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_rollD, 2);        TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_velErrore, 2);    TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_velP, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_velI, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_velD, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(ALTITUDINE_TARGET, 1);  TELEMETRIA.print(",");
                TELEMETRIA.print(G_targetVelocitaAttuale, 1);
                TELEMETRIA.println();

            } else {
                // TEL4: GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC estesi
                aggiornaDiagnosticaIMU();
                TELEMETRIA.print("$4,");
                // GPS esteso
                TELEMETRIA.print(gps.altitude.isValid() ? gps.altitude.meters() : -1.0, 1); TELEMETRIA.print(",");
                TELEMETRIA.print(gps.course.isValid() ? gps.course.deg() : -1.0, 1);        TELEMETRIA.print(",");
                TELEMETRIA.print(gps.speed.isValid() ? "1" : "0");   TELEMETRIA.print(",");
                TELEMETRIA.print(gps.course.isValid() ? "1" : "0");  TELEMETRIA.print(",");
                TELEMETRIA.print(gps.location.isValid() ? "1" : "0"); TELEMETRIA.print(",");
                // Barometro esteso
                TELEMETRIA.print(G_pressione_baro, 1);   TELEMETRIA.print(","); // Pa
                TELEMETRIA.print(G_tara_altitudine, 1);  TELEMETRIA.print(","); // tara ASL
                TELEMETRIA.print(baroPronto ? "1" : "0"); TELEMETRIA.print(",");
                // Pitot grezzo
                TELEMETRIA.print(G_pitot_raw);           TELEMETRIA.print(",");
                TELEMETRIA.print(VALORE_ZERO, 1);        TELEMETRIA.print(",");
                TELEMETRIA.print(G_pitot_differenza, 1); TELEMETRIA.print(",");
                TELEMETRIA.print(G_pitot_valido ? "1" : "0"); TELEMETRIA.print(",");
                // IMU estesa
                TELEMETRIA.print(offsetPitch, 2); TELEMETRIA.print(",");
                TELEMETRIA.print(offsetRoll, 2);  TELEMETRIA.print(",");
                TELEMETRIA.print(offsetyaw, 2);   TELEMETRIA.print(",");
                TELEMETRIA.print(G_accelX, 2);    TELEMETRIA.print(",");
                TELEMETRIA.print(G_accelY, 2);    TELEMETRIA.print(",");
                TELEMETRIA.print(G_accelZ, 2);    TELEMETRIA.print(",");
                TELEMETRIA.print(G_accelTotale, 2); TELEMETRIA.print(",");
                TELEMETRIA.print(G_gyroX, 2);     TELEMETRIA.print(",");
                TELEMETRIA.print(G_gyroY, 2);     TELEMETRIA.print(",");
                TELEMETRIA.print(G_gyroZ, 2);     TELEMETRIA.print(",");
                TELEMETRIA.print(G_imuCalSys);    TELEMETRIA.print(",");
                TELEMETRIA.print(G_imuCalGyro);   TELEMETRIA.print(",");
                TELEMETRIA.print(G_imuCalAccel);  TELEMETRIA.print(",");
                TELEMETRIA.print(G_imuCalMag);    TELEMETRIA.print(",");
                // RC estesi (canali 4-16, indici 3..15). Canali 1-3 già in TEL1.
                for (int i = 3; i < 16; i++) {
                    TELEMETRIA.print(canaliRC[i]);
                    if (i < 15) TELEMETRIA.print(",");
                }
                TELEMETRIA.println();
            }
            contatorePacchettoDiag = (contatorePacchettoDiag + 1) % 3;
        }
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

    G_targetVelocitaAttuale = targetVelocita; // salvato per diagnostica telemetria (TEL3)

    // 2. PID ALTITUDINE
    float targetPitch_Auto = 0.0;
    int gasCorrente = gasDiBase; 

    if (G_altitudine > ALTEZZA_MAX) {
        targetPitch_Auto = -8.0;
        gasCorrente = GAS_MINIMO; 
        pid_sommaErroriAlt =  0.0;  
        pid_errorePassatoAlt=  0.0;
        G_pid_altErrore = 0.0; G_pid_altP = 0.0; G_pid_altI = 0.0; G_pid_altD = 0.0;
    } else if (G_altitudine < ALTEZZA_MIN) {
        targetPitch_Auto = 12.0;
        gasCorrente = GAS_MASSIMO - 10; 
        pid_sommaErroriAlt =  0.0;
        pid_errorePassatoAlt =  0.0;
        G_pid_altErrore = 0.0; G_pid_altP = 0.0; G_pid_altI = 0.0; G_pid_altD = 0.0;
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

        G_pid_altErrore = erroreAltitudine; G_pid_altP = P_alt; G_pid_altI = I_alt; G_pid_altD = D_alt;
    }
    G_pid_targetPitchAuto = targetPitch_Auto;

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

    G_pid_pitchErrore = errorePitch; G_pid_pitchP = P_Pitch; G_pid_pitchI = I_Pitch; G_pid_pitchD = D_Pitch;

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

    G_pid_rollErrore = erroreRoll; G_pid_rollP = P_Roll; G_pid_rollI = I_Roll; G_pid_rollD = D_Roll;

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

    G_pid_velErrore = erroreVel; G_pid_velP = P_vel; G_pid_velI = I_vel; G_pid_velD = D_vel;
}


void verifica_drone_in_volo() {          
    if (!droneInVolo) {
        bool velocitaSufficiente   = (G_Airspeed_MS > SOGLIA_VELO_DECOLLO_MS && G_Groundspeed_MS > SOGLIA_VELO_DECOLLO_MS);
        bool altitudineSufficiente = (G_altitudine > SOGLIA_ALT_DECOLLO_M);

        if (velocitaSufficiente && altitudineSufficiente) {
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

    // Salvataggio per diagnostica/telemetria
    G_accelX = accel.x();
    G_accelY = accel.y();
    G_accelZ = accel.z();
    G_accelTotale = accelerazioneTotale;

    
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
    G_altitudine_lidar = -1.0f;  // Indica che il LIDAR non è valido Perché troppo alto
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

void velocità_flusso_ottico(float angoloYaw) {
    static unsigned long tempoPassatoFlussoOttico = 0;

    int dx = 0;
    int dy = 0;
    flusso_ottico.readMotionCount(&dx, &dy);
    G_flow_dx = dx; // salvataggio grezzo per diagnostica/telemetria
    G_flow_dy = dy;

    unsigned long now = millis();
    float dt = (now - tempoPassatoFlussoOttico) / 1000.0f;
    tempoPassatoFlussoOttico = now;  


    if (dt <= 0.0f || G_altitudine > ALTEZZA_MAX_SENSORE_OTTICO) {
        G_vel_x_optical_sensor = -1.0f;
        G_vel_y_optical_sensor = -1.0f;
        return;
    }

    float v_x = (dx * COSTANTE_OTTICA * G_altitudine) / dt;
    float v_y = (dy * COSTANTE_OTTICA * G_altitudine) / dt;

    float yaw_rad = radians(angoloYaw);
    G_vel_x_optical_sensor = v_x * cos(yaw_rad) - v_y * sin(yaw_rad);
    G_vel_y_optical_sensor = v_x * sin(yaw_rad) + v_y * cos(yaw_rad);
}


void aggiorna_velocita(float Velocita_pitot_Ms, float velocita_gps_Ms) {
    if (G_vel_x_optical_sensor >= 0.0f && G_vel_y_optical_sensor >= 0.0f 
        && G_altitudine < ALTEZZA_MAX_SENSORE_OTTICO) {
        
        G_Groundspeed_MS = sqrtf(G_vel_x_optical_sensor * G_vel_x_optical_sensor + 
                                 G_vel_y_optical_sensor * G_vel_y_optical_sensor);
                                 
    } else if (gps.speed.isValid()) {
        G_Groundspeed_MS = velocita_gps_Ms;
    } else {
        G_Groundspeed_MS = -1.0f; 
    }

    // CORREZIONE ERRORE EVIDENTE: era "velocita_pitot_Ms" (minuscola) — variabile
    // inesistente, il parametro si chiama "Velocita_pitot_Ms" (maiuscola).
    if (Velocita_pitot_Ms > 0 && Velocita_pitot_Ms < MAX_AIRSPEED_X8) {
        G_Airspeed_MS = Velocita_pitot_Ms;
    } else {        
        if (gps.speed.isValid() || G_altitudine < ALTEZZA_MAX_SENSORE_OTTICO) {
            G_Airspeed_MS = G_Groundspeed_MS; 
        } else {
            G_Airspeed_MS = 0.0f; // qui tocca pregare
        }
    }
}
void aggiorna_altitudine(){
    if (lidarOk && G_altitudine_lidar > 0.0f && G_altitudine_baro < ALTEZZA_MAX_LIDAR) {
        G_altitudine = G_altitudine_lidar;
    } else {
        G_altitudine = G_altitudine_baro;
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
 elaborvoidaComando(const String& cmd) {
    if (!cmd.startsWith("CMD:")) return;

    int sep = cmd.indexOf(':', 4);
    if (sep < 0) return;

    String campo     = cmd.substring(4, sep);
    String valoreStr = cmd.substring(sep + 1);   
    int    val       = valoreStr.toInt();        

    //Servi: posizione 
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

    //Servi: attach/detach
    } else if (campo == "SERVO_ISX_ATTACH") {
        servoInternoSX.attach(pinIntSX);
        inviaAck(campo, "");

    } else if (campo == "SERVO_IDX_ATTACH") {
        servoInternoDX.attach(pinIntDX);
        inviaAck(campo, "");

    } else if (campo == "SERVO_ESX_ATTACH") {
        servoEsternoSX.attach(pinEstSX);  
        inviaAck(campo, "");

    } else if (campo == "SERVO_EDX_ATTACH") {
        servoEsternoDX.attach(pinEstDX);  
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
        relèAttivato = true;
        inviaAck(campo, "");

    } else if (campo == "RELE_OFF") {
        digitalWrite(PIN_RELE, LOW);
        relèAttivato = false;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SCHIANTO_ON") {
        schianto_sicurezza = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SCHIANTO_OFF") {
        schianto_sicurezza = false;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_ALIMENTAZIONE_ON") {
        alimentazione_sicurezza = true;   
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_ALIMENTAZIONE_OFF") {
        alimentazione_sicurezza = false;   
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SERVI_ON") {
        servo_sicurezza = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_SERVI_OFF") {
        servo_sicurezza = false;
        inviaAck(campo, "");
    }else if (campo == "SICUREZZA_TEMP_ON") {
        sistema_sicurezza_temp = true;
        inviaAck(campo, "");

    } else if (campo == "SICUREZZA_TEMP_OFF") {
        sistema_sicurezza_temp = false;
        inviaAck(campo, "");

    // Gas 
    } else if (campo == "GAS") {
        if (global_modalitaVolo == 1) {
            motore.writeMicroseconds(constrain(val, GAS_NEUTRO, GAS_MASSIMO));   
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "non_in_manuale");
        }

    //Modalità di volo
    } else if (campo == "MODO") {
        if (!failsafe && val >= 1 && val <= 2) {
            global_modalitaVolo = val;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "valore_non_valido_o_failsafe");
        }

    } else if (campo == "SET_LATITUDE") {
        float lat = valoreStr.toFloat();
        if (lat >= -90.0 && lat <= 90.0) {
            TARGET_LAT = lat;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    } else if (campo == "SET_LONGITUDE") {
        float lon = valoreStr.toFloat();
        if (lon >= -180.0 && lon <= 180.0) {
            TARGET_LON = lon;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    } else if (campo == "SET_ALTITUDE") {
        float alt = valoreStr.toFloat();
        if (alt >= 0.0 && alt <= 500.0) {
            ALTITUDINE_TARGET = alt;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    // ─── NUOVI COMANDI: PARAMETRI PID (validati, entro limiti di sicurezza) ───
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

    // ─── NUOVI COMANDI: PARAMETRI OPERATIVI ───
    } else if (campo == "SET_VEL_CROCIERA") {
        float v = valoreStr.toFloat();
        if (v >= 20.0 && v <= 150.0) { VELOCITA_CROCIERA = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_VEL_AVVICINAMENTO") {
        float v = valoreStr.toFloat();
        if (v >= 15.0 && v <= 150.0) { VELOCITA_AVVICINAMENTO = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_ALT_MIN") {
        float v = valoreStr.toFloat();
        if (v >= 2.0 && v < ALTEZZA_MAX) { ALTEZZA_MIN = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite_o_maggiore_di_max");
    } else if (campo == "SET_ALT_MAX") {
        float v = valoreStr.toFloat();
        if (v > ALTEZZA_MIN && v <= 500.0) { ALTEZZA_MAX = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite_o_minore_di_min");
    } else if (campo == "SET_LIMITE_TEMP_MOTORE") {
        float v = valoreStr.toFloat();
        if (v > T_MOTORE_THROTTLE_START && v <= 120.0) { T_MOTORE_THROTTLE_END = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_RAGGIO_WAYPOINT") {
        float v = valoreStr.toFloat();
        if (v >= 5.0 && v <= 100.0) { RAGGIO_ACCETTAZIONE_MINIMO = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

    // ─── NUOVI COMANDI: CONTROLLO/DIAGNOSTICA ───
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
