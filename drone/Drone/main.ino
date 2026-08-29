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
bool forzaInvioDiagnostica = false;   

//  RICEVENTE SBUS (FrSky)
SBUS   ricevente(Serial7);
uint16_t canaliRC[16];
bool failsafe   = false;
bool pacchettoPerso = false;

//  PITOT (VELOCITÀ ARIA) --------------------------------------------------------
const int   PIN_ARIA  = A0;              
float desità_aria_aggiornata = 1.225;           // Densità dell'aria al livello del mare, in kg/m^3 
float VALORE_ZERO  = 0.0;                
const float FATTORE_CONVERSIONE_PA = 3.22;  // Fattore di conversione da "conteggi ADC" a Pascal (Pa) — dipende dal trasduttore usato

// --- Diagnostica pitot (per telemetria) ---
int   G_pitot_raw = 0;             
float G_pitot_differenza = 0.0f;    // Differenza (ADC counts) tra lettura grezza e VALORE_ZERO
bool  G_pitot_valido = false;       // true se la differenza è positiva 

//  SERVO -----------------------------------------------------------------------
const int pinIntSX  = 6;   
const int pinIntDX  = 22;  
const int pinEstSX  = 23;  
const int pinEstDX  = 24;   
const int pinMotore = 10;  

Servo servoInternoSX;  // (pitch)
Servo servoInternoDX;  // (pitch)
Servo servoEsternoSX;  // (pitch + roll)
Servo servoEsternoDX;  // (pitch + roll)
Servo motore;          

//  SENSORE CORRENTE --------------------------------------------------------------
Adafruit_INA219 sensoreMotore(0x40); 
Adafruit_INA219 sensoreIntSX(0x41);   
Adafruit_INA219 sensoreIntDX(0x42);    
Adafruit_INA219 sensoreEstSX(0x43);    
Adafruit_INA219 sensoreEstDX(0x44);    
Adafruit_INA219 sensoreTeensy(0x45);   

float G_correnteIntSX = 0.0f;   
float G_correnteIntDX = 0.0f;   
float G_correnteEstSX = 0.0f;   
float G_correnteEstDX = 0.0f;   

//  COSTANTI ------------------------------------------------------------------

const float R_SPECIFIC = 287.05f;

const float VALORE_BATT_MOTORE_BASSA_V = 13.5f;  
const float VALORE_BATT_TEENSY_BASSA_V = 4.9f;   

const int SOGLIA_G_SCHIANTO=50;          
const int SEMPLE_VALORI_SCHIANTO=3;      

const int CENTRO_SERVO  = 90;   
const int MAX_ROLL_g = 35;       
const int MAX_PITCH_g  = 20;      


float ALTEZZA_MAX_m = 120;   
float ALTEZZA_MIN_m = 10;    

const int GAS_NEUTRO = 1000;     
const int GAS_MASSIMO = 2000;   
const int GAS_MINIMO = 1200;     
const int GAS_CROCIERA= 1450;   

const float MAX_AIRSPEED_X8_km = 45.0f;         
float VELOCITA_CROCIERA_km  = 60.0;             
float VELOCITA_AVVICINAMENTO_km= 45.0;  
const float GAS_AVVICINAMENTO = 1250;       
const float DISTANZA_FRENATA_m = 150.0;        
float RAGGIO_ACCETTAZIONE_MINIMO_m = 25.0f;     

const int   IMU_CAMPIONI_TARA = 200;    

const float T_MOTORE_THROTTLE_START = 70.0f;   
float T_MOTORE_THROTTLE_END  = 90.0f;         

int G_limiteGasTermico = GAS_MASSIMO;     
int  G_comandoGasPreLimite = GAS_NEUTRO;  
bool G_limitazioneTermicaAttiva = false;  // true se in questo ciclo il limite termico ha effettivamente tagliato il gas

const float ALPHA_LIDAR = 0.25f;               // Coefficiente del filtro passa-basso (EMA) sul LIDAR, adimensionale (0-1, più alto = più reattivo/meno filtrato)
const float ALTEZZA_MAX_LIDAR_m = 6.0f;          
const float ALTEZZA_MAX_SENSORE_OTTICO_m = 4.0f; 
const float PITCH_DOWN_FORZATO = -8.0f;
const float PITCH_UP_FORZATO = 12.0f;

const float COSTANTE_OTTICA = 0.0012; 

//  NAVIGAZIONE ------------------------------------------------------------------
double TARGET_LAT_g = 41.902782;   
double TARGET_LON_g = 12.496366;   
float  ALTITUDINE_TARGET_g = 40.0;  

float G_altitudine_lidar_m = -1.0;  
float set_up_lidar_alt_m = 0.0;     
float G_altitudine_baro_m = 0.0;    
float set_up_gps_alt_m = 0.0;       
float G_altitudine_m = 0.0;          

float G_tara_altitudine_m = 0.0;       // Offset sottratto alla lettura barometrica per ottenere altitudine relativa al punto di decollo, in metri (m)
float G_targetRoll_g  = 0.0;           // Angolo di rollio target calcolato dalla guida L1_m, in gradi (°)
float G_distanzaDalTarget_m = 0.0;  
float G_rottaVersoTarget_g = 0.0;      // Rotta (bearing) verso il target, in gradi (°), 0=Nord/90=Est/180=Sud/270=Ovest
float G_errore_rotta_g  = 0.0;         // Differenza tra rotta verso il target e rotta/yaw attuale, in gradi (°), normalizzata in ±180°

float G_Airspeed_ms = 0.0;      
float G_Groundspeed_ms = 0.0;  

float G_vel_x_optical_sensor_ms = 0.0; 
float G_vel_y_optical_sensor_ms = 0.0;  
int   G_flow_dx = 0;   
int   G_flow_dy = 0;   

float offsetRoll_g  = 0.0f;   
float offsetPitch_g = 0.0f;   
float offsetyaw_g   = 0.0f;   

// --- Diagnostica IMU estesa (accelerazione lineare, giroscopio, calibrazione) ---
float G_accelX = 0.0f, G_accelY = 0.0f, G_accelZ = 0.0f, G_accelTotale = 0.0f;  // m/s^2
float G_gyroX = 0.0f, G_gyroY = 0.0f, G_gyroZ = 0.0f;                          //gradi/secondo (°/s)
uint8_t G_imuCalSys = 0, G_imuCalGyro = 0, G_imuCalAccel = 0, G_imuCalMag = 0; 

// --- Diagnostica barometro estesa ---
float G_pressione_baro_pa = 0.0f;    

//  PID — GUADAGNI ------------------------------------------------------------
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

// Limiti di validazione per i guadagni PID ricevuti da terra 
const float LIMITE_KP_MAX = 10.0f;   
const float LIMITE_KI_MAX = 2.0f;    
const float LIMITE_KD_MAX = 5.0f;    

unsigned long tempoPassatoPID = 0;  

//  VARIABILI DI STATO PID GLOBALI ---------------------------------------------
float pid_sommaErroriAlt   = 0.0;    
float pid_errorePassatoAlt_ms = 0.0;   

float pid_sommaErroriPitch = 0.0;   
float pid_errorePassatoPitch_g = 0.0;  

float pid_sommaErroriRoll  = 0.0;    
float pid_errorePassatoRoll_g = 0.0;  

float pid_sommaErroriVel   = 0.0;    
float pid_errorePassatoVel_km = 0.0;    

// --- Diagnostica PID (salvata ad ogni ciclo calcolaPID, per telemetria TEL3) ---
float G_pid_altErrore=0, G_pid_altP=0, G_pid_altI=0, G_pid_altD=0;        
float G_pid_targetPitchAuto=0;                                          
float G_pid_pitchErrore=0, G_pid_pitchP=0, G_pid_pitchI=0, G_pid_pitchD=0; 
float G_pid_rollErrore=0, G_pid_rollP=0, G_pid_rollI=0, G_pid_rollD=0;    
float G_pid_velErrore=0, G_pid_velP=0, G_pid_velI=0, G_pid_velD=0;       
float G_targetVelocitaAttuale_km = 0.0f;                                   


const unsigned long TEMPO_DECOLLO_SICURO_MS = 1500;  
const float SOGLIA_VELO_DECOLLO_MS = 5.0;           
const float SOGLIA_ALT_DECOLLO_M = 5.0;             
unsigned long timestampDecollo = 0;                  
static int contatoreImpatto = 0;                     

//  LED DI STATO E ALLARMI ------------------------------------------------------
const int PIN_LED_ROSSO_ALARM = 2; // Pin digitale LED rosso — Allarmi come moduli mancanti / Batteria
const int PIN_LED_VERDE_GPS = 3;   // Pin digitale LED verde — GPS Fix e settaggio pitot
const int PIN_LED_BLU_PID  = 4;    // Pin digitale LED blu — Modalità AUTO pid e settaggio barometro
const int PIN_BUZZER = 33;       
const int PIN_RELE = 20;           
//  FLAGS STATO SISTEMA ----------------------------------------------------------
bool imuPronto       = false;   
bool flusso_otticoOK = false;   
bool lidarOk = false;          
bool baroPronto      = false;  
bool pitotCalibrato  = false;   
bool Voltaggio       = true;    
int  tentativi       = 0;       
const int MAX_TENTATIVI = 3;   

bool servo_sicurezza         = true;   // Abilita/disabilita la diagnostica di sicurezza sui servi (via comando da terra)
bool alimentazione_sicurezza = true;   // Abilita/disabilita il controllo di sicurezza sulle batterie
bool schianto_sicurezza      = true;   // Abilita/disabilita il rilevamento schianto

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
int  global_modalitaVolo       = 1;      // Modalità di volo corrente: 1=Manuale, 2=Auto, (3=Failsafe gestito a parte, non scritto qui)
bool statoPrecedenteInterni    = true;   
bool statoPrecedenteEsterni    = true;   

bool sistema_sicurezza_temp = true;   // Abilita/disabilita la limitazione termica del gas

//  PROTOTIPI -------------------------------------------------------------------

void applicaMixer4Servi(int pitch, int roll);
void aggiornaNavigazione(float angoloYaw_g);
void diagnosticaServi();
void inviaTelemetria(float pitch, float roll, float yaw, float velPitotKmh, float velGpsKmh, int outPitch, int outRoll, int outGas);
void calcolaPID(float targetAltitudine, float targetRoll, float pitchReale, float rollReale, float velocitaAttuale, float targetVelocita, int gasDiBase, int &comandoPitchOut_g, int &comandoRollOut_g, int &comandoGasOut);
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
void velocità_flusso_ottico(float angoloYaw_g);
void resettaPID();
void aggiornaDiagnosticaIMU();
void inviaAck(const String& campo, const String& valore);
void inviaNack(const String& campo, const String& motivo);
void aggiorna_densita_aria(float pressione_pa, float temperatura_c);


void segnalaOK() {
    digitalWrite(PIN_LED_VERDE_GPS, HIGH);
    tone(PIN_BUZZER, 1200, 150);   // Frequenza 1200 Hz, durata 150 ms
    digitalWrite(PIN_LED_VERDE_GPS, LOW);
}

void segnalaErrore() {
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        tone(PIN_BUZZER, 400, 100);   // Frequenza 400 Hz, durata 100 ms
        delay(150);
        digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        delay(100);
    }
}

void segnalaCalibrazione(int pin_led) {
    digitalWrite(pin_led, !digitalRead(pin_led));  // toggle
    tone(PIN_BUZZER, 1000, 30);                    // Beep breve 1000 Hz, 30 ms
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

    // Bip di accensione (sequenza di 3 toni crescenti)
    tone(PIN_BUZZER, 800,  100);   delay(150);
    tone(PIN_BUZZER, 1200, 100);   delay(150);
    tone(PIN_BUZZER, 1600, 150);   delay(300);

    Serial.println("     SISTEMA DRONE — AVVIO IN CORSO     ");;

    ricevente.begin();                       
    TELEMETRIA.begin(BAUD_RATE_LORA);        
    GPS_SERIAL.begin(BAUD_RATE_GPS);        
    Serial2.begin(BAUD_RATE_LIDAR);          
    delay(100);

    while (Serial2.available()) Serial2.read();   // Svuota eventuali byte residui nel buffer seriale del LIDAR
    
    // INIZIALIZZAZIONE SENSORI — ciclo ripetuto fino a MAX_TENTATIVI se qualcosa non è pronto
    while ((!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) && tentativi < MAX_TENTATIVI) {
        tentativi++;
        Serial.println("\n-----------------------------------------");
        Serial.print  ("  Tentativo ");
        Serial.print  (tentativi);
        Serial.print  (" / ");
        Serial.println(MAX_TENTATIVI);
        Serial.println("-----------------------------------------");


        // --- Flusso ottico: tentativo di inizializzazione via SPI ---
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

        // --- LIDAR TF-Luna: attende per 3000 ms (3 s) un pacchetto valido con header 0x59 0x59 ---
        if (!lidarOk) {
            Serial.println("[ ] TF-Luna LIDAR .............. ");
            unsigned long t0 = millis();
            while (millis() - t0 < 3000) {   
                if (Serial2.available() >= 9) {    // Un pacchetto TF-Luna è lungo 9 byte
                    if (Serial2.read() == 0x59 && Serial2.peek() == 0x59) {
                        Serial2.read();
                        for (int i = 0; i < 7; i++) {
                            Serial2.read();     // Scarta il resto del pacchetto (non lo interpreta qui)
                        }
                        lidarOk = true;
                        break;
                    }
                }
            }
            if (lidarOk) {
                for (int i = 0; i < 10; i++) {
                    aggiornaLidar();    // Effettua 10 letture per "riscaldare" il filtro EMA sull'altezza LIDAR
                    delay(20);          // Pausa 20 ms tra una lettura e l'altra
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
            giroscopio.setExtCrystalUse(true);   // Usa il cristallo esterno per un clock più stabile
            Serial.println("OK");
            // 1. calibrazione interna giroscopio (attende che il livello di calibrazione del giroscopio raggiunga almeno 2 su 3)
            Serial.print("   Calibrazione interna (non muovere)");
            uint8_t sys, gyro, accel, mag;
            unsigned long timeout = millis();
            do {
                giroscopio.getCalibration(&sys, &gyro, &accel, &mag);
                Serial.print(".");
                delay(100);                         // Controllo ogni 100 ms
                if (millis() - timeout > 10000) {    // Timeout massimo di attesa: 10000 ms (10 s)
                    Serial.println(" timeout, continuo");
                    break;
                }
            } while (gyro < 2);

            // 2. tara: media di IMU_CAMPIONI_TARA (200) campioni per calcolare l'offset statico di roll/pitch/yaw
            Serial.println("\n   Tara offset in corso...");
            double sommaRoll  = 0.0;
            double sommaPitch = 0.0;
            double sommaYawSin = 0.0;
            double sommaYawCos = 0.0;
            for (int i = 0; i < IMU_CAMPIONI_TARA; i++) {
                sensors_event_t ev;
                giroscopio.getEvent(&ev);
                sommaRoll  += ev.orientation.z;   // roll
                sommaPitch += ev.orientation.y;   // pitch
                float yawRad = radians(ev.orientation.x);
                sommaYawSin += sin(yawRad);
                sommaYawCos += cos(yawRad);
                delay(10);                     
            }
            offsetRoll_g  = (float)(sommaRoll  / IMU_CAMPIONI_TARA);
            offsetPitch_g = (float)(sommaPitch / IMU_CAMPIONI_TARA);   
            offsetyaw_g = degrees(atan2(sommaYawSin, sommaYawCos));
            if (offsetyaw_g < 0.0f) offsetyaw_g += 360.0f;

            imuPronto = true; 
            segnalaOK();
            Serial.print(" imu offsets: roll= ");
            Serial.print(offsetRoll_g, 2);
            Serial.print(" ; pitch= ");
            Serial.print(offsetPitch_g, 2);
            Serial.print(" ; yaw= ");
            Serial.println(offsetyaw_g, 2);
        } else {
            Serial.println("\n ERRORE (cavi I2C?)");
            segnalaErrore();
        }
        } else {
            Serial.println(" \n [OK] IMU BNO055");
        }

        //2. BAROMETRO — inizializzazione, oversampling, filtro, tara altitudine ASL
        if (!baroPronto) {
            Serial.print("[ ] Barometro BMP390 ....... ");
            if (barometro.begin_I2C()) {
                barometro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);   
                barometro.setPressureOversampling(BMP3_OVERSAMPLING_32X);    
                barometro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);       
                barometro.setOutputDataRate(BMP3_ODR_50_HZ);                 // Frequenza di aggiornamento dati 50 Hz
                delay(100);

                // --- LETTURE A VUOTO per scartare (le prime letture dopo un cambio di configurazione possono essere instabili) ---
                for(int j=0; j<3; j++) {
                    barometro.readAltitude(1013.25);   // Pressione di riferimento SLP standard: 1013.25 hPa
                    delay(25);
                }
                
                float sommaAlt = 0.0;
                bool erroreCalibrazione = false;

                for (int i = 0; i < 20; i++) {   // 20 campioni per la media di tara
                    segnalaCalibrazione(PIN_LED_BLU_PID);
                    
                    float altIstantanea_m = barometro.readAltitude(1013.25);   // Altitudine calcolata rispetto a 1013.25 hPa, in metri (m)
                    
                    // VALIDAZIONE HARDWARE (Limiti ASL estremi): scarta letture fisicamente impossibili
                    if (altIstantanea_m < -500.0 || altIstantanea_m > 8000.0) {   // Range plausibile: da -500 m a 8000 m ASL
                        Serial.println("\n ERRORE: Lettura barometrica impossibile");
                        Serial.print(" Altitudine letta: ");
                        Serial.print(altIstantanea_m);
                        Serial.println(" m");
                        
                        digitalWrite(PIN_LED_BLU_PID, LOW);
                        erroreCalibrazione = true;
                        break; 
                    }
                    sommaAlt += altIstantanea_m;
                    delay(25);   // 20 x 25 ms = 500 ms totali di campionamento
                }
                
                if (erroreCalibrazione) {
                    segnalaErrore();
                    continue;    
                }

                digitalWrite(PIN_LED_BLU_PID, LOW);
                digitalWrite(PIN_BUZZER, LOW);

                baroPronto = true;
                float mediaBaroASL = sommaAlt / 20.0;   
                set_up_gps_alt_m = mediaBaroASL;         // (nome variabile fuorviante: qui non c'entra il GPS, è la media barometrica)
                
                // Se il LIDAR è disponibile ed entrambe le altezze indicano "vicino a terra" (<5 m), usa il LIDAR per tarare l'offset barometrico
                if (set_up_gps_alt_m < 5.0 && set_up_lidar_alt_m < 5.0 && set_up_lidar_alt_m > 0.0 && lidarOk) {
                    
                    G_tara_altitudine_m = mediaBaroASL - set_up_lidar_alt_m;   
                    Serial.print("OK (Tara ASL corretta da LIDAR: ");
                } else {
                    G_tara_altitudine_m = mediaBaroASL;    
                    Serial.print("OK (Tara ASL standard: ");
                }
                
                Serial.print(G_tara_altitudine_m, 1);
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
            long sommaLetture = 0;
            for (int i = 0; i < 100; i++) {    // 100 campioni
                if (i % 10 == 0) {
                    segnalaCalibrazione(PIN_LED_VERDE_GPS);
                }
                sommaLetture += analogRead(PIN_ARIA);   
                delay(10);                              
            }
            digitalWrite(PIN_LED_VERDE_GPS, LOW);
            digitalWrite(PIN_BUZZER,LOW);

            VALORE_ZERO = sommaLetture / 100.0;    
            if (VALORE_ZERO > 5 && VALORE_ZERO < 1020) {   // Verifica plausibilità 
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

        // 4. INA219 — verifica che tutti i sensori di corrente/tensione rispondano sul bus I2C
        Voltaggio = true;
        Serial.print("[ ] INA219 Batteria motore........ ");
        if (sensoreMotore.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); Voltaggio = false; }

        Serial.print("[ ] INA219 Batteria Teensy........ ");
        if (sensoreTeensy.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); Voltaggio = false; }

        Serial.print("[ ] INA219 Servo IntSX ..... ");
        if (sensoreIntSX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); Voltaggio = false; }

        Serial.print("[ ] INA219 Servo IntDX ..... ");
        if (sensoreIntDX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); Voltaggio = false; }

        Serial.print("[ ] INA219 Servo EstSX ..... ");
        if (sensoreEstSX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); Voltaggio = false; }

        Serial.print("[ ] INA219 Servo EstDX ..... ");
        if (sensoreEstDX.begin()) { Serial.println("OK"); } else { Serial.println("ERRORE"); Voltaggio = false; }

        if (Voltaggio) { segnalaOK(); } else { segnalaErrore(); }

        //  Riepilogo tentativo: se manca ancora qualcosa, aspetta 2s e riprova
        if (!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) {
            Serial.println("\n  >> Sensori mancanti. Nuovo tentativo tra 2s...");
            digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
            delay(2000);
            digitalWrite(PIN_LED_ROSSO_ALARM, LOW);
        }
    }
    //  ERRORE CRITICO — se dopo MAX_TENTATIVI manca un sensore necessario, blocca l'avvio
    if (!imuPronto || !baroPronto || !pitotCalibrato || !Voltaggio) {
        Serial.println("\n!!! ERRORE CRITICO — AVVIO BLOCCATO !!!");
        Serial.println("    Controlla l'hardware e riavvia.");
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
        while (1) {                          // Blocco infinito
            tone(PIN_BUZZER, 2000, 300);
            delay(400);
        }
    }
    
    //  TUTTO OK — INIT SERVO
    Serial.println("     TUTTI I SENSORI OPERATIVI          ");
    Serial.println("Inizializzazione servomotori...");

    inizializzazione_servo();   // Attacca tutti i servi e li porta al centro (90°)
    inizializzazione_motore();  // Attacca l'ESC e invia comando neutro (GAS_NEUTRO)
    Serial.println("Settati flap neutri e gas al minimo");

    // Jingle avvio riuscito
    tone(PIN_BUZZER, 800,  120); delay(170);
    tone(PIN_BUZZER, 1200, 120); delay(170);
    tone(PIN_BUZZER, 1800, 200); delay(350);

    Serial.println("Verifica oculare luci di stato...");
    digitalWrite(PIN_LED_ROSSO_ALARM, HIGH);
    digitalWrite(PIN_LED_VERDE_GPS,HIGH);
    digitalWrite(PIN_LED_BLU_PID, HIGH);
    delay(1000);   // Tiene tutti i LED accesi 1 s
    digitalWrite(PIN_LED_ROSSO_ALARM,LOW);
    digitalWrite(PIN_LED_VERDE_GPS,LOW);
    digitalWrite(PIN_LED_BLU_PID,LOW);

    tempoPassatoPID = millis();   //riferimento temporale per il primo calcolo PID
    Serial.println("\n  >> SISTEMA PRONTO AL VOLO\n");
    delay(500);
}

// Collega i 4 servocomandi ai rispettivi pin e li porta tutti alla posizione centrale (90°)
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

// Collega l'ESC del motore e invia il comando neutro (GAS_NEUTRO)
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
    float angoloPitch_g = event.orientation.y - offsetPitch_g;   
    float angoloRoll_g  = event.orientation.z - offsetRoll_g;   
    float angoloYaw_g   = event.orientation.x ;                 // Yaw reale = lettura IMU SENZA sottrarre offsetyaw_g (offsetyaw_g calcolato ma non usato qui), in gradi (°)
    //Normalizzazione angolo per mantenerlo  nel range [0, 360)
    if (angoloYaw_g < 0.0f) {
        angoloYaw_g += 360.0f;
    } else if (angoloYaw_g >= 360.0f) {
        angoloYaw_g -= 360.0f;
    }
    // 2. temperatura motore e barometro
    temp_aria_barometro = barometro.temperature;    
    float voltaggio_Sensore_motore_V = analogRead(PIN_T_motore) * (3.3 / 1023.0);   // assumendo Vref 3.3V
    Global_Temperatura_motore = (voltaggio_Sensore_motore_V - 0.5) * 100.0;        

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
        Velocita_pitot_Ms = sqrtf((2.0f * pressionePascal) / desità_aria_aggiornata);              // v = sqrt(2*p/rho) 
    }
    // 4. GPS – lettura velocità GPS (Ground Speed, cioè rispetto al suolo)
    float velocita_gps_Ms = 0.0f;
    if (gps.speed.isValid()) {
        velocita_gps_Ms = gps.speed.mps();  
    } else {
        velocita_gps_Ms = -1.0f;   // Valore negativo indica che la velocità GPS non è valida
    }

    // 5. Barometro – lettura altitudine (relativa, dopo sottrazione della tara)
    G_altitudine_baro_m = barometro.readAltitude(1013.25f) - G_tara_altitudine_m;   
    G_pressione_baro_pa = barometro.pressure; 

    aggiorna_densita_aria(G_pressione_baro_pa, temp_aria_barometro);   // Aggiorna la densità con la pressione statica e la temperatura
    aggiornaLidar();                     
    aggiorna_altitudine();                 // metri (m)
    velocità_flusso_ottico(angoloYaw_g);    // m/s
    aggiorna_velocita(Velocita_pitot_Ms, velocita_gps_Ms);   //  (m/s)
    aggiornaNavigazione(angoloYaw_g);     

    // 9. PREPARAZIONE DATI MOTORE — sceglie velocità target e gas di base in base alla distanza dal target
    float targetVelocita = 0.0;
    int gasDiBase = 0;
    if (G_distanzaDalTarget_m > DISTANZA_FRENATA_m) {     
        targetVelocita = VELOCITA_CROCIERA_km;            
        gasDiBase= GAS_CROCIERA;                         
    } else {                                             
        targetVelocita = VELOCITA_AVVICINAMENTO_km;       
        gasDiBase= GAS_AVVICINAMENTO;                                 
    }

    int correzionePitch_g  = 0;    
    int correzioneRoll_g   = 0;    
    int comandoGasFinale = GAS_NEUTRO;  

    // 1 = Manuale, 2 = Auto, 3 = Failsafe
    if (ricevente.read(&canaliRC[0], &failsafe, &pacchettoPerso)) {   // Se è arrivato un nuovo pacchetto SBUS valido
        // Sblocco emergenza: se in stato di schianto rilevato, il canale 5 (indice 4) sotto 992 lo resetta manualmente
        if (statoSchiantoRilevato && canaliRC[4] < 992) {
            statoSchiantoRilevato = false;
            schiantoBloccato = false;
            droneInVolo = false;           
            inizializzazione_servo();          // Ricentra e riattacca i servi

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
            if(droneInVolo  && statoSchiantoRilevato == false){
                global_modalitaVolo = 2; 
            }
        }
    }
    
    int statoAttuale;
    static int ultimoStatoStampato = 0;
    if(failsafe){
        statoAttuale = 3;                 // Stato "3" = failsafe, per la sola logica di stampa/reset PID qui sotto
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
        // Gas: da 172-1811 (range SBUS) a GAS_NEUTRO-GAS_MASSIMO (µs), vincolato ai limiti meccanici motore
        comandoGasFinale = constrain(map(canaliRC[2], 172, 1811, GAS_NEUTRO, GAS_MASSIMO), GAS_NEUTRO, GAS_MASSIMO);
        correzioneRoll_g   = constrain(map(canaliRC[0], 172, 1811, -MAX_ROLL_g,   MAX_ROLL_g),   -MAX_ROLL_g,  MAX_ROLL_g);      
        correzionePitch_g  = constrain(map(canaliRC[1], 172, 1811,  MAX_PITCH_g, -MAX_PITCH_g),  -MAX_PITCH_g, MAX_PITCH_g);      
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
        // Chiamata al PID: quota target (m), roll target (°), pitch/roll reali (°), velocità attuale/target (km/h), gas di base (µs)
        calcolaPID(ALTITUDINE_TARGET_g, G_targetRoll_g,angoloPitch_g, angoloRoll_g,G_Airspeed_ms *3.6, targetVelocita,gasDiBase,correzionePitch_g, correzioneRoll_g, comandoGasFinale);
    }
    int gasEffettivo = GAS_NEUTRO;   // Valore di gas realmente inviato al motore in questo ciclo, in µs (per la telemetria)

    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO);   // Dopo uno schianto: motore forzato a comando neutro
        gasEffettivo = GAS_NEUTRO;
    } else {
        gestisci_allarmi();                              // Aggiorna i LED di stato in base ad allarmi/GPS/modalità
        applicaMixer4Servi(correzionePitch_g, correzioneRoll_g);   

        int limiteTermico = gasMaxTermico();           
        G_limiteGasTermico = limiteTermico; 
        G_comandoGasPreLimite = comandoGasFinale;      
        if (comandoGasFinale > limiteTermico && sistema_sicurezza_temp) {
            comandoGasFinale = limiteTermico;             // Applica il taglio termico se abilitato
            G_limitazioneTermicaAttiva = true;
        } else {
            G_limitazioneTermicaAttiva = false;
        }
        motore.writeMicroseconds(comandoGasFinale);       
        gasEffettivo = comandoGasFinale;
    }

    inviaTelemetria(
    angoloPitch_g, angoloRoll_g, angoloYaw_g,
    Velocita_pitot_Ms * 3.6f,     
    velocita_gps_Ms   * 3.6f,    
    correzionePitch_g, correzioneRoll_g, gasEffettivo);
}

void aggiorna_densita_aria(float pressione_pa, float temperatura_c) {
    float temperaturaK = temperatura_c + 273.15f;
    if (temperaturaK > 0.0f && pressione_pa > 0.0f) {
        desità_aria_aggiornata = pressione_pa / (R_SPECIFIC * temperaturaK);
    }
}

//  MIXER 4 SERVI ----------------------------------------------------------------
void applicaMixer4Servi(int pitch, int roll){
    int posIntSX = CENTRO_SERVO;
    int posIntDX = CENTRO_SERVO;
    int posEstSX = CENTRO_SERVO;
    int posEstDX = CENTRO_SERVO;

    // Batteria Teensy bassa forza solo esterni per risparmiare corrente (disattiva i servi interni)
    if (batteriaBassa_teensy) {
        intSX_Ok = false;
        intDX_Ok = false;
    }

    bool esterniAttivi = estSX_Ok && estDX_Ok;
    bool interniAttivi = intSX_Ok && intDX_Ok;

    if (esterniAttivi && interniAttivi) {
        // Caso A: tutto OK interni = SOLO PITCH, esterni = SOLO ROLL
        posIntSX = CENTRO_SERVO + pitch;
        posIntDX = CENTRO_SERVO + pitch;
        posEstSX = CENTRO_SERVO + roll;
        posEstDX = CENTRO_SERVO - roll;   
    } else if (esterniAttivi && !interniAttivi) {
        // Caso B: interni rotti esterni fanno pitch + roll 
        posEstSX = CENTRO_SERVO + pitch + roll;
        posEstDX = CENTRO_SERVO + pitch - roll;
    } else if (!esterniAttivi && interniAttivi) {
        // Caso C: esterni rotti interni fanno pitch + roll 
        posIntSX = CENTRO_SERVO + pitch + roll;
        posIntDX = CENTRO_SERVO + pitch - roll;
    } else {
        return;   
    }

    // ── Attach/detach automatico in base a se i servi sono considerati attivi o no ──
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

    // Limiti di sicurezza meccanici: 45°-135° (±45° dal centro 90°)
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

//  NAVIGAZIONE GPS ----------------------------------------------------------------
void aggiornaNavigazione(float angoloYaw_g)
{
    if (gps.location.isValid()) {
        // geometria target: distanza (in metri) e rotta (in gradi) verso il waypoint
        G_distanzaDalTarget_m = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), TARGET_LAT_g, TARGET_LON_g); 
        G_rottaVersoTarget_g = TinyGPSPlus::courseTo(gps.location.lat(), gps.location.lng(), TARGET_LAT_g, TARGET_LON_g);

        float velPerCalcolo_ms = max(G_Groundspeed_ms, 1.0f);   
        float L1_m = max(velPerCalcolo_ms * 4.0f, 1.0f);         
        float raggio_accettazione_dinamico_m = max(RAGGIO_ACCETTAZIONE_MINIMO_m, L1_m * 0.75f);   

        if (G_distanzaDalTarget_m <= raggio_accettazione_dinamico_m) {
            Serial.println("WAYPOINT RAGGIUNTO! Inizializza passaggio al prossimo target...");
            G_distanzaDalTarget_m = 0.0f;
            return;  
        }
        // calcolo deriva vento / rotta reale
        float rotta_attuale;
        // Se la velocità è < 2 m/s il GPS è impreciso sulla direzione (course), quindi usiamo la bussola (Yaw) dell'IMU
        if (gps.course.isValid() && G_Groundspeed_ms > 2.0f) {
            rotta_attuale = gps.course.deg();  
        } else {
            rotta_attuale = angoloYaw_g;          
        }
    
        G_errore_rotta_g = G_rottaVersoTarget_g - rotta_attuale;

        // ERRORE ROTTA: Via più breve per girare (normalizzazione a ±180°)
        if(G_errore_rotta_g >  180.0) {
            G_errore_rotta_g -= 360.0;
        }else if (G_errore_rotta_g < -180.0) {
            G_errore_rotta_g += 360.0;
        }
        // Guida L1_m: calcola l'accelerazione laterale necessaria per curvare verso la rotta: a_lat = 2*V^2/L1_m * sin(eta)
        float eta = radians(G_errore_rotta_g);                                         
        float a_lat = (2.0f * velPerCalcolo_ms * velPerCalcolo_ms / L1_m) * sin(eta);          // m/s^2
        float rollRad = atan(a_lat / 9.81f);                                          
        G_targetRoll_g = constrain(degrees(rollRad), -MAX_ROLL_g, MAX_ROLL_g);              
        Serial.print("Dist WP: ");
        Serial.print(G_distanzaDalTarget_m);
        Serial.print("m | Rotta Target: ");
        Serial.print(G_rottaVersoTarget_g);
        Serial.print("° | Err: ");
        Serial.print(G_errore_rotta_g);
        Serial.print("° | Target Roll: ");
        Serial.println(G_targetRoll_g);

    } else {
        Serial.println("GPS: In attesa di segnale valido (FIX 3D)...");
    }
}

//  DIAGNOSTICA SERVI ----------------------------------------------------------------
// Legge la corrente di ciascun servo (mA) e dichiara un servo "guasto" dopo 5 letture anomale consecutive
void diagnosticaServi(){
    if (!servo_sicurezza) {
        estSX_Ok = estDX_Ok = intSX_Ok = intDX_Ok = true;  
        return;
    }
    static int consecutiveErrors[4] = {0, 0, 0, 0};   // Conteggio errori consecutivi per [EstSX, EstDX, IntSX, IntDX]
    float mA = 0.0f;

    // Servo Esterno SX: anomalia se corrente < 0.5 mA (quasi nulla, scollegato) o > 2500 mA (stallo/cortocircuito)
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

//  GESTIONE LUCI DI STATO --------------------------------------------------------
void gestisci_allarmi() {
    // 1. EMERGENZA CRITICA: Guasto Servi (Tutti i LED accesi fissi come segnale univoco di emergenza)
    if (!estSX_Ok || !estDX_Ok || !intSX_Ok || !intDX_Ok) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH); 
        digitalWrite(PIN_LED_VERDE_GPS, HIGH);
        digitalWrite(PIN_LED_BLU_PID,   HIGH);
        return; 
    }
    // 2. LED ROSSO: Allarmi (Batteria motore bassa o Failsafe radio)
    if (batteriaBassa_motore || failsafe) {
        digitalWrite(PIN_LED_ROSSO_ALARM, HIGH); 
    } else {
        digitalWrite(PIN_LED_ROSSO_ALARM, LOW);  
    }
    // 3. LED VERDE: Stato fix GPS
    if (gps.location.isValid()) {
        digitalWrite(PIN_LED_VERDE_GPS, HIGH); 
    } else {
        digitalWrite(PIN_LED_VERDE_GPS, LOW);  
    }
    // 4. LED BLU: Modalità di volo (acceso = AUTO)
    if (global_modalitaVolo == 2) {
        digitalWrite(PIN_LED_BLU_PID, HIGH);   
    } else {
        digitalWrite(PIN_LED_BLU_PID, LOW);    
    }
}

// calibrazione e giroscopio dal BNO055
void aggiornaDiagnosticaIMU() {
    giroscopio.getCalibration(&G_imuCalSys, &G_imuCalGyro, &G_imuCalAccel, &G_imuCalMag);   
    imu::Vector<3> gyro = giroscopio.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);          // gradi/secondo (°/s)
    G_gyroX = gyro.x();
    G_gyroY = gyro.y();
    G_gyroZ = gyro.z();
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

// Azzera tutti gli stati integrali/derivativi dei 4 PID e resetta il riferimento temporale dt
void resettaPID() {
    pid_sommaErroriAlt  = 0.0;  pid_errorePassatoAlt_ms = 0.0;
    pid_sommaErroriPitch= 0.0;  pid_errorePassatoPitch_g= 0.0;
    pid_sommaErroriRoll = 0.0;  pid_errorePassatoRoll_g = 0.0;
    pid_sommaErroriVel = 0.0;  pid_errorePassatoVel_km = 0.0;
    tempoPassatoPID = millis();
}

//  INVIO TELEMETRIA COMPLETA (Formato CSV) --------------------------------------
//
// TEL1 ("$,")  -> stato + assetto + navigazione + batterie + servi (FORMATO ORIGINALE,
//                 INVARIATO, stessa frequenza di prima ~2Hz, per non rompere il parser
//                 GCS esistente). In coda sono stati aggiunti SOLO 2 campi nuovi
//                 (timestamp millis, contatore pacchetto).
// TEL2 ("$2,") -> correnti/potenze batterie + dettaglio flusso ottico grezzo + gas pre-limite
// TEL3 ("$3,") -> diagnostica PID completa
// TEL4 ("$4,") -> GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC estesi (4-16)
//
// TEL2/3/4 vengono inviati in round-robin ogni 2 secondi circa (uno alla volta,
// alternati), per non appesantire la banda LoRa: sono dati diagnostici che non
// servono ad alta frequenza. REQ_DIAG forza l'invio immediato del prossimo del ciclo.
void inviaTelemetria(float pitch, float roll, float yaw, float velPitotKmh, float velGpsKmh, int outPitch, int outRoll, int outGas) {
    
    unsigned long tempoAttuale = millis();

    // Invio TEL1 a 2 Hz (ogni 500 ms) per non saturare la banda radio LoRa
    if (tempoAttuale - timerTelemetria > 500) {
        timerTelemetria = tempoAttuale;

        // --- LETTURE VOLTAGGI IN TEMPO REALE, tutte in Volt (V) ---
        float vBatt  = sensoreMotore.getBusVoltage_V();
        float vTeensy = sensoreTeensy.getBusVoltage_V();
        float vIntSX = sensoreIntSX.getBusVoltage_V();
        float vIntDX = sensoreIntDX.getBusVoltage_V();
        float vEstSX = sensoreEstSX.getBusVoltage_V();
        float vEstDX = sensoreEstDX.getBusVoltage_V();


        int codiceAllarme = 0;   // Bitmask allarmi globali (adimensionale)
        if (failsafe)             codiceAllarme += 1;  // Bit 0
        if (batteriaBassa_motore) codiceAllarme += 2;  // Bit 1
        if (relèAttivato)         codiceAllarme += 4;  // Bit 2
        if (batteriaBassa_teensy) codiceAllarme += 8;  // Bit 3
        if (statoSchiantoRilevato)codiceAllarme += 16; // Bit 4
        if (droneInVolo)          codiceAllarme += 32; // Bit 5 


        TELEMETRIA.print("$,"); // 0. Start indicatore pacchetto TEL1
        
        // --- STATO E ALLARMI ---
        TELEMETRIA.print(global_modalitaVolo);     TELEMETRIA.print(","); // 1. Modalità (1=Manuale, 2=Auto, 3=Failsafe)
        TELEMETRIA.print(codiceAllarme);           TELEMETRIA.print(","); // 2. Bitmask allarmi globali
        
        // --- ALIMENTAZIONE (tutte le tensioni in Volt, V) ---
        TELEMETRIA.print(vBatt, 2);                TELEMETRIA.print(","); // 3. V Motore
        TELEMETRIA.print(vTeensy, 2);              TELEMETRIA.print(","); // 4. V Teensy
        TELEMETRIA.print(vIntSX, 2);               TELEMETRIA.print(","); // 5. V Servo Int SX
        TELEMETRIA.print(vIntDX, 2);               TELEMETRIA.print(","); // 6. V Servo Int DX
        TELEMETRIA.print(vEstSX, 2);               TELEMETRIA.print(","); // 7. V Servo Est SX
        TELEMETRIA.print(vEstDX, 2);               TELEMETRIA.print(","); // 8. V Servo Est DX
        
        // --- STATO SALUTE SERVI (4 cifre "1"/"0" concatenate, non separate da virgola) ---
        TELEMETRIA.print(intSX_Ok ? "1" : "0");
        TELEMETRIA.print(intDX_Ok ? "1" : "0");
        TELEMETRIA.print(estSX_Ok ? "1" : "0");
        TELEMETRIA.print(estDX_Ok ? "1" : "0");    TELEMETRIA.print(","); // 9. Salute Servi

        // --- ASSETTO E QUOTA (IMU + Baro) ---
        TELEMETRIA.print(pitch, 1);                TELEMETRIA.print(","); // 10. Pitch reale, in gradi (°)
        TELEMETRIA.print(roll, 1);                 TELEMETRIA.print(","); // 11. Roll reale, in gradi (°)
        TELEMETRIA.print(yaw, 1);                  TELEMETRIA.print(","); // 12. Yaw reale (Bussola), in gradi (°)
        TELEMETRIA.print(G_altitudine_m, 1);         TELEMETRIA.print(","); // 13. Altitudine relativa (fusa), in metri (m)
        
        // --- VELOCITÀ (tutte in km/h, già convertite dal chiamante) ---
        TELEMETRIA.print(velPitotKmh, 1);             TELEMETRIA.print(","); // 14. Velocità Aria (Pitot grezza), km/h
        TELEMETRIA.print(velGpsKmh, 1);               TELEMETRIA.print(","); // 15. Velocità Suolo (GPS grezza), km/h
        TELEMETRIA.print(G_Airspeed_ms * 3.6f, 1);    TELEMETRIA.print(","); // 16. Airspeed fusa (usata dal PID), km/h
        TELEMETRIA.print(G_Groundspeed_ms * 3.6f, 1); TELEMETRIA.print(","); // 17. Groundspeed fusa (flusso ottico/GPS), km/h
        
        // --- NAVIGAZIONE ---
        TELEMETRIA.print(G_distanzaDalTarget_m, 0); TELEMETRIA.print(","); // 18. Distanza target, in metri (m)
        TELEMETRIA.print(G_rottaVersoTarget_g, 1);  TELEMETRIA.print(","); // 19. Rotta target, in gradi (°)
        TELEMETRIA.print(G_targetRoll_g, 1);        TELEMETRIA.print(","); // 20. Rollio comandato da L1_m, in gradi (°)
        
        // --- INPUT RADIOCOMANDO (valori grezzi SBUS, range tipico 172-1811, adimensionale) ---
        TELEMETRIA.print(canaliRC[1]);             TELEMETRIA.print(","); // 21. RC Pitch
        TELEMETRIA.print(canaliRC[0]);             TELEMETRIA.print(","); // 22. RC Roll
        TELEMETRIA.print(canaliRC[2]);             TELEMETRIA.print(","); // 23. RC Gas
        
        // --- OUTPUT PID/MIXER ---
        TELEMETRIA.print(outPitch);                TELEMETRIA.print(","); // 24. PID Pitch Out, in gradi (°)
        TELEMETRIA.print(outRoll);                 TELEMETRIA.print(","); // 25. PID Roll Out, in gradi (°)
        TELEMETRIA.print(outGas);                  TELEMETRIA.print(","); // 26. PID Gas Out (effettivo, post-limite termico), in µs
        
        // --- POSIZIONE FISICA ATTUALE SERVI (in gradi, range 45-135) ---
        TELEMETRIA.print(servoInternoSX.read());   TELEMETRIA.print(","); // 27. Pos Servo Int SX, °
        TELEMETRIA.print(servoInternoDX.read());   TELEMETRIA.print(","); // 28. Pos Servo Int DX, °
        TELEMETRIA.print(servoEsternoSX.read());   TELEMETRIA.print(","); // 29. Pos Servo Est SX, °
        TELEMETRIA.print(servoEsternoDX.read());   TELEMETRIA.print(","); // 30. Pos Servo Est DX, °

        // --- TEMPERATURE (in gradi Celsius, °C) ---
        TELEMETRIA.print(Global_Temperatura_motore, 1); TELEMETRIA.print(","); // 31. Temp Motore, °C
        TELEMETRIA.print(temp_aria_barometro, 1);       TELEMETRIA.print(","); // 32. Temp Avionica/Teensy, °C
        
        // --- SATELLITI E COORDINATE GPS ---
        if (gps.location.isValid()) {
            TELEMETRIA.print(gps.satellites.value());  TELEMETRIA.print(","); // 33. Numero Satelliti (adimensionale)
            TELEMETRIA.print(gps.location.lat(), 6);   TELEMETRIA.print(","); // 34. Latitudine, gradi decimali (°)
            TELEMETRIA.print(gps.location.lng(), 6);   // 35. Longitudine, gradi decimali (°)
        } else {
            TELEMETRIA.print("0,0.000000,0.000000"); // Satelliti=0, Lat=0, Lon=0 (nessun fix)
        }
        TELEMETRIA.print(",");
        TELEMETRIA.print(relèAttivato ? "1" : "0");   // 36. Relè attivato (booleano 0/1)
        TELEMETRIA.print(",");

        // --- FLUSSO OTTICO (velocità stimata, in m/s) ---
        TELEMETRIA.print(G_vel_x_optical_sensor_ms, 2);  TELEMETRIA.print(","); // 37. Vel X flusso ottico [m/s]
        TELEMETRIA.print(G_vel_y_optical_sensor_ms, 2);  TELEMETRIA.print(","); // 38. Vel Y flusso ottico [m/s]

        // --- STATO SENSORI (bitmask: b0=flusso ottico OK, b1=LIDAR OK, b2=pacchetto SBUS perso) ---
        int statoSensori = 0;
        if (flusso_otticoOK)  statoSensori += 1;
        if (lidarOk)          statoSensori += 2;
        if (pacchettoPerso)   statoSensori += 4;
        TELEMETRIA.print(statoSensori);               TELEMETRIA.print(","); // 39. Bitmask stato sensori

        // --- NAVIGAZIONE (errore rotta) ---
        TELEMETRIA.print(G_errore_rotta_g, 1);          TELEMETRIA.print(","); // 40. Errore rotta, in gradi (°)

        // --- PROTEZIONE TERMICA MOTORE ---
        TELEMETRIA.print(G_limiteGasTermico);         TELEMETRIA.print(","); // 41. Limite gas termico, in µs

        // --- ALTITUDINI GREZZE (per diagnostica sensori, prima della fusione), in metri (m) ---
        TELEMETRIA.print(G_altitudine_lidar_m, 2);      TELEMETRIA.print(","); // 42. Altitudine LIDAR grezza [m]
        TELEMETRIA.print(G_altitudine_baro_m, 2);       TELEMETRIA.print(","); // 43. Altitudine Baro grezza [m]

        // --- Timestamp e contatore pacchetto (per rilevare pacchetti persi lato GCS) ---
        TELEMETRIA.print(tempoAttuale);               TELEMETRIA.print(","); // 44. millis() al momento dell'invio (ms da accensione Teensy)
        TELEMETRIA.print(numeroPacchettoTEL1);                                // 45. Numero progressivo pacchetto TEL1 (adimensionale)

        TELEMETRIA.println(); 
        numeroPacchettoTEL1++;

        // ── Pacchetti diagnostici supplementari (round-robin ogni ~2s, o forzati da REQ_DIAG) ──
        if (forzaInvioDiagnostica || (tempoAttuale - timerTelemetriaDiag > 2000)) {   // Ogni 2000 ms (2 s), salvo forzatura
            timerTelemetriaDiag = tempoAttuale;
            forzaInvioDiagnostica = false;

            if (contatorePacchettoDiag == 0) {
                // TEL2: correnti/potenze batterie + flusso ottico grezzo + gas pre-limite
                float pMotore  = sensoreMotore.getPower_mW();    // Potenza motore, in milliwatt (mW)
                float pTeensy  = sensoreTeensy.getPower_mW();    // Potenza Teensy, in milliwatt (mW)
                TELEMETRIA.print("$2,");
                TELEMETRIA.print(sensoreMotore.getCurrent_mA(), 1); TELEMETRIA.print(","); // corrente motore, mA
                TELEMETRIA.print(sensoreTeensy.getCurrent_mA(), 1); TELEMETRIA.print(","); // corrente Teensy, mA
                TELEMETRIA.print(G_correnteIntSX, 1);               TELEMETRIA.print(","); // corrente servo Int SX, mA
                TELEMETRIA.print(G_correnteIntDX, 1);               TELEMETRIA.print(","); // corrente servo Int DX, mA
                TELEMETRIA.print(G_correnteEstSX, 1);               TELEMETRIA.print(","); // corrente servo Est SX, mA
                TELEMETRIA.print(G_correnteEstDX, 1);               TELEMETRIA.print(","); // corrente servo Est DX, mA
                TELEMETRIA.print(pMotore, 1);                       TELEMETRIA.print(","); // potenza motore, mW
                TELEMETRIA.print(pTeensy, 1);                       TELEMETRIA.print(","); // potenza Teensy, mW
                TELEMETRIA.print(batteriaBassa_motore ? "1" : "0"); TELEMETRIA.print(","); // batteria motore bassa (bool)
                TELEMETRIA.print(batteriaBassa_teensy ? "1" : "0"); TELEMETRIA.print(","); // batteria Teensy bassa (bool)
                TELEMETRIA.print(G_flow_dx);                        TELEMETRIA.print(","); // flusso ottico dx grezzo (conteggi)
                TELEMETRIA.print(G_flow_dy);                        TELEMETRIA.print(","); // flusso ottico dy grezzo (conteggi)
                TELEMETRIA.print(G_comandoGasPreLimite);            TELEMETRIA.print(","); // gas comandato pre-limite termico, µs
                TELEMETRIA.print(G_limitazioneTermicaAttiva ? "1" : "0"); // limitazione termica attiva (bool)
                TELEMETRIA.println();

            } else if (contatorePacchettoDiag == 1) {
                // TEL3: diagnostica PID completa (unità: vedi dichiarazione variabili G_pid_* più in alto)
                TELEMETRIA.print("$3,");
                TELEMETRIA.print(G_pid_altErrore, 2);    TELEMETRIA.print(",");   // errore quota, m
                TELEMETRIA.print(G_pid_altP, 2);         TELEMETRIA.print(",");   // termine P quota (°, pitch target)
                TELEMETRIA.print(G_pid_altI, 2);         TELEMETRIA.print(",");   // termine I quota
                TELEMETRIA.print(G_pid_altD, 2);         TELEMETRIA.print(",");   // termine D quota
                TELEMETRIA.print(G_pid_targetPitchAuto, 2); TELEMETRIA.print(","); // pitch target risultante, °
                TELEMETRIA.print(G_pid_pitchErrore, 2);  TELEMETRIA.print(",");   // errore pitch, °
                TELEMETRIA.print(G_pid_pitchP, 2);       TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_pitchI, 2);       TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_pitchD, 2);       TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_rollErrore, 2);   TELEMETRIA.print(",");   // errore roll, °
                TELEMETRIA.print(G_pid_rollP, 2);        TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_rollI, 2);        TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_rollD, 2);        TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_velErrore, 2);    TELEMETRIA.print(",");   // errore velocità, km/h
                TELEMETRIA.print(G_pid_velP, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_velI, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(G_pid_velD, 2);         TELEMETRIA.print(",");
                TELEMETRIA.print(ALTITUDINE_TARGET_g, 1);  TELEMETRIA.print(",");   // quota target, m
                TELEMETRIA.print(G_targetVelocitaAttuale_km, 1);                     // velocità target, km/h
                TELEMETRIA.println();

            } else {
                // TEL4: GPS esteso, barometro esteso, pitot grezzo, IMU estesa, RC estesi
                aggiornaDiagnosticaIMU();   // Legge calibrazione+giroscopio SOLO qui (bassa frequenza, ~ogni 6 s)
                TELEMETRIA.print("$4,");
                // GPS esteso
                TELEMETRIA.print(gps.altitude.isValid() ? gps.altitude.meters() : -1.0, 1); TELEMETRIA.print(","); // Altitudine GPS, m (-1 se non valida)
                TELEMETRIA.print(gps.course.isValid() ? gps.course.deg() : -1.0, 1);        TELEMETRIA.print(","); // Rotta GPS, ° (-1 se non valida)
                TELEMETRIA.print(gps.speed.isValid() ? "1" : "0");   TELEMETRIA.print(",");   // Validità velocità GPS (bool)
                TELEMETRIA.print(gps.course.isValid() ? "1" : "0");  TELEMETRIA.print(",");   // Validità rotta GPS (bool)
                TELEMETRIA.print(gps.location.isValid() ? "1" : "0"); TELEMETRIA.print(","); // Validità posizione GPS (bool)
                // Barometro esteso
                TELEMETRIA.print(G_pressione_baro_pa, 1);   TELEMETRIA.print(","); // Pressione, Pa
                TELEMETRIA.print(G_tara_altitudine_m, 1);  TELEMETRIA.print(","); // tara ASL, m
                TELEMETRIA.print(baroPronto ? "1" : "0"); TELEMETRIA.print(",");
                // Pitot grezzo
                TELEMETRIA.print(G_pitot_raw);           TELEMETRIA.print(","); // conteggi ADC (0-1023)
                TELEMETRIA.print(VALORE_ZERO, 1);        TELEMETRIA.print(","); // conteggi ADC (zero calibrato)
                TELEMETRIA.print(G_pitot_differenza, 1); TELEMETRIA.print(","); // conteggi ADC (differenza)
                TELEMETRIA.print(G_pitot_valido ? "1" : "0"); TELEMETRIA.print(",");
                // IMU estesa
                TELEMETRIA.print(offsetPitch_g, 2); TELEMETRIA.print(","); // °
                TELEMETRIA.print(offsetRoll_g, 2);  TELEMETRIA.print(","); // °
                TELEMETRIA.print(offsetyaw_g, 2);   TELEMETRIA.print(","); // °
                TELEMETRIA.print(G_accelX, 2);    TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(G_accelY, 2);    TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(G_accelZ, 2);    TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(G_accelTotale, 2); TELEMETRIA.print(","); // m/s^2
                TELEMETRIA.print(G_gyroX, 2);     TELEMETRIA.print(","); // °/s
                TELEMETRIA.print(G_gyroY, 2);     TELEMETRIA.print(","); // °/s
                TELEMETRIA.print(G_gyroZ, 2);     TELEMETRIA.print(","); // °/s
                TELEMETRIA.print(G_imuCalSys);    TELEMETRIA.print(","); // 0-3
                TELEMETRIA.print(G_imuCalGyro);   TELEMETRIA.print(","); // 0-3
                TELEMETRIA.print(G_imuCalAccel);  TELEMETRIA.print(","); // 0-3
                TELEMETRIA.print(G_imuCalMag);    TELEMETRIA.print(","); // 0-3
                // RC estesi (canali 4-16, indici 3..15). Canali 1-3 già inviati in TEL1.
                for (int i = 3; i < 16; i++) {
                    TELEMETRIA.print(canaliRC[i]);
                    if (i < 15) TELEMETRIA.print(",");
                }
                TELEMETRIA.println();
            }
            contatorePacchettoDiag = (contatorePacchettoDiag + 1) % 3;   // Passa al prossimo pacchetto diagnostico nel ciclo TEL2->TEL3->TEL4->TEL2...
        }
    }
}

//  CALCOLO PID -----------------------------------------------------------------
void calcolaPID(float targetAltitudine, float targetRoll,
                float pitchReale, float rollReale,
                float velocitaAttuale, float targetVelocita,
                int gasDiBase,
                int &comandoPitchOut_g, int &comandoRollOut_g, int &comandoGasOut)
{
    // 1. CALCOLO DEL TEMPO
    unsigned long tempoAttuale = millis();
    float dt = (tempoAttuale - tempoPassatoPID) / 1000.0;   

    if (dt <= 0.001) return; // Evita divisioni per zero
    if (dt > 0.5) dt = 0.5;  // Evita lag improvvisi: limita dt max a 0.5 s 
    tempoPassatoPID = tempoAttuale;

    G_targetVelocitaAttuale_km = targetVelocita; 

    // 2. PID ALTITUDINE 
    float targetPitch_Auto_g = 0.0;   
    int gasCorrente = gasDiBase;      //  µs

    if (G_altitudine_m > ALTEZZA_MAX_m) {
        // Sopra la quota massima: forza un pitch negativo (scendi) e riduce il gas al minimo, ignorando il PID normale
        targetPitch_Auto_g = PITCH_DOWN_FORZATO;              // -8°, valore fisso
        gasCorrente = GAS_MINIMO; 
        pid_sommaErroriAlt =  0.0;             // Azzera l'integrale per evitare windup
        pid_errorePassatoAlt_ms=  0.0;
        G_pid_altErrore = 0.0; G_pid_altP = 0.0; G_pid_altI = 0.0; G_pid_altD = 0.0;
    } else if (G_altitudine_m < ALTEZZA_MIN_m) {
        // Sotto la quota minima: forza un pitch positivo (sali) e aumenta il gas quasi al massimo
        targetPitch_Auto_g = PITCH_UP_FORZATO;              // +12°, valore fisso
        gasCorrente = GAS_MASSIMO - 10; 
        pid_sommaErroriAlt =  0.0;
        pid_errorePassatoAlt_ms =  0.0;
        G_pid_altErrore = 0.0; G_pid_altP = 0.0; G_pid_altI = 0.0; G_pid_altD = 0.0;
    } else {
        // Quota nel range ammesso: calcolo PID normale
        float erroreAltitudine_m = targetAltitudine - G_altitudine_m;     
        erroreAltitudine_m = constrain(erroreAltitudine_m, -20.0, 20.0);     // Limitato a ±20 m per evitare comandi eccessivi

        float P_alt_g = Kp_alt * erroreAltitudine_m;                       

        pid_sommaErroriAlt += erroreAltitudine_m * dt;                      //  (m*s)
        pid_sommaErroriAlt  = constrain(pid_sommaErroriAlt, -20.0, 20.0); // Anti-windup: satura l'integrale
        float I_alt_g = Ki_alt * pid_sommaErroriAlt;                        

        float D_alt_g = Kd_alt * ((erroreAltitudine_m - pid_errorePassatoAlt_ms) / dt);  
        pid_errorePassatoAlt_ms = erroreAltitudine_m;

        targetPitch_Auto_g = constrain(P_alt_g + I_alt_g + D_alt_g, -10.0, 15.0); // Somma PID, limitata a [-10°, +15°]

        G_pid_altErrore = erroreAltitudine_m; G_pid_altP = P_alt_g; G_pid_altI = I_alt_g; G_pid_altD = D_alt_g;
    }
    G_pid_targetPitchAuto = targetPitch_Auto_g;

    // 3. PID PITCH
    float errorePitch_g = targetPitch_Auto_g - pitchReale;   

    float P_Pitch = Kp_pitch * errorePitch_g;

    pid_sommaErroriPitch += errorePitch_g * dt;
    pid_sommaErroriPitch  = constrain(pid_sommaErroriPitch, -40.0, 40.0);   // Anti-windup
    float I_Pitch = Ki_pitch * pid_sommaErroriPitch;

    float D_Pitch = Kd_pitch * ((errorePitch_g - pid_errorePassatoPitch_g) / dt);
    pid_errorePassatoPitch_g = errorePitch_g;

    comandoPitchOut_g = (int)(P_Pitch + I_Pitch + D_Pitch);        
    comandoPitchOut_g = constrain(comandoPitchOut_g, -MAX_PITCH_g, MAX_PITCH_g);

    G_pid_pitchErrore = errorePitch_g; G_pid_pitchP = P_Pitch; G_pid_pitchI = I_Pitch; G_pid_pitchD = D_Pitch;

    // 4. PID ROLL 
    float erroreRoll_g = targetRoll - rollReale;    

    float P_Roll = Kp_roll * erroreRoll_g;

    pid_sommaErroriRoll += erroreRoll_g * dt;
    pid_sommaErroriRoll  = constrain(pid_sommaErroriRoll, -40.0, 40.0);
    float I_Roll = Ki_roll * pid_sommaErroriRoll;

    float D_Roll = Kd_roll * ((erroreRoll_g - pid_errorePassatoRoll_g) / dt);
    pid_errorePassatoRoll_g = erroreRoll_g;

    comandoRollOut_g = (int)(P_Roll + I_Roll + D_Roll);     
    comandoRollOut_g  = constrain(comandoRollOut_g,  -MAX_ROLL_g,  MAX_ROLL_g);

    G_pid_rollErrore = erroreRoll_g; G_pid_rollP = P_Roll; G_pid_rollI = I_Roll; G_pid_rollD = D_Roll;

    // 5. AUTOTHROTTLE (PID velocità)
    float erroreVel_km = targetVelocita - velocitaAttuale;    

    float P_vel = Kp_vel * erroreVel_km;

    pid_sommaErroriVel += erroreVel_km * dt;
    pid_sommaErroriVel  = constrain(pid_sommaErroriVel, -30.0, 30.0);
    float I_vel = Ki_vel * pid_sommaErroriVel;

    float D_vel = Kd_vel * ((erroreVel_km - pid_errorePassatoVel_km) / dt);
    pid_errorePassatoVel_km = erroreVel_km;

    int gasCalcolato = gasCorrente + (int)(P_vel + I_vel + D_vel);   //µs
    comandoGasOut = constrain(gasCalcolato, GAS_MINIMO, GAS_MASSIMO);

    G_pid_velErrore = erroreVel_km; G_pid_velP = P_vel; G_pid_velI = I_vel; G_pid_velD = D_vel;
}

// Determina se il drone è "in volo"
void verifica_drone_in_volo() {          
    if (!droneInVolo) {
        bool velocitaSufficiente   = (G_Airspeed_ms > SOGLIA_VELO_DECOLLO_MS && G_Groundspeed_ms > SOGLIA_VELO_DECOLLO_MS);   
        bool altitudineSufficiente = (G_altitudine_m > SOGLIA_ALT_DECOLLO_M);                                                 

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

// Rileva un impatto tramite l'accelerazione lineare totale dell'IMU
void gestisciSchianto() {
    if (!schianto_sicurezza) {
        statoSchiantoRilevato = false;   
        return;
    }
    if (statoSchiantoRilevato) {
        motore.writeMicroseconds(GAS_NEUTRO);    // Se già rilevato uno schianto, tiene il motore forzatamente spento
        return; 
    }

    if (!droneInVolo) return;    // Non controlla schianti se il drone non è ancora considerato in volo (evita falsi positivi a terra)

    imu::Vector<3> accel = giroscopio.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);   // In m/s^2
    float accelerazioneTotale = sqrt((accel.x() * accel.x()) + (accel.y() * accel.y()) + (accel.z() * accel.z()));   // Modulo vettoriale, in m/s^2

    // Salvataggio per diagnostica/telemetria
    G_accelX = accel.x();
    G_accelY = accel.y();
    G_accelZ = accel.z();
    G_accelTotale = accelerazioneTotale;
    
    if (accelerazioneTotale > SOGLIA_G_SCHIANTO) {    // Sopra 50 m/s^2
        contatoreImpatto++;
        if (contatoreImpatto >= SEMPLE_VALORI_SCHIANTO) {   // Confermato dopo 3 CICLI di loop consecutivi 
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
        contatoreImpatto = 0;    // Reset 
    }
}

void aggiornaLidar() {
  // Solo se sotto margine di sicurezza (atterraggio/volo basso) — altrimenti il LIDAR  non è affidabile
  if (G_altitudine_baro_m > ALTEZZA_MAX_LIDAR_m) {   // Sopra 6 m di quota barometrica
    while (Serial2.available()) {
      Serial2.read();  // Svuota buffer (scarta i dati, non li elabora)
    }
    G_altitudine_lidar_m = -1.0f; 
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
      
      // Filtro passa-basso 
      if (G_altitudine_lidar_m < 0.0f) {
        G_altitudine_lidar_m = distanza_m; 
      } else {
        G_altitudine_lidar_m = ALPHA_LIDAR * distanza_m + (1.0f - ALPHA_LIDAR) * G_altitudine_lidar_m;  
      }
      set_up_lidar_alt_m = G_altitudine_lidar_m;  
      return;   
    }
  }
}

// Stima la velocità al suolo tramite il sensore di flusso ottico PMW3901, corretta per l'altezza e ruotata secondo lo yaw
void velocità_flusso_ottico(float angoloYaw_g) {
    static unsigned long tempoPassatoFlussoOttico = 0;

    int dx = 0;
    int dy = 0;
    flusso_ottico.readMotionCount(&dx, &dy);   // Conteggi grezzi di movimento ottico 
    G_flow_dx = dx; // salvataggio grezzo per diagnostica/telemetria
    G_flow_dy = dy;

    unsigned long now = millis();
    float dt = (now - tempoPassatoFlussoOttico) / 1000.0f;   
    tempoPassatoFlussoOttico = now;  


    if (dt <= 0.0f || G_altitudine_m > ALTEZZA_MAX_SENSORE_OTTICO_m) {   
        G_vel_x_optical_sensor_ms = -1.0f;   // -1 = valore non valido
        G_vel_y_optical_sensor_ms = -1.0f;
        return;
    }

    float v_x_ms = (dx * COSTANTE_OTTICA * G_altitudine_m) / dt;   
    float v_y_ms = (dy * COSTANTE_OTTICA * G_altitudine_m) / dt;   

    float yaw_rad = radians(angoloYaw_g);                        
    G_vel_x_optical_sensor_ms = v_x_ms * cos(yaw_rad) - v_y_ms * sin(yaw_rad);   
    G_vel_y_optical_sensor_ms = v_x_ms * sin(yaw_rad) + v_y_ms * cos(yaw_rad);  
}


void aggiorna_velocita(float Velocita_pitot_Ms, float velocita_gps_Ms) {
    // --- GROUNDSPEED flusso ottico se disponibile e a bassa quota, altrimenti il GPS ---
    if (G_vel_x_optical_sensor_ms != -1.0f && G_vel_y_optical_sensor_ms != -1.0f
        && G_altitudine_m < ALTEZZA_MAX_SENSORE_OTTICO_m) {
        
        G_Groundspeed_ms = sqrtf(G_vel_x_optical_sensor_ms * G_vel_x_optical_sensor_ms + 
                                 G_vel_y_optical_sensor_ms * G_vel_y_optical_sensor_ms);   
                                 
    } else if (gps.speed.isValid()) {
        G_Groundspeed_ms = velocita_gps_Ms;  
    } else {
        G_Groundspeed_ms = 0.0f;
    }
 
    if (Velocita_pitot_Ms > 0 && Velocita_pitot_Ms < MAX_AIRSPEED_X8_km/3.6f) {   // N
        G_Airspeed_ms = Velocita_pitot_Ms;   // In m/s
    } else {        
        if (gps.speed.isValid() || G_altitudine_m < ALTEZZA_MAX_SENSORE_OTTICO_m) {
            G_Airspeed_ms = max(G_Groundspeed_ms, 0.0f);
        } else {
            G_Airspeed_ms = 0.0f; // qui tocca pregare 
        }
    }
}

// LIDAR se disponibile e affidabile, altrimenti barometro
void aggiorna_altitudine(){
    if (lidarOk && G_altitudine_lidar_m > 0.0f && G_altitudine_baro_m < ALTEZZA_MAX_LIDAR_m) {
        G_altitudine_m = G_altitudine_lidar_m; 
    } else {
        G_altitudine_m = G_altitudine_baro_m;   
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
    batteriaBassa_teensy = (vTeensy < VALORE_BATT_TEENSY_BASSA_V);   
    if (batteriaBassa_teensy) {
        Serial.println("WARN: Batteria Teensy bassa");
        if (!relèAttivato) {
            digitalWrite(PIN_RELE, HIGH);    // Attiva il relè: la batteria motore subentra ad alimentare l'elettronica
            relèAttivato = true;
            Serial.println(">>> FAILOVER: Rele' attivato, subentra batteria motore");
        }
    }
    // ── BATTERIA MOTORE 
    batteriaBassa_motore = (vMotore < VALORE_BATT_MOTORE_BASSA_V);   
}


int gasMaxTermico() {
    if (Global_Temperatura_motore <= T_MOTORE_THROTTLE_START) {   // Sotto 70°C: nessun limite
        return GAS_MASSIMO;   
    }
    if (Global_Temperatura_motore >= T_MOTORE_THROTTLE_END) {     // Sopra 90°C: limite al minimo
        return GAS_MINIMO;
    }

    // Tra 70°C e 90°C: interpolazione lineare tra GAS_MASSIMO e GAS_MINIMO
    float t = (Global_Temperatura_motore - T_MOTORE_THROTTLE_START) /
              (T_MOTORE_THROTTLE_END   - T_MOTORE_THROTTLE_START);   // Fattore di interpolazione 0-1 (adimensionale)
    int limite = (int)(GAS_MASSIMO - t * (GAS_MASSIMO - GAS_MINIMO));   // Limite gas interpolato, in µs
    return limite;
}

// Legge comandi testuali (terminati da '\n') sia da USB (Serial) che da LoRa (TELEMETRIA) e li passa al parser
void comandi_da_terra() {
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

//  PARSER COMANDI ----------------------------------------------------------------
void elaboraComando(const String& cmd) {
    if (!cmd.startsWith("CMD:")) return;   // Tutti i comandi validi iniziano con "CMD:"

    int sep = cmd.indexOf(':', 4);   // Cerca il secondo ":" che separa il nome campo dal valore
    if (sep < 0) return;

    String campo     = cmd.substring(4, sep);     // Nome del campo/comando (es. "GAS", "SET_KP_ALT", ...)
    String valoreStr = cmd.substring(sep + 1);    // Valore associato, come stringa
    int    val       = valoreStr.toInt();         // Conversione a intero 

    //Servi: posizione (gradi, vincolati 45-135)
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

    //Servi: attach/detach manuale da terra
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

    // Gas — solo comandabile manualmente in modalità 1, in microsecondi (µs), vincolato tra GAS_NEUTRO e GAS_MASSIMO
    } else if (campo == "GAS") {
        if (global_modalitaVolo == 1) {
            motore.writeMicroseconds(constrain(val, GAS_NEUTRO, GAS_MASSIMO));   
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "non_in_manuale");
        }

    //Modalità di volo (1=Manuale, 2=Auto), rifiutata se in failsafe o valore fuori range
    } else if (campo == "MODO") {
        if (!failsafe && val >= 1 && val <= 2) {
            global_modalitaVolo = val;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "valore_non_valido_o_failsafe");
        }

    } else if (campo == "SET_LATITUDE") {
        float lat_g = valoreStr.toFloat();   
        if (lat_g >= -90.0 && lat_g <= 90.0) {
            TARGET_LAT_g = lat_g;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    } else if (campo == "SET_LONGITUDE") {
        float lon_g = valoreStr.toFloat();   
        if (lon_g >= -180.0 && lon_g <= 180.0) {
            TARGET_LON_g = lon_g;
            inviaAck(campo, valoreStr);
        } else {
            inviaNack(campo, "fuori_limite");
        }

    } else if (campo == "SET_ALTITUDE") {
        float alt_m = valoreStr.toFloat();   
        if (alt_m >= 0.0 && alt_m <= 500.0) {
            ALTITUDINE_TARGET_g = alt_m;
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
        if (v >= 20.0 && v <= 150.0) { VELOCITA_CROCIERA_km = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_VEL_AVVICINAMENTO") {
        float v = valoreStr.toFloat();   // km/h
        if (v >= 15.0 && v <= 150.0) { VELOCITA_AVVICINAMENTO_km = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_ALT_MIN") {
        float v = valoreStr.toFloat();   // metri (m)
        if (v >= 2.0 && v < ALTEZZA_MAX_m) { ALTEZZA_MIN_m = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite_o_maggiore_di_max");
    } else if (campo == "SET_ALT_MAX") {
        float v = valoreStr.toFloat();   // metri (m)
        if (v > ALTEZZA_MIN_m && v <= 500.0) { ALTEZZA_MAX_m = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite_o_minore_di_min");
    } else if (campo == "SET_LIMITE_TEMP_MOTORE") {
        float v = valoreStr.toFloat();   // gradi Celsius (°C)
        if (v > T_MOTORE_THROTTLE_START && v <= 120.0) { T_MOTORE_THROTTLE_END = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");
    } else if (campo == "SET_RAGGIO_WAYPOINT") {
        float v = valoreStr.toFloat();   // metri (m)
        if (v >= 5.0 && v <= 100.0) { RAGGIO_ACCETTAZIONE_MINIMO_m = v; inviaAck(campo, valoreStr); } else inviaNack(campo, "fuori_limite");

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
