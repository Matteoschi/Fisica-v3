#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <Adafruit_BMP3XX.h>

//  SENSORI
Adafruit_BNO055 giroscopio = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BMP3XX barometro;

//  TELEMETRIA LORA
#define TELEMETRIA Serial4
unsigned long timerTelemetria = 0;
const unsigned long INTERVALLO_TELEMETRIA_MS = 200; 

//  SERVO E PIN
const int pinSX = 6;
const int pinDX  = 7;
const int pinAvanti = 8;
const int pinDietro  = 9;
const int pinParacadute = 10;

Servo servo_SX;
Servo servo_DX;
Servo servo_Avanti;
Servo servo_Dietro;
Servo servo_Paracadute;

int CENTRO = 90;
int LIMITE_SERVO_MIN = 45;
int LIMITE_SERVO_MAX = 135;
int ANGOLO_SERVO_PARACADUTE_CHIUSO = 0;
int ANGOLO_SERVO_PARACADUTE_APERTO = 90;

//  VARIABILI DI VOLO
float posizione_partenza_roll = 0.0;
float posizione_partenza_pitch= 0.0;
float posizione_partenza_yaw = 0.0;

float global_altitudineDiPartenza = 0.0;
float global_altitudineDalSuolo = 0.0;
float global_altitudine_baro_raw = 0.0; 
float altitudine_massima  = 0.0;

bool paracadute_aperto = false;

//  ACCELEROMETRO (tutti e 3 gli assi)
float accel_x = 0.0; // asse longitudinale 
float accel_x_filtrata = 0.0;
float accel_y = 0.0; // laterale
float accel_z = 0.0; // laterale

//  PID — COSTANTI E STATO
float Kp = 1.5;
float Ki = 0.1;
float Kd = 0.5;
float LIMITE_INTEGRATORE = 100.0; // Anti-windup
float somma_errori_roll  = 0.0;
float errore_passato_roll  = 0.0;
float somma_errori_pitch = 0.0;
float errore_passato_pitch = 0.0;
float somma_errori_yaw   = 0.0;
float errore_passato_yaw = 0.0;

float PID_output_roll  = 0.0;
float PID_output_pitch = 0.0;
float PID_output_yaw   = 0.0;

//  FILTRO DI KALMAN (ALTITUDINE E VELOCITÀ)
float stima_altitudine = 0.0;
float stima_velocita   = 0.0;
const float K_alt = 0.15;
const float K_vel = 0.05;

//  LED E FLAGS
const int PIN_LED_ARMATO= 2;
const int PIN_LED_VERDE_DISARMATO = 3;

bool imuPronto = false;
bool baroPronto = false;
bool sensori_calibrati = false; 
int  tentativi = 0;
const int MAX_TENTATIVI = 3;

//  TIMING LOOP
unsigned long tempoPassato_loop = 0;
float dt_loop = 0.0; 

// ─────────────────────────────────────────────
//  PROTOTIPI FUNZIONI
// ─────────────────────────────────────────────
void  aggiornaKalman(float baro_alt_misurata, float accel_verticale, float dt);
void  calcolaPid(float angoloRoll, float angoloPitch, float angoloYaw, float dt);
void  mixer_servi(float out_Pitch, float out_Roll, float out_Yaw);
void  inviaTelemetria(float pitch, float roll, float yaw,float pid_pitch, float pid_roll, float pid_yaw,bool paracadute);

// ============================================================
//  SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); 

  Serial.println("Inizializzazione LoRa per Telemetria...");
  TELEMETRIA.begin(9600);
  Serial.println("Radio LoRa attivata su Serial4.");

  // ── Inizializzazione sensori con retry ──
  while ((!imuPronto || !baroPronto) && tentativi < MAX_TENTATIVI) {
      Serial.println("\n--- Controllo Sensori in corso ---");
      Serial.print("Tentativo numero: ");
      Serial.println(tentativi + 1);
      tentativi++;

      if (!imuPronto) {
          Serial.print("IMU (BNO055)........ ");
          if (giroscopio.begin()) {
              giroscopio.setExtCrystalUse(true);
              imuPronto = true;
              Serial.println("OK!");
          } else {
              Serial.println("FALLITO! (Controlla cavi I2C)");
          }
      }

      if (!baroPronto) {
          Serial.print("Barometro (BMP390).. ");
          if (barometro.begin_I2C()) {
              barometro.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
              barometro.setPressureOversampling(BMP3_OVERSAMPLING_32X);
              barometro.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
              barometro.setOutputDataRate(BMP3_ODR_50_HZ);
              baroPronto = true;
              Serial.println("OK!");
          } else {
              Serial.println("FALLITO! (Controlla cavi I2C)");
          }
      }
  }

  // ── Blocco in caso di errore critico ──
  if (!imuPronto || !baroPronto) {
      Serial.println("\nERRORE CRITICO: AVVIO BLOCCATO. Controlla i sensori.");
      while (1) {
          digitalWrite(PIN_LED_ARMATO, HIGH); delay(100);
          digitalWrite(PIN_LED_ARMATO, LOW);  delay(100);
      }
  }


  // ── Inizializzazione servi ──
  Serial.println("Inizializzazione Servomotori...");
  servo_SX.attach(pinSX);
  servo_DX.attach(pinDX);
  servo_Avanti.attach(pinAvanti);
  servo_Dietro.attach(pinDietro);
  servo_Paracadute.attach(pinParacadute);

  servo_SX.write(CENTRO);
  servo_DX.write(CENTRO);
  servo_Avanti.write(CENTRO);
  servo_Dietro.write(CENTRO);
  servo_Paracadute.write(ANGOLO_SERVO_PARACADUTE_CHIUSO);

  // ── LED di conferma ──
  pinMode(PIN_LED_ARMATO, OUTPUT);
  pinMode(PIN_LED_VERDE_DISARMATO, OUTPUT);
  digitalWrite(PIN_LED_ARMATO, HIGH);
  digitalWrite(PIN_LED_VERDE_DISARMATO, HIGH);
  delay(1000);
  digitalWrite(PIN_LED_ARMATO, LOW);
  digitalWrite(PIN_LED_VERDE_DISARMATO, LOW);

  tempoPassato_loop = millis();
  Serial.println("=== HARDWARE OK. IN ATTESA DI COMANDO DI CALIBRAZIONE ('K') ===");
  delay(1000);
}

void loop() {
    if (!sensori_calibrati) {
        leggiComandiDaTerra();
        return; 
    }
    unsigned long tempoAttuale = millis();
    if (tempoAttuale - tempoPassato_loop < 20) {
        return; 
    }
    dt_loop = (tempoAttuale - tempoPassato_loop) / 1000.0;

    if (dt_loop > 0.5) {
        tempoPassato_loop = tempoAttuale;
        return;
    }
    tempoPassato_loop = tempoAttuale;

    // ── 2. LETTURA IMU ──
    sensors_event_t event_ori;
    giroscopio.getEvent(&event_ori, Adafruit_BNO055::VECTOR_EULER);
    float angoloPitch = event_ori.orientation.y;
    float angoloRoll  = event_ori.orientation.z;
    float angoloYaw   = event_ori.orientation.x;

    float pitch_rad = angoloPitch * (PI / 180.0);
    float roll_rad  = angoloRoll * (PI / 180.0);

  // Accelerometro lineare 
  sensors_event_t event_accel;
  giroscopio.getEvent(&event_accel, Adafruit_BNO055::VECTOR_LINEARACCEL);
  accel_x = event_accel.acceleration.x; 
  accel_y = event_accel.acceleration.y;
  accel_z = event_accel.acceleration.z;

  accel_x_filtrata = (0.8 * accel_x_filtrata) + (0.2 * accel_x);
  
  float accel_verticale_vera = (accel_x_filtrata * cos(pitch_rad) * cos(roll_rad)) + (accel_y * sin(roll_rad)) + (accel_z * sin(pitch_rad));

  // ── 3. LETTURA BAROMETRO ──
  if (barometro.performReading()) {
      global_altitudine_baro_raw = barometro.readAltitude(1013.25);
      global_altitudineDalSuolo  = global_altitudine_baro_raw - global_altitudineDiPartenza;
  }

  aggiornaKalman(global_altitudineDalSuolo, accel_verticale_vera, dt_loop);
  leggiComandiDaTerra();
  inviaTelemetria(angoloPitch, angoloRoll, angoloYaw, PID_output_pitch, PID_output_roll, PID_output_yaw, paracadute_aperto);


  if (!razzo_armato_per_volo) {
    digitalWrite(PIN_LED_VERDE_DISARMATO, HIGH);
    digitalWrite(PIN_LED_ARMATO, LOW);
    return; 
  }else{
    digitalWrite(PIN_LED_VERDE_DISARMATO, LOW);
    digitalWrite(PIN_LED_ARMATO, HIGH);
  }

  // ── 5. TRACCIAMENTO ALTITUDINE MASSIMA ──
  if (stima_altitudine > altitudine_massima) {
      altitudine_massima = stima_altitudine;
  }
  
  // ── 6. LOGICA APOGEO E PARACADUTE ──
  if (!paracadute_aperto) {
      bool quota_minima_ok  = (stima_altitudine > 15.0);
      bool velocita_negativa= (stima_velocita < -0.5);
      bool caduta_dal_picco= ((altitudine_massima - stima_altitudine) > 1.5);

      if (quota_minima_ok && velocita_negativa && caduta_dal_picco) {
          servo_Paracadute.write(ANGOLO_SERVO_PARACADUTE_APERTO);
          paracadute_aperto = true;
          Serial.println(">>> APOGEO CONFERMATO! PARACADUTE ESPULSO! <<<");
          Serial.print("Altitudine apogeo stimata: ");
          Serial.print(altitudine_massima);
          Serial.println(" m");
      }
  }

  // ── 7. PID E AZIONAMENTO SERVI ──
  if (!paracadute_aperto) {
      calcolaPid(angoloRoll, angoloPitch, angoloYaw, dt_loop);
      mixer_servi(PID_output_pitch, PID_output_roll, PID_output_yaw);
  } else {
      // Frenata aerodinamica a paracadute aperto
      servo_SX.write(LIMITE_SERVO_MAX);
      servo_DX.write(LIMITE_SERVO_MAX);
      servo_Avanti.write(LIMITE_SERVO_MAX);
      servo_Dietro.write(LIMITE_SERVO_MAX);
  }
}

void aggiornaKalman(float baro_alt_misurata, float accel_verticale, float dt) {
    //  uniformemente accelerato
    float predizione_alt = stima_altitudine+ (stima_velocita * dt)+ (0.5 * accel_verticale * dt * dt);
    float predizione_vel = stima_velocita + (accel_verticale * dt);

    // FASE DI CORREZIONE 
    float errore = baro_alt_misurata - predizione_alt;

    stima_altitudine = predizione_alt + (K_alt * errore);
    stima_velocita   = predizione_vel + (K_vel * errore);
}

void calcolaPid(float angoloRoll, float angoloPitch, float angoloYaw, float dt) {
  const float MAX_PID_OUT = 30.0; 
  // --- ROLL ---
  float errore_roll = posizione_partenza_roll - angoloRoll;
  float p_roll = Kp * errore_roll;
  somma_errori_roll += errore_roll * dt;
  somma_errori_roll = constrain(somma_errori_roll, -LIMITE_INTEGRATORE, LIMITE_INTEGRATORE);
  float i_roll = somma_errori_roll * Ki;
  float d_roll = ((errore_roll - errore_passato_roll) / dt) * Kd;
  errore_passato_roll = errore_roll;
  // clamp
  PID_output_roll = constrain(p_roll + i_roll + d_roll, -MAX_PID_OUT, MAX_PID_OUT);

  // --- PITCH ---
  float errore_pitch = posizione_partenza_pitch - angoloPitch;
  float p_pitch = Kp * errore_pitch;
  somma_errori_pitch += errore_pitch * dt;
  somma_errori_pitch = constrain(somma_errori_pitch, -LIMITE_INTEGRATORE, LIMITE_INTEGRATORE);
  float i_pitch = somma_errori_pitch * Ki;
  float d_pitch = ((errore_pitch - errore_passato_pitch) / dt) * Kd;
  errore_passato_pitch = errore_pitch;
  // CLAMP
  PID_output_pitch = constrain(p_pitch + i_pitch + d_pitch, -MAX_PID_OUT, MAX_PID_OUT);

  // --- YAW ---
  float errore_yaw = posizione_partenza_yaw - angoloYaw;
  float p_yaw = Kp * errore_yaw;
  somma_errori_yaw += errore_yaw * dt;
  somma_errori_yaw = constrain(somma_errori_yaw, -LIMITE_INTEGRATORE, LIMITE_INTEGRATORE);
  float i_yaw = somma_errori_yaw * Ki;
  float d_yaw = ((errore_yaw - errore_passato_yaw) / dt) * Kd;
  errore_passato_yaw  = errore_yaw;
  // CLAMP
  PID_output_yaw = constrain(p_yaw + i_yaw + d_yaw, -MAX_PID_OUT, MAX_PID_OUT);
}


void mixer_servi(float out_Pitch, float out_Roll, float out_Yaw) {
    int posAvanti= CENTRO +(int)out_Pitch + (int)out_Roll;
    int posDietro= CENTRO -(int)out_Pitch- (int)out_Roll;
    int posSX = CENTRO +(int)out_Yaw + (int)out_Roll;
    int posDX = CENTRO - (int)out_Yaw - (int)out_Roll;

    posAvanti= constrain(posAvanti,LIMITE_SERVO_MIN, LIMITE_SERVO_MAX);
    posDietro= constrain(posDietro,LIMITE_SERVO_MIN, LIMITE_SERVO_MAX);
    posSX  = constrain(posSX,LIMITE_SERVO_MIN, LIMITE_SERVO_MAX);
    posDX = constrain(posDX,LIMITE_SERVO_MIN, LIMITE_SERVO_MAX);

    servo_Avanti.write(posAvanti);
    servo_Dietro.write(posDietro);
    servo_SX.write(posSX);
    servo_DX.write(posDX);
}

void inviaTelemetria(float pitch, float roll, float yaw, float pid_pitch, float pid_roll, float pid_yaw, bool paracadute) {
  unsigned long tempoAttuale = millis();
  if (tempoAttuale - timerTelemetria < INTERVALLO_TELEMETRIA_MS) {
      return;
  }
  timerTelemetria = tempoAttuale;

  int pos_avanti = servo_Avanti.read();
  int pos_dietro = servo_Dietro.read();
  int pos_sx     = servo_SX.read();
  int pos_dx     = servo_DX.read();

  TELEMETRIA.print("$,"); 
  TELEMETRIA.print(tempoAttuale);
  TELEMETRIA.print(",");
  TELEMETRIA.print(global_altitudineDalSuolo, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(stima_altitudine, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(stima_velocita, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(altitudine_massima, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pitch, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(roll, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(yaw, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(accel_x, 3);
  TELEMETRIA.print(",");
  TELEMETRIA.print(accel_y, 3);
  TELEMETRIA.print(",");
  TELEMETRIA.print(accel_z, 3);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pid_pitch, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pid_roll, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pid_yaw, 2);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pos_avanti);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pos_dietro);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pos_sx);
  TELEMETRIA.print(",");
  TELEMETRIA.print(pos_dx);
  TELEMETRIA.print(",");
  TELEMETRIA.print((int)(dt_loop * 1000));
  TELEMETRIA.print(",");
  TELEMETRIA.println(paracadute ? 1 : 0); 
}

bool razzo_armato_per_volo = false;

void leggiComandiDaTerra() {
    if (Serial.available() > 0) {
        char comando = Serial.read(); // Leggi la lettera digitata
        if (!sensori_calibrati)
        {
            Serial.println("Sensori non ancora calibrati , loop inaccessibile");
        }
        

        switch (comando) {
            case 'K': 
            case 'k':
                if (razzo_armato_per_volo) {
                    Serial.println("\n[GCS] ERRORE: Disarma il razzo prima di calibrare!");
                } else {
                    eseguiCalibrazionePad();
                }
                break;

            case 'A': 
                if (!sensori_calibrati) {
                    Serial.println("\n[GCS] ERRORE FATALE: Sensori non calibrati. Esegui comando 'K' prima di armare.");
                } else {
                    razzo_armato_per_volo = true;
                    Serial.println("\n[GCS] ATTENZIONE: SISTEMA ARMATO! PRONTO AL LANCIO!");
                }
                break;

            case 'S': 
                razzo_armato_per_volo = false;
                Serial.println("\n[GCS] Sistema DISARMATO (Safe). Lancio bloccato.");
                break;

            // --- TUNING PID ---
            case 'P': // Kp
            case 'p':
                Kp = Serial.parseFloat();
                Serial.print("\n[LIVE TUNING] Nuovo Kp impostato a: "); 
                Serial.println(Kp);
                break;

            case 'I': //Ki
            case 'i':
                Ki = Serial.parseFloat();
                Serial.print("\n[LIVE TUNING] Nuovo Ki impostato a: "); 
                Serial.println(Ki);
                break;

            case 'D': //r Kd
            case 'd':
                Kd = Serial.parseFloat();
                Serial.print("\n[LIVE TUNING] Nuovo Kd impostato a: "); 
                Serial.println(Kd);
                break;
            
            // --- TUNING MECCANICO SERVI ---
            case 'C': //  CENTRO
            case 'c':
                CENTRO = Serial.parseInt();
                Serial.print("\n[LIVE TUNING] Nuovo CENTRO servo impostato a: "); 
                Serial.println(CENTRO);
                break;
            
            case 'M': //  LIMITE MIN
            case 'm':
                LIMITE_SERVO_MIN = Serial.parseInt();
                Serial.print("\n[LIVE TUNING] Nuovo LIMITE_SERVO_MIN impostato a: "); 
                Serial.println(LIMITE_SERVO_MIN);
                break;

            case 'N': // LIMITE MAX
            case 'n':
                LIMITE_SERVO_MAX = Serial.parseInt();
                Serial.print("\n[LIVE TUNING] Nuovo LIMITE_SERVO_MAX impostato a: "); 
                Serial.println(LIMITE_SERVO_MAX);
                break;
            
            case 'O': //  Paracadute CHIUSO
            case 'o':
                ANGOLO_SERVO_PARACADUTE_CHIUSO = Serial.parseInt();
                Serial.print("\n[LIVE TUNING] Nuovo ANGOLO_PARACADUTE_CHIUSO impostato a: ");
                Serial.println(ANGOLO_SERVO_PARACADUTE_CHIUSO);
                break;
            
            // --- LETTURE E TEST ---
            case 'V': // 'V' per Visualizzare i valori attuali
                Serial.println("\n[GCS] Costanti attuali:");
                Serial.print("Kp: "); Serial.println(Kp);
                Serial.print("Ki: "); Serial.println(Ki);
                Serial.print("Kd: "); Serial.println(Kd);
                Serial.print("Centro: "); Serial.println(CENTRO);
                Serial.print("Min/Max: "); Serial.print(LIMITE_SERVO_MIN); Serial.print(" / "); Serial.println(LIMITE_SERVO_MAX);
                break;

            case 'B': // Digita 'B' per leggere il Barometro
                Serial.println("\n[GCS] Dati Sensori:");
                Serial.print("Altitudine da terra: "); Serial.print(global_altitudineDalSuolo); Serial.println(" m");
                Serial.print("Velocità stimata: "); Serial.print(stima_velocita); Serial.println(" m/s");
                break;

            case 'T': // Digita 'T' per il TEST DEI SERVI (Wiggle Test)
                if (razzo_armato_per_volo) {
                    Serial.println("\n[GCS] ERRORE: Non puoi testare i servi se il razzo è armato!");
                } else {
                    Serial.println("\n[GCS] TEST SERVI: Movimento in corso...");
                    servo_Avanti.write(LIMITE_SERVO_MAX); servo_Dietro.write(LIMITE_SERVO_MAX);
                    servo_SX.write(LIMITE_SERVO_MAX); servo_DX.write(LIMITE_SERVO_MAX);
                    delay(500);
                    servo_Avanti.write(CENTRO); servo_Dietro.write(CENTRO);
                    servo_SX.write(CENTRO); servo_DX.write(CENTRO);
                    Serial.println("[GCS] Test servi completato. OK.");
                }
                break;
                
            case 'X': 
                if (!razzo_armato_per_volo) {
                    Serial.println("\n[GCS] TEST PARACADUTE...");
                    servo_Paracadute.write(ANGOLO_SERVO_PARACADUTE_APERTO);
                    delay(1000);
                    servo_Paracadute.write(ANGOLO_SERVO_PARACADUTE_CHIUSO);
                    Serial.println("[GCS] Paracadute testato e richiuso.");
                } else {
                    Serial.println("\n[GCS] ERRORE: Paracadute bloccato. Sistema armato.");
                }
                break;
        }
    }
}

void eseguiCalibrazionePad() {
    Serial.println("\n[GCS] Inizio Calibrazione... NON TOCCARE IL RAZZO!");

    // 1. Fase di stabilizzazione sensori (2 secondi)
    for (int j = 0; j < 10; j++) {
        digitalWrite(PIN_LED_VERDE_DISARMATO, HIGH);
        delay(100);
        digitalWrite(PIN_LED_VERDE_DISARMATO, LOW);
        delay(100);
    }

    // Altitudine di partenza: media di 20 campioni
    float sommaAlt   = 0.0;
    int   campioniOK = 0;
    
    // 2. Fase di campionamento altimetro
    Serila.print("\n Calibrazione altimetro");
    for (int i = 0; i < 20; i++) {
        if (i % 2 == 0) {
            digitalWrite(PIN_LED_VERDE_DISARMATO, HIGH);
        } else {
            digitalWrite(PIN_LED_VERDE_DISARMATO, LOW);
        }

        if (barometro.performReading()) {
            sommaAlt += barometro.readAltitude(1013.25);
            campioniOK++;
        }
        delay(50); 
    }
    
    global_altitudineDiPartenza = (campioniOK > 0) ? (sommaAlt / campioniOK) : 0.0;
    stima_altitudine = 0.0; 
    stima_velocita   = 0.0;

    Serial.print("\n [GCS] Altitudine base acquisita: ");
    Serial.print(global_altitudineDiPartenza);
    Serial.println(" m");

    Serial.print("\n Calibazione assetto di partenza");
    sensors_event_t event;
    giroscopio.getEvent(&event);
    posizione_partenza_roll  = event.orientation.z;
    posizione_partenza_pitch = event.orientation.y;
    posizione_partenza_yaw   = event.orientation.x;

    digitalWrite(PIN_LED_VERDE_DISARMATO, HIGH);
    digitalWrite(PIN_LED_ARMATO, HIGH);
    Serial.println("[GCS] Assetto Zero acquisito.");
    sensori_calibrati = true; 
    Serial.println("[GCS] CALIBRAZIONE COMPLETATA. Pronto per l'Armamento.");
    digitalWrite(PIN_LED_VERDE_DISARMATO, LOW);
    digitalWrite(PIN_LED_ARMATO, LOW);
    tempoPassato_loop = millis();
}