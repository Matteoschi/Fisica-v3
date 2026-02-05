#include <stdio.h>
#include <math.h>
#include "costanti_missile.h"

// --- Funzioni Matematiche Ausiliarie ---

double modulo_vettore(double vettore[3]) { 
    return sqrt(vettore[0]*vettore[0] + vettore[1]*vettore[1] + vettore[2]*vettore[2]); 
}

void prodotto_vettore_scalare(double vettore[3], double scalare, double out[3]) {
    for(int i=0; i<3; i++)
    {
        out[i] = vettore[i] * scalare;
    }
}

double prodotto_scalare(double a[3], double b[3]) {
    double risultato = 0.0;
    for (int i = 0; i < 3; i++)
    {
        risultato += a[i] * b[i];
    }
    return risultato;
}

void prodotto_vettroiale(double a[3], double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}


double calcola_fisica_step(
    double posM[3], double velM[3], double massa_attuale,
    double posT[3], double velT[3], 
    double spinta_attuale, 
    double drag_coeff,
    double acc_out[3],     
    int Endgame_Strategy,   
    double dati_debug[14]    
) {
    int stato_sicurezza = STATUS_OK;
    // 1. Calcolo Distanza e Vettori base
    double Vettore_linea_vista[3], dist_sq = 0.0;
    for(int i=0; i<3; i++) {
        Vettore_linea_vista[i] = posT[i] - posM[i];
        dist_sq += Vettore_linea_vista[i]*Vettore_linea_vista[i];
    }
    double distanza = sqrt(dist_sq); // TROVATA LA DISTANZA TRA MISSILE E TARGET
    
    // Protezione divisione per zero
    if (distanza < EPSILON) {
        distanza = EPSILON;
    }

    double Versore_los[3]; // vettore direzione verso il target l=1
    
    for(int i=0; i<3; i++) {
        Versore_los[i] = Vettore_linea_vista[i] / distanza; // normalizzazione r/|r|
    }

    double velocità_relativa[3]; // velocità relativa
    for(int i=0; i<3; i++) {
        velocità_relativa[i] = velT[i] - velM[i]; //differenza velocità
    }

    // =============================================================================
    // 4. Guida Proporzionale (Pro-Nav) Acmd=K*Vc*(ω_LOS x u_LOS)
    // =============================================================================
    
    double velocità_chiusura = -prodotto_scalare(velocità_relativa, Versore_los); //prodotto scalare Vc=d/dt|r|
    
    double accelerazione_comandata_legge_guida[3] = {0,0,0};

    if (velocità_chiusura > 0) { 
        double cross_r_v[3], velocità_angolare_los[3], cross_omega_los[3];
        prodotto_vettroiale(Vettore_linea_vista, velocità_relativa, cross_r_v); // calcolo r x V
        prodotto_vettore_scalare(cross_r_v, 1.0 / dist_sq, velocità_angolare_los); // calcolo velocità angolare LOS ω_LOS = (r x V) / |r|^2
        prodotto_vettroiale(velocità_angolare_los, Versore_los, cross_omega_los);
        prodotto_vettore_scalare(cross_omega_los, COSTANTE_NAVIGAZIONE * velocità_chiusura, accelerazione_comandata_legge_guida); // risultato finale Acmd
    }

    // 4. Forze Fisiche
    double modulo_velocità_missile = modulo_vettore(velM);
    double versore_velocità[3] = {1,0,0};
    if (modulo_velocità_missile > EPSILON) {
        for(int i=0; i<3; i++) {
            versore_velocità[i] = velM[i] / modulo_velocità_missile; //normalizzando la velocità.
        } 
    } else {
        // Se fermo al lancio, punta verso il target
        for(int i=0; i<3; i++) { 
            versore_velocità[i] = Versore_los[i];
        }
    }

    // ==========================================
    // A. CALCOLO PARAMETRI ATMOSFERICI
    // ==========================================
    double altitudine = posM[2]; 
    if (altitudine < 0) {
        altitudine = 0; 
    }

    // Atmosfera Standard USSA-1976 (Modello a due strati)
    double pressione_attuale;
    double temperatura_kelvin;
    double densita_aria;

    if (altitudine < LIMITE_TROPOSFERA_STRATOSFERA) {
        temperatura_kelvin = TEMPERATURA_LIV_MARE - (GRADIENTE_TERMICO_TROPOSFERA * altitudine);
        double esponente = G_ACCEL / (R_GAS * GRADIENTE_TERMICO_TROPOSFERA);
        // 3. Pressione: P = P0 * (T / T0)^esponente
        pressione_attuale = PRESSIONE_SEA_LEVEL * pow(temperatura_kelvin / TEMPERATURA_LIV_MARE, esponente);
        // 4. Densità: rho = P / (R * T)
        densita_aria = pressione_attuale / (R_GAS * temperatura_kelvin);
    } 
    else if (altitudine < 20000.0) {
        // 1. Temperatura fissa
        temperatura_kelvin = TEMPERATURA_STRATOSFERA; // 216.65 K
        // P = P_11k * e^(-g * delta_h / (R * T_strato))
        double delta_h = altitudine - LIMITE_TROPOSFERA_STRATOSFERA;
        double argomento_exp = (-G_ACCEL * delta_h) / (R_GAS * TEMPERATURA_STRATOSFERA);
        pressione_attuale = PRESSIONE_11KM * exp(argomento_exp);
        // 4. Densità
        densita_aria = pressione_attuale / (R_GAS * temperatura_kelvin);
    }
    else {
        temperatura_kelvin = TEMPERATURA_STRATOSFERA;
        densita_aria = 0.088; 
    }

    // Velocità del suono (Gas ideale): a = sqrt(gamma * R * T)
    double velocità_suono = sqrt(GAMMA * R_GAS * temperatura_kelvin);
    
    // Numero di Mach: M = v / a
    double numero_di_mach = modulo_velocità_missile / velocità_suono;

    // Pressione Dinamica (Bernoulli): q = 1/2 * rho * v^2 quantità energetica cinetica per unità di volume
    double pressione_dinamica = 0.5 * densita_aria * modulo_velocità_missile * modulo_velocità_missile;

    // Calcolo Temperatura di Ristagno
    double temperatura_ristagno = temperatura_kelvin * (1.0 + ((GAMMA - 1.0) / 2.0) * numero_di_mach * numero_di_mach);

    //Controllo Proattivo della Temperatura (Soft Limiter)
    double soglia_inizio_intervento = MAX_TEMP_STRUTTURA - MARGINE_SICUREZZA_TEMPERATURA; 
    
    double fattore_spinta = 1.0; 

    if (!OVERCLOCK && !Endgame_Strategy) {
        if (temperatura_ristagno > soglia_inizio_intervento) {
            double delta_temp = temperatura_ristagno - soglia_inizio_intervento;
            fattore_spinta = 1.0 - (delta_temp / MARGINE_SICUREZZA_TEMPERATURA);
            if (fattore_spinta < 0.0) fattore_spinta = 0.0;
            if (temperatura_ristagno > MAX_TEMP_STRUTTURA) {
                stato_sicurezza |= STATUS_LIMIT_SPEED; 
            } else {
                stato_sicurezza |= STATUS_LIMIT_TEMPERATURE;
            }
        }
    }else {        
        if (temperatura_ristagno > soglia_inizio_intervento) {
             if (temperatura_ristagno > MAX_TEMP_STRUTTURA) {
                    stato_sicurezza |= STATUS_OVERCLOCK_SPEED;
             } else {
                    stato_sicurezza |= STATUS_OVERCLOCK_TEMPERATURE;
             }
        }
    }

    double spinta_modulata = spinta_attuale * fattore_spinta;

    // ==========================================
    // 6. EFFICIENZA MOTORE (Compensazione Quota)
    // ==========================================

    // Calcolo Pressione Atmosferica Locale
    double pressione_locale = densita_aria * R_GAS * temperatura_kelvin;
    
    // Variabile che useremo per la fisica F=ma
    double spinta_motore_totale = 0.0; 

    // calcoliamo il bonus di quota
    if (spinta_modulata > EPSILON) {
        double spinta_bonus = (PRESSIONE_SEA_LEVEL - pressione_locale) * AREA_UGELLO; 
        if (spinta_bonus < 0) spinta_bonus = 0; 
        spinta_motore_totale = spinta_modulata + spinta_bonus; 
    }

    // ==========================================
    // 7. COEFFICIENTE DRAG BASE (Cd0)
    // ==========================================
    // Resistenza d'onda 
    double mach_multiplier = 1.0;
    if (numero_di_mach >= 0.8 && numero_di_mach < 1.2) {
        mach_multiplier = 1.0 + (numero_di_mach - 0.8) * K_WAVE; 
    } 
    double coefficiente_drag_aria = drag_coeff * mach_multiplier; // Questo è il Cd0

    // ==========================================
    // 8. Flight Envelope Protection
    // ==========================================
    
    // sccelerazione sterzande richiesta
    double guide_force = modulo_vettore(accelerazione_comandata_legge_guida);

    // Equazione della Portanza (allo stallo): L_max = q * S * CL_max
    double max_forza_aero = pressione_dinamica * AREA_MISSILE * LIMITE_STALLO;
    
    // Seconda Legge di Newton: a_max = F_max / m
    double max_acc_aerodinamica = max_forza_aero / massa_attuale;

    //Vincoli di Integrità Strutturale
    double max_acc_strutturale = LIMIT_G_LOAD * G_ACCEL;

    // Il limite è il minimo tra la resistenza dei materiali e la portanza disponibile
    double limite_reale = max_acc_strutturale;
    

    // 1. Calcolo del limite reale
    int limitato_da_aerodinamica = 0; 

    if (max_acc_aerodinamica < max_acc_strutturale) {
        limite_reale = max_acc_aerodinamica;
        limitato_da_aerodinamica = 1; 
    }

    if (guide_force > limite_reale) {
            if (!OVERCLOCK && !Endgame_Strategy) {
                if (limite_reale < EPSILON) { 
                    for(int i=0; i<3; i++) accelerazione_comandata_legge_guida[i] = 0.0;
                    guide_force = 0.0;
                    stato_sicurezza |= STATUS_LIMIT_SPEED; 
                } 
                else {
                    double ratio = limite_reale / guide_force;
                    prodotto_vettore_scalare(accelerazione_comandata_legge_guida, ratio, accelerazione_comandata_legge_guida); 
                    guide_force = limite_reale;
                    if (limitato_da_aerodinamica) {
                        stato_sicurezza |= STATUS_LIMIT_AERO; 
                    } else {
                        stato_sicurezza |= STATUS_LIMIT_STRUCTURAL; 
                    }
                }
            } 
            else {
                if (guide_force > max_acc_aerodinamica) {
                    double ratio = max_acc_aerodinamica / guide_force;
                    prodotto_vettore_scalare(accelerazione_comandata_legge_guida, ratio, accelerazione_comandata_legge_guida);
                    guide_force = max_acc_aerodinamica;
                    stato_sicurezza |= STATUS_OVERCLOCK_AERO;
                }
                else {
                    stato_sicurezza |= STATUS_OVERCLOCK_STRUCTURAL;
                }
            }
        }

    double prodotto_boresight = prodotto_scalare(versore_velocità, Versore_los);
    double limite_coseno = cos(ANG_GRADI_CONO_VISIONE * (PI / 180.0));

    if (pressione_dinamica < 5000.0) {
        for(int i=0; i<3; i++) accelerazione_comandata_legge_guida[i] = 0.0;
        stato_sicurezza |= STATUS_LIMIT_SPEED; 
    } 
    else if (prodotto_boresight < limite_coseno) {
        for(int i=0; i<3; i++) accelerazione_comandata_legge_guida[i] = 0.0;
        stato_sicurezza |= STATUS_LOCK_LOST; 
    }
    // ==========================================
    // 9. Bilancio Resistivo
    // ==========================================
    
    double Coefficiente_Portanza_richiesto = 0.0;
    
    if (pressione_dinamica > EPSILON) {        
        double forza_Portanza_richiesta = massa_attuale * guide_force;// Newton: F_lift = m * acmd        
        Coefficiente_Portanza_richiesto = forza_Portanza_richiesta / (pressione_dinamica * AREA_MISSILE);// Equazione Portanza Inversa: CL = L / (q * S)
        
        // LIMITAZIONE: Cl non può superare il massimo fisico (stallo)
        if (Coefficiente_Portanza_richiesto > LIMITE_STALLO) {
            Coefficiente_Portanza_richiesto = LIMITE_STALLO;
        }
    }

    //Cd = Cd0 + k * CL^2 dovuto ai vortici di estremità ala quando si gira
    double resistenza_indotta = INDUCED_DRAG_K * (Coefficiente_Portanza_richiesto * Coefficiente_Portanza_richiesto); 

    double coefficiente_attrito_tot = coefficiente_drag_aria + resistenza_indotta;

    // Calcolo della Forza Resistiva Finale (Drag): D = q * S * Cd , pressione_dinamica 0.5 * rho * v^2
    double drag_force_mag = pressione_dinamica * AREA_MISSILE * coefficiente_attrito_tot;
    
    //Calcolo Accelerazione Totale 
    for(int i=0; i<3; i++) {
        double f_thrust = versore_velocità[i] * spinta_motore_totale; // Forza di spinta = Spinta * versore_velocità
        double f_drag   = -versore_velocità[i] * drag_force_mag; // Forza di drag = -Versore_velocità * Drag_Mag (opppos)
        double f_grav   = (i == 2) ? -G_ACCEL * massa_attuale : 0.0; // Solo asse Z
        
        // Newton: a = SommaForze / Massa_Variabile
        acc_out[i] = (f_thrust + f_drag + f_grav) / massa_attuale;
        
        // Aggiungi l'accelerazione comandata dalle alette (Guida)
        acc_out[i] += accelerazione_comandata_legge_guida[i];
    }

    if (dati_debug != NULL) {
        double g_load_val = modulo_vettore(accelerazione_comandata_legge_guida) / G_ACCEL; // G-Force
        dati_debug[0] = distanza;          // Distanza Target-Missile
        dati_debug[1] = guide_force;        // guide_force
        dati_debug[2] = densita_aria;      // Densità Aria Attuale
        dati_debug[3]= temperatura_kelvin; // Temperatura Attuale
        dati_debug[4]=temperatura_ristagno; // Temperatura di Ristagno
        dati_debug[5]= numero_di_mach;        // Numero di Mach Attuale
        dati_debug[6]= pressione_dinamica;          // Pressione Dinamica Attuale
        dati_debug[7]= pressione_locale;          // Pressione Atmosferica Attuale
        dati_debug[8]= spinta_motore_totale;          // Spinta Motore Attuale
        dati_debug[9]=spinta_attuale;          // Spinta Motore Base
        dati_debug[10]= coefficiente_attrito_tot;          // Coefficiente Drag Totale
        dati_debug[11]= Coefficiente_Portanza_richiesto;         // Coefficiente Lift Attuale
        dati_debug[12] = stato_sicurezza;          // Stato di Sicurezza
    }
    
    return fattore_spinta;
}

// =============================================================================
// FUNZIONE PRINCIPALE (INTEGRATORE RK4)
// =============================================================================

double aggiorna_missile(
    double posM[3], double velM[3], 
    double posT[3], double velT[3], 
    double dt, 
    double tempo_corrente,      // (Solo per debug/log)
    double massa_totale_launch, // (Non usato, calcoliamo dinamico)
    double massa_carburante,    // (Non usato, usiamo il puntatore)
    double durata_motore,       // (Non usato, dipende dal fuel)
    double massa_a_vuoto,       // Peso del missile a secco (Costante)
    double *ptr_massa_fuel,     // PUNTATORE alla benzina rimasta (Input/Output)
    double consumo_kg_s,        // Rateo di consumo a piena potenza (kg/s)
    double spinta_nominale,     // Spinta massima in Newton
    double drag_coeff,
    double dati_output[14]      // Array output esteso per debug
) {
    double diff_temp[3], dist_sq_temp = 0.0;
    for(int i=0; i<3; i++) {
        diff_temp[i] = posT[i] - posM[i];
        dist_sq_temp += diff_temp[i]*diff_temp[i];
    }
    double distanza_stimata_inizio = sqrt(dist_sq_temp);

    // Decisione Strategica
    int modalita_terminale = 0;
    if (distanza_stimata_inizio < SOGLIA_ENDGAME) { // <--- SOGLIA ENDGAME
        modalita_terminale = 1;
    }else {
        modalita_terminale = 0;
    }

    // --- 2. GESTIONE MASSA E SPINTA (Dal serbatoio) ---
    double fuel_attuale = *ptr_massa_fuel; // Leggiamo valore attuale dal puntatore
    double spinta_disponibile = spinta_nominale;

    // Se il serbatoio è vuoto o negativo , spegniamo il motore
    if (fuel_attuale <= 1e-6) {
        fuel_attuale = 0.0;
        spinta_disponibile = 0.0; 
    }

    // Massa Totale F=ma (Massa Secca + Benzina rimasta)
    double massa_totale_fisica = massa_a_vuoto + fuel_attuale;

    // =============================================================================
    // RUNGE-KUTTA 4° ORDINE (RK4)
    // =============================================================================
    
    double k1_acc[3], k2_acc[3], k3_acc[3], k4_acc[3];
    double k1_vel[3], k2_vel[3], k3_vel[3], k4_vel[3];
    double pos_temp[3], vel_temp[3];
    double massa_temp;
    
    // --- STEP K1 (Stato Iniziale) ---
    for(int i=0; i<3; i++) k1_vel[i] = velM[i];
    
    // NOTA IMPORTANTE: Qui catturiamo il 'throttle_factor' (0.0 - 1.0) 
    // restituito da calcola_fisica_step (che contiene la logica termica)
    double throttle_usato = calcola_fisica_step(posM, velM, massa_totale_fisica, posT, velT, spinta_disponibile, drag_coeff, k1_acc, modalita_terminale,dati_output);

    // --- STEP K2 (Metà strada) ---
    for(int i=0; i<3; i++) {
        pos_temp[i] = posM[i] + k1_vel[i] * 0.5 * dt;
        vel_temp[i] = velM[i] + k1_acc[i] * 0.5 * dt; 
    }
    for(int i=0; i<3; i++) k2_vel[i] = vel_temp[i];
    massa_temp = massa_totale_fisica - (consumo_kg_s * throttle_usato * 0.5 * dt);
    
    calcola_fisica_step(pos_temp, vel_temp, massa_temp, posT, velT, spinta_disponibile, drag_coeff, k2_acc,modalita_terminale, NULL);

    // --- STEP K3 (Metà strada) ---
    for(int i=0; i<3; i++) {
        pos_temp[i] = posM[i] + k2_vel[i] * 0.5 * dt;
        vel_temp[i] = velM[i] + k2_acc[i] * 0.5 * dt; 
    }
    for(int i=0; i<3; i++) k3_vel[i] = vel_temp[i];
    massa_temp = massa_totale_fisica - (consumo_kg_s * throttle_usato * 0.5 * dt);
    
    calcola_fisica_step(pos_temp, vel_temp, massa_temp, posT, velT, spinta_disponibile, drag_coeff, k3_acc, modalita_terminale,NULL);

    // --- STEP K4 (Fine strada) ---
    for(int i=0; i<3; i++) {
        pos_temp[i] = posM[i] + k3_vel[i] * dt;     
        vel_temp[i] = velM[i] + k3_acc[i] * dt;
    }
    for(int i=0; i<3; i++) k4_vel[i] = vel_temp[i];
    massa_temp = massa_totale_fisica - (consumo_kg_s * throttle_usato * dt);

    calcola_fisica_step(pos_temp, vel_temp, massa_temp, posT, velT, spinta_disponibile, drag_coeff, k4_acc, modalita_terminale, NULL);





// --- GESTIONE MANOVRA TARGET (Zoom Climb 62k ft/min) ---
// --- PARAMETRI MANOVRA F-22 RAPTOR ---
    double soglia_x_manovra = 1000.0;    // Trigger evasione (m)
    
    // Performance F-22
    double max_g_load = 9.0 * 9.81;      // 9G Operativi (standard evasione)
    double target_vz_limit = 350.0;      // Salita rapida (superiore ai caccia standard)
    double target_vy_limit = 210.0;      // Velocità laterale (virata)
    
    // Ripartizione vettoriale dei G (Virata + Cabrata)
    // L'F-22 usa il vettore di spinta per tirare G sia in verticale che laterale.
    // Ipotizziamo una ripartizione: 7G per salire, 5.5G per virare (Totale vettoriale ~9G)
    double acc_z_climb = 8.0 * 9.81;     
    double acc_y_turn  = 7.5 * 9.81;     

    // LOGICA EVASIONE "BREAK TURN & CLIMB"
    if (posT[0] > soglia_x_manovra) {
        
        // 1. CABRATA (Asse Z) - Pull Up
        if (velT[2] < target_vz_limit) {
            velT[2] += acc_z_climb * dt;
            // Clamp Z
            if (velT[2] > target_vz_limit) velT[2] = target_vz_limit;
        }

        // 2. VIRATA DIFENSIVA (Asse Y) - Break Turn
        // Questo costringe il missile a calcolare l'anticipo su due piani (Orizz. e Vert.)
        if (velT[1] < target_vy_limit) {
            velT[1] += acc_y_turn * dt;
            // Clamp Y
            if (velT[1] > target_vy_limit) velT[1] = target_vy_limit;
        }

        // 3. CONSERVAZIONE ENERGIA (F-22 Thrust-to-Weight > 1)
        // Normalmente una virata fa perdere velocità frontale (X). 
        // L'F-22 ha motori mostruosi, quindi perde pochissimo, ma per realismo
        // riduciamo leggermente la V_x mentre manovra (Bleeding energy).
        if (velT[0] > 250.0) { // Non scende sotto i 250 m/s
             velT[0] -= 2.0 * dt; // Decelerazione leggera dovuta al drag indotto
        }
    }

    // --- SOMMA FINALE RK4 (Aggiornamento Cinematico) ---
    for(int i=0; i<3; i++) 
    {
        // Aggiorna posizione Missile (RK4)
        posM[i] += (dt / 6.0) * (k1_vel[i] + 2.0*k2_vel[i] + 2.0*k3_vel[i] + k4_vel[i]);
        
        // Aggiorna velocità Missile (RK4)
        velM[i] += (dt / 6.0) * (k1_acc[i] + 2.0*k2_acc[i] + 2.0*k3_acc[i] + k4_acc[i]);
        
        // Aggiorna Posizione Target (Eulero)
        // Usa la velocità velT aggiornata dalla logica di manovra qui sopra
        posT[i] += velT[i] * dt;
    }





    // --- 3. AGGIORNAMENTO CARBURANTE ---
    // Se c'è carburante, lo consumiamo in base a quanto abbiamo spinto (throttle_usato)
    if (fuel_attuale > 0.0) {
        // Consumo = Rateo_Max * Throttle * Tempo
        double fuel_bruciato = consumo_kg_s * throttle_usato * dt;
        
        *ptr_massa_fuel -= fuel_bruciato; // Scriviamo nella variabile puntata
        
        if (*ptr_massa_fuel < 0.0) *ptr_massa_fuel = 0.0;
    }

    // --- 4. CALCOLO DISTANZA FINALE (Semplice) ---
    double diff_fin[3], d_sq_fin = 0.0;
    for(int i=0; i<3; i++) {
        diff_fin[i] = posT[i] - posM[i];
        d_sq_fin += diff_fin[i]*diff_fin[i];
    }    
    return sqrt(d_sq_fin);
}