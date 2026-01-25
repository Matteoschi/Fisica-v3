#include <stdio.h>
#include <math.h>

// --- COSTANTI FISICHE ---
#define G_ACCEL 9.81
#define COSTANTE_NAVIGAZIONE 4.0      
#define LIMIT_G_LOAD 35.0             
#define RAGGIO_PROSSIMITA 10.0        
#define EPSILON 1e-6
#define RHO_0 1.225         // Densità dell'aria a livello del mare (kg/m^3)
#define SCALE_HEIGHT 8500.0 // L'aria diminuisce del 63% ogni 8500m 
#define AREA_MISSILE 0.0314 // Area sezione frontale in m^2 (es. raggio 10cm -> pi*0.1^2)  
#define GAMMA 1.4            // Indice adiabatico aria
#define R_GAS 287.05         // Costante specifica gas aria J/(kg·K)     
#define MAX_CL 1.2           // Coefficiente di portanza massimo stimato (limite stallo alette)
#define INDUCED_DRAG_K 0.05   // Fattore resistenza indotta (efficienza alette)
#define PRESSIONE_SEA_LEVEL 101325.0 // Pascal (Pa) standard a livello del mare
#define AREA_UGELLO 0.0045 // Area ugello motore in m^2 (es. raggio 3.8cm -> pi*0.038^2)
 

double max_acc_strutturale = LIMIT_G_LOAD * G_ACCEL;

// --- Funzioni Matematiche Ausiliarie ---

double modulo_vettore(double vettore[3]) { // pitagora in 3D
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


void calcola_fisica_step(
    double posM[3], double velM[3], double massa_attuale,
    double posT[3], double velT[3], 
    double spinta_attuale, 
    double drag_coeff,
    double acc_out[3],      
    double dati_debug[4]    
) {
    // 1. Calcolo Distanza e Vettori base
    double differenza_vettoreiale[3], dist_sq = 0.0;
    for(int i=0; i<3; i++) {
        differenza_vettoreiale[i] = posT[i] - posM[i];
        dist_sq += differenza_vettoreiale[i]*differenza_vettoreiale[i];
    }
    double distanza = sqrt(dist_sq);
    
    // Protezione divisione per zero
    if (distanza < EPSILON) distanza = EPSILON;

    // =============================================================================
    // 3. Guida Proporzionale (Pro-Nav) Acmd=K*Vc*(ω_LOS x u_LOS)
    // =============================================================================

    double vettore_LOS_normalizzato[3]; // vettore direzione verso il target l=1
    double velocità_relativa[3]; // velocità relativa
    
    for(int i=0; i<3; i++) {
        vettore_LOS_normalizzato[i] = differenza_vettoreiale[i] / distanza; // normalizzazione
    }

    for(int i=0; i<3; i++) {
        velocità_relativa[i] = velT[i] - velM[i]; //differenza velocità
    }
    
    double closing_speed = -prodotto_scalare(velocità_relativa, vettore_LOS_normalizzato); //prodotto scalare Vc=d/dt|r|
    double accelerazione_comandata_legge_guida[3] = {0,0,0};

    if (closing_speed > 0) { 
        double cross_r_v[3], velocità_angolare_los[3], cross_omega_los[3];
        prodotto_vettroiale(differenza_vettoreiale, velocità_relativa, cross_r_v); // calcolo r x V
        prodotto_vettore_scalare(cross_r_v, 1.0 / dist_sq, velocità_angolare_los); // calcolo velocità angolare LOS ω_LOS = (r x V) / |r|^2
        prodotto_vettroiale(velocità_angolare_los, vettore_LOS_normalizzato, cross_omega_los);
        
        double gain = COSTANTE_NAVIGAZIONE * closing_speed;
        prodotto_vettore_scalare(cross_omega_los, gain, accelerazione_comandata_legge_guida); // risultato finale Acmd
    }

    // Limite G-Force strutturale 
    double guide_force = modulo_vettore(accelerazione_comandata_legge_guida);
    if (guide_force > max_acc_strutturale) {
        prodotto_vettore_scalare(accelerazione_comandata_legge_guida, max_acc_strutturale / guide_force, accelerazione_comandata_legge_guida); //Saturazione vettoriale = acc * (max_acc / acc)
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
            versore_velocità[i] = vettore_LOS_normalizzato[i];
        }
    }

    // ==========================================
    // A. CALCOLO PARAMETRI ATMOSFERICI
    // ==========================================
    double altitudine = posM[2]; 
    if (altitudine < 0) altitudine = 0; 

    // Formula Barometrica (approssimata): rho(h) = rho0 * e^(-h/H)
    double densita_aria = RHO_0 * exp(-altitudine / SCALE_HEIGHT);

    // Gradiente Termico Verticale: T(h) = T0 - L*h
    double temperatura_kelvin = 288.15 - 0.0065 * altitudine; 
    if (temperatura_kelvin < 50.0) temperatura_kelvin = 50.0;

    // Velocità del suono (Gas ideale): a = sqrt(gamma * R * T)
    double speed_of_sound = sqrt(GAMMA * R_GAS * temperatura_kelvin);
    
    // Numero di Mach: M = v / a
    double mach_number = modulo_velocità_missile / speed_of_sound;

    // Pressione Dinamica (Bernoulli): q = 1/2 * rho * v^2
    double pressione_dinamica = 0.5 * densita_aria * modulo_velocità_missile * modulo_velocità_missile;


    //F. EFFICIENZA MOTORE (Variazione Isp con Quota)

    //  Calcolo Pressione Atmosferica Locale
    // Legge dei Gas Ideali: P = rho * R * T
    double pressione_locale = densita_aria * R_GAS * temperatura_kelvin;
    double spinta_motore = spinta_attuale;
    // se il motore è acceso 
    if (spinta_attuale > EPSILON) {
        double delta_pressione = PRESSIONE_SEA_LEVEL - pressione_locale;
        double spinta_bonus = delta_pressione * AREA_UGELLO;
        
        spinta_motore = spinta_attuale + spinta_bonus;
    }

    // ==========================================
    // C. COEFFICIENTE DRAG BASE (Cd0)
    // ==========================================
    // Resistenza d'onda 
    double mach_multiplier = 1.0;
    if (mach_number >= 0.8 && mach_number < 1.2) {
        // Approssimazione picco resistenza transonica (Prandtl-Glauert singularity)
        mach_multiplier = 1.0 + (mach_number - 0.8) * 3.75; 
    } 
    double cd_base = drag_coeff * mach_multiplier; // Questo è il Cd0

    // ==========================================
    // D. LIMITATORE G-FORCE (Aerodinamica vs Struttura)
    // ==========================================
    
    // Equazione della Portanza (allo stallo): L_max = q * S * CL_max
    double max_forza_aero = pressione_dinamica * AREA_MISSILE * MAX_CL;
    
    // Seconda Legge di Newton: a_max = F_max / m
    double max_acc_aerodinamica = max_forza_aero / massa_attuale;

    // Il limite è il minimo tra la resistenza dei materiali e la portanza disponibile
    double limite_reale = max_acc_strutturale;
    if (max_acc_aerodinamica < max_acc_strutturale) {
        limite_reale = max_acc_aerodinamica;
    }
    
    if (guide_force > limite_reale) {
        if (limite_reale < EPSILON) { // Nessuna capacità di virata
            accelerazione_comandata_legge_guida[0] = 0;
            accelerazione_comandata_legge_guida[1] = 0;
            accelerazione_comandata_legge_guida[2] = 0;
        } else {
            double ratio = limite_reale / guide_force;
            prodotto_vettore_scalare(accelerazione_comandata_legge_guida, ratio, accelerazione_comandata_legge_guida); //Saturazione vettoriale = acc * (max_acc / acc)
        }
    }

    // ==========================================
    // E. CALCOLO DRAG TOTALE (Polare Aerodinamica)
    // ==========================================
    
    double cl_attuale = 0.0;
    
    if (pressione_dinamica > EPSILON) {
        // a_req è l'accelerazione effettiva dopo il taglio del limitatore
        double acc_virata_effettiva = modulo_vettore(accelerazione_comandata_legge_guida);
        
        // Newton: F_lift = m * a
        double forza_lift_richiesta = massa_attuale * acc_virata_effettiva;
        
        // Equazione Portanza Inversa: CL = L / (q * S)
        cl_attuale = forza_lift_richiesta / (pressione_dinamica * AREA_MISSILE);
    }

    // Polare Aerodinamica (Parabolica): Cd = Cd0 + k * CL^2
    double cd_indotto = INDUCED_DRAG_K * (cl_attuale * cl_attuale);
    double cd_totale = cd_base + cd_indotto;

    // Equazione della Resistenza (Drag): D = q * S * Cd
    double drag_force_mag = pressione_dinamica * AREA_MISSILE * cd_totale;
    
    // B. Calcolo Accelerazione Totale 
    for(int i=0; i<3; i++) {
        double f_thrust = versore_velocità[i] * spinta_motore; // Forza di spinta = Spinta * versore_velocità
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
        dati_debug[4]= mach_number;        // Numero di Mach Attuale
        dati_debug[5]= pressione_dinamica;          // Pressione Dinamica Attuale
        dati_debug[6]= pressione_locale;          // Pressione Atmosferica Attuale
        dati_debug[7]= spinta_motore;          // Spinta Motore Attuale
        dati_debug[8]=spinta_attuale;          // Spinta Motore Base
        dati_debug[9]= cd_totale;          // Coefficiente Drag Totale
        dati_debug[10]= cl_attuale;         // Coefficiente Lift Attuale

    }
}

// =============================================================================
// FUNZIONE PRINCIPALE (INTEGRATORE RK4)
// =============================================================================

double aggiorna_missile(
    double posM[3], double velM[3], 
    double posT[3], double velT[3], 
    double dt, 
    double tempo_corrente,      // Tempo dal lancio (sec)
    double massa_totale_launch, // Peso al decollo
    double massa_carburante,    // Peso del solo carburante
    double durata_motore,       // Quanto dura la fiamma (sec)
    double spinta_newton,       
    double drag_coeff,
    double dati_output[4]
) {
    // 1. Check rapido distanza iniziale
    double diff[3], d_sq = 0.0;
    for(int i=0; i<3; i++) {
        diff[i] = posT[i] - posM[i];
        d_sq += diff[i]*diff[i];
    }
    double distanza = sqrt(d_sq);
    if (distanza < RAGGIO_PROSSIMITA) return 0.0; 
    if (distanza < EPSILON) return distanza;

    // 2. Calcolo Massa e Spinta (Una volta sola per step temporale)
    double massa_attuale;
    double spinta_attuale;
    double ratio_consumo;

    if (tempo_corrente < durata_motore) {
        spinta_attuale = spinta_newton;
        ratio_consumo = (massa_carburante / durata_motore); 
        massa_attuale = massa_totale_launch - ratio_consumo * tempo_corrente;
    } else {
        spinta_attuale = 0.0;
        massa_attuale = massa_totale_launch - massa_carburante;
    }

    // =============================================================================
    // RUNGE-KUTTA 4° ORDINE (RK4)
    // =============================================================================
    
    double k1_acc[3], k2_acc[3], k3_acc[3], k4_acc[3];
    double k1_vel[3], k2_vel[3], k3_vel[3], k4_vel[3];
    
    double pos_temp[3], vel_temp[3];
    
    // --- STEP K1 (Stato Iniziale) ---
    for(int i=0; i<3; i++) k1_vel[i] = velM[i];
    
    // Nota: Passiamo dati_output QUI per salvare la telemetria dello stato reale corrente
    calcola_fisica_step(posM, velM, massa_attuale, posT, velT, spinta_attuale, drag_coeff, k1_acc, dati_output);

    // --- STEP K2 (Metà strada con pendenza K1) ---
    for(int i=0; i<3; i++) {
        pos_temp[i] = posM[i] + k1_vel[i] * 0.5 * dt;
        vel_temp[i] = velM[i] + k1_acc[i] * 0.5 * dt; 
    }
    for(int i=0; i<3; i++) k2_vel[i] = vel_temp[i];
    
    // NULL perché non ci interessa la telemetria dei passaggi intermedi
    calcola_fisica_step(pos_temp, vel_temp, massa_attuale, posT, velT, spinta_attuale, drag_coeff, k2_acc, NULL);

    // --- STEP K3 (Metà strada con pendenza K2) ---
    for(int i=0; i<3; i++) {
        pos_temp[i] = posM[i] + k2_vel[i] * 0.5 * dt;
        vel_temp[i] = velM[i] + k2_acc[i] * 0.5 * dt; 
    }
    for(int i=0; i<3; i++) k3_vel[i] = vel_temp[i];
    
    calcola_fisica_step(pos_temp, vel_temp, massa_attuale, posT, velT, spinta_attuale, drag_coeff, k3_acc, NULL);

    // --- STEP K4 (Fine strada con pendenza K3) ---
    for(int i=0; i<3; i++) {
        pos_temp[i] = posM[i] + k3_vel[i] * dt;     
        vel_temp[i] = velM[i] + k3_acc[i] * dt;
    }
    for(int i=0; i<3; i++) k4_vel[i] = vel_temp[i];

    calcola_fisica_step(pos_temp, vel_temp, massa_attuale, posT, velT, spinta_attuale, drag_coeff, k4_acc, NULL);

    // SOMMA FINALE 

    for(int i=0; i<3; i++) 
    {
        // Aggiorna posizione
        posM[i] += (dt / 6.0) * (k1_vel[i] + 2.0*k2_vel[i] + 2.0*k3_vel[i] + k4_vel[i]);
        
        // Aggiorna velocità
        velM[i] += (dt / 6.0) * (k1_acc[i] + 2.0*k2_acc[i] + 2.0*k3_acc[i] + k4_acc[i]);
        
        // Aggiorna Target assumendo moto rettilineo uniforme per questo dt
        posT[i] += velT[i] * dt;
    }

    return distanza;
}