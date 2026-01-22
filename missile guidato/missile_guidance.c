#include <stdio.h>
#include <math.h>

// --- COSTANTI FISICHE ---
#define G_ACCEL 9.81
#define COSTANTE_NAVIGAZIONE 4.0      
#define LIMIT_G_LOAD 35.0             
#define RAGGIO_PROSSIMITA 10.0        
#define EPSILON 1e-6
#define RHO_0 1.225         // Densità dell'aria a livello del mare (kg/m^3)
#define SCALE_HEIGHT 8500.0 // L'aria diminuisce del 63% ogni 8500m (Costante di scala)
#define AREA_MISSILE 0.0314 // Area sezione frontale in m^2 (es. raggio 10cm -> pi*0.1^2)        

double max_acc_strutturale = LIMIT_G_LOAD * G_ACCEL;

// --- Funzioni ---

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
    // 1. Calcolo Distanza
    double differenza_vettoreiale[3], dist_sq = 0.0;
    for(int i=0; i<3; i++) {
        differenza_vettoreiale[i] = posT[i] - posM[i];
        dist_sq += differenza_vettoreiale[i]*differenza_vettoreiale[i];
    }
    double distanza = sqrt(dist_sq);
    if (distanza < RAGGIO_PROSSIMITA) return 0.0; 
    if (distanza < EPSILON) return distanza;

    // 2. Gestione Fisica Variabile (Motore e Massa)
    double massa_attuale;
    double spinta_attuale;
    double ratio_consumo;

    if (tempo_corrente < durata_motore) {
        // Il motore è acceso
        spinta_attuale = spinta_newton;
        // m(t) = m0 - (dm/dt)*t
        ratio_consumo = (massa_carburante / durata_motore); // kg/sec
        massa_attuale = massa_totale_launch - ratio_consumo * tempo_corrente;
    } else {
        // Motore spento
        spinta_attuale = 0.0;
        massa_attuale = massa_totale_launch - massa_carburante;
    }

    // =============================================================================
    // 3. Guida Proporzionale (Pro-Nav) Acmd=K*Vc*(ω_LOS x u_LOS)
    //==============================================================================

    double vettore_LOS_normalizzato[3]; // vettore direzione verso il target l=1
    double velocità_relativa[3]; // velocità relativa
    
    for(int i=0; i<3; i++)
    {
        vettore_LOS_normalizzato[i] = differenza_vettoreiale[i] / distanza; // normalizzazione
    }

    for(int i=0; i<3; i++)
    {
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
    double speed_m = modulo_vettore(velM);
    double versore_velocità[3] = {1,0,0};
    if (speed_m > EPSILON) {
        for(int i=0; i<3; i++) versore_velocità[i] = velM[i] / speed_m; //normalizzando la velocità.
    } else {
        // Se fermo al lancio, punta verso il target
        for(int i=0; i<3; i++) versore_velocità[i] = vettore_LOS_normalizzato[i];
    }

// A. Drag Atmosferico (Densità variabile)
    double altitudine = posM[2]; // Assumiamo che Z sia l'altitudine
    
    
    if (altitudine < 0){ 
        altitudine = 0; 
    } // Non andiamo sotto il livello del mare
    
    // Calcolo densità attuale: Rho = Rho0 * e^(-h/H)
    double densita_aria = RHO_0 * exp(-altitudine / SCALE_HEIGHT);

    // Formula Aerodinamica completa: Fd = 1/2 * rho * v^2 * Cd * A
    // drag_coeff deve essere Cd puro 
    double drag_force_mag = 0.5 * densita_aria * (speed_m * speed_m) * drag_coeff * AREA_MISSILE;
    
    // B. Calcolo Accelerazione Totale 
    double acc_total[3];
    for(int i=0; i<3; i++) {
        double f_thrust = versore_velocità[i] * spinta_attuale; // Forza di spinta = Spinta * versore_velocità
        double f_drag   = -versore_velocità[i] * drag_force_mag; // Forza di drag = -Versore_velocità * Drag_Mag (opppos)
        double f_grav   = (i == 2) ? -G_ACCEL * massa_attuale : 0.0; // Solo asse Z
        
        // Newton: a = SommaForze / Massa_Variabile
        acc_total[i] = (f_thrust + f_drag + f_grav) / massa_attuale;
        
        // Aggiungi l'accelerazione comandata dalle alette (Guida)
        acc_total[i] += accelerazione_comandata_legge_guida[i];
    }
    double g_load_val = modulo_vettore(accelerazione_comandata_legge_guida) / G_ACCEL; // G-Force

    // === QUI RIEMPIAMO L'ARRAY PER PYTHON ===
    // Assicuriamoci che non sia NULL per evitare crash
    if (dati_output != NULL) {
        dati_output[0] = drag_force_mag; // Indice 0: Drag
        dati_output[1] = g_load_val;     // Indice 1: G-Force
        dati_output[2] = densita_aria;   // Indice 2: Densità
        dati_output[3] = massa_attuale;  // Indice 3: Massa
    }
    // 5. Integrazione Eulero
    for(int i=0; i<3; i++) {
        velM[i] += acc_total[i] * dt;
        posM[i] += velM[i] * dt;
        velT[i] += 1.0 * dt; 
        posT[i] += velT[i] * dt;
    }

    return distanza;
}