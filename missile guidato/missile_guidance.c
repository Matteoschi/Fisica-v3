#include <stdio.h>
#include <math.h>

// --- COSTANTI FISICHE ---
#define G_ACCEL 9.81
#define COSTANTE_NAVIGAZIONE 4.0      
#define LIMIT_G_LOAD 35.0             
#define RAGGIO_PROSSIMITA 10.0        
#define EPSILON 1e-6                  

// --- HELPER MATEMATICI (Invariati) ---
double vec_mag(double v[3]) { return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]); }
double vec_dot(double a[3], double b[3]) { return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]; }
void vec_cross(double a[3], double b[3], double out[3]) {
    out[0] = a[1]*b[2] - a[2]*b[1];
    out[1] = a[2]*b[0] - a[0]*b[2];
    out[2] = a[0]*b[1] - a[1]*b[0];
}
void vec_scale(double v[3], double s, double out[3]) { for(int i=0; i<3; i++) out[i] = v[i] * s; }


double aggiorna_missile(
    double posM[3], double velM[3], 
    double posT[3], double velT[3], 
    double dt, 
    double tempo_corrente,      // Tempo dal lancio (sec)
    double massa_totale_launch, // Peso al decollo
    double massa_carburante,    // Peso del solo carburante
    double durata_motore,       // Quanto dura la fiamma (sec)
    double spinta_newton,       
    double drag_coeff 
) {
    // 1. Calcolo Distanza
    double r[3], dist_sq = 0.0;
    for(int i=0; i<3; i++) {
        r[i] = posT[i] - posM[i];
        dist_sq += r[i]*r[i];
    }
    double dist = sqrt(dist_sq);
    if (dist < RAGGIO_PROSSIMITA) return 0.0; 
    if (dist < EPSILON) return dist;

    // 2. Gestione Fisica Variabile (Motore e Massa)
    double massa_attuale;
    double spinta_attuale;

    if (tempo_corrente < durata_motore) {
        // Il motore è acceso
        spinta_attuale = spinta_newton;
        
        // La massa scende linearmente mentre bruciamo carburante
        // Rateo di consumo (kg/s) = massa_carburante / durata
        double consumo = (massa_carburante / durata_motore) * tempo_corrente;
        massa_attuale = massa_totale_launch - consumo;
    } else {
        // FASE COAST: Motore spento
        spinta_attuale = 0.0;
        // La massa è quella a vuoto (Totale - Carburante)
        massa_attuale = massa_totale_launch - massa_carburante;
    }

    // 3. Guida Proporzionale (PN)
    double unit_los[3], v_rel[3];
    for(int i=0; i<3; i++) unit_los[i] = r[i] / dist;
    for(int i=0; i<3; i++) v_rel[i] = velT[i] - velM[i];
    
    double closing_speed = -vec_dot(v_rel, unit_los);
    double acc_guide[3] = {0,0,0};

    if (closing_speed > 0) { 
        double cross_r_v[3], omega[3], cross_omega_los[3];
        vec_cross(r, v_rel, cross_r_v);
        vec_scale(cross_r_v, 1.0 / dist_sq, omega);
        vec_cross(omega, unit_los, cross_omega_los);
        
        // Accelerazione di guida proporzionale alla velocità di chiusura
        double gain = COSTANTE_NAVIGAZIONE * closing_speed;
        vec_scale(cross_omega_los, gain, acc_guide);
    }

    // Limite G-Force strutturale delle alette
    double guide_mag = vec_mag(acc_guide);
    double max_acc_strutturale = LIMIT_G_LOAD * G_ACCEL;
    if (guide_mag > max_acc_strutturale) {
        vec_scale(acc_guide, max_acc_strutturale / guide_mag, acc_guide);
    }

    // 4. Forze Fisiche
    double speed_m = vec_mag(velM);
    double unit_vel[3] = {1,0,0};
    if (speed_m > EPSILON) {
        for(int i=0; i<3; i++) unit_vel[i] = velM[i] / speed_m;
    } else {
        // Se fermo al lancio, punta verso il target
        for(int i=0; i<3; i++) unit_vel[i] = unit_los[i];
    }

    // A. Drag (Resistenza)
    double drag_force_mag = drag_coeff * speed_m * speed_m;
    
    // B. Calcolo Accelerazione Totale 
    double acc_total[3];
    for(int i=0; i<3; i++) {
        double f_thrust = unit_vel[i] * spinta_attuale;
        double f_drag   = -unit_vel[i] * drag_force_mag;
        double f_grav   = (i == 2) ? -G_ACCEL * massa_attuale : 0.0; // Solo asse Z
        
        // Newton: a = SommaForze / Massa_Variabile
        acc_total[i] = (f_thrust + f_drag + f_grav) / massa_attuale;
        
        // Aggiungi l'accelerazione comandata dalle alette (Guida)
        acc_total[i] += acc_guide[i];
    }

    // 5. Integrazione Eulero
    for(int i=0; i<3; i++) {
        velM[i] += acc_total[i] * dt;
        posM[i] += velM[i] * dt;
        velT[i] += 0.0 * dt; 
        posT[i] += velT[i] * dt;
    }

    return dist;
}