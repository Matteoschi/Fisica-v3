import time
import math

gravity = float(input("Inserire gravità (m/s^2): "))
mezza_gravita = -0.5 * gravity  # Usato per la formula -1/2 * g * t^2

programma = input("Scegli: gravi - pendolo - moto circolare - piano inclinato: ").strip().lower()

if programma == "gravi":
    tipo = input("1 = Moto parabolico | 2 = Caduta da fermo | 3 = Caduta con velocità iniziale | 4 = Altezza in t: ")

    if tipo == "1":
        ha_velocita = input("Hai la velocità? (si/no): ").strip().lower()
        if ha_velocita == "no":
            t = float(input("Inserire tempo (s): "))
            spazio = float(input("Inserire spazio (m): "))
            velocita = spazio / t
        else:
            velocita = float(input("Inserire velocità (m/s): "))
        print(f"Velocità = {velocita:.2f} m/s ({velocita * 3.6:.2f} km/h)")

        altezza = float(input("Inserire altezza iniziale (m): "))
        angolo = float(input("Inserire angolo di lancio (°): "))
        ang_rad = math.radians(angolo)

        v0x = velocita * math.cos(ang_rad)
        v0y = velocita * math.sin(ang_rad)

        delta = v0y**2 - 4 * mezza_gravita * altezza
        if delta < 0:
            print("Errore: traiettoria non valida (delta < 0)")
        else:
            t_volo = (-v0y - math.sqrt(delta)) / (2 * mezza_gravita)
            print(f"Tempo di volo: {t_volo:.2f} s")
            distanza = v0x * t_volo
            print(f"Distanza: {distanza:.2f} m")

            t_mezzo = t_volo / 2
            h_max = mezza_gravita * t_mezzo**2 + v0y * t_mezzo + altezza
            print(f"Altezza massima: {h_max:.2f} m")

            v_finale = velocita + gravity * t_volo
            print(f"Velocità finale approssimata: {v_finale:.2f} m/s")

    elif tipo == "2":
        altezza = float(input("Inserire altezza (m): "))
        t = math.sqrt(altezza / (gravity / 2))
        v_finale = gravity * t
        print(f"Tempo di caduta: {t:.2f} s")
        print(f"Velocità finale: {v_finale:.2f} m/s")

    elif tipo == "3":
        altezza = float(input("Inserire altezza (m): "))
        ha_velocita = input("Hai la velocità iniziale? (si/no): ").strip().lower()
        if ha_velocita == "no":
            t = float(input("Tempo impiegato (s): "))
            spazio = float(input("Spazio percorso (m): "))
            velocita = spazio / t
        else:
            velocita = float(input("Inserire velocità (m/s): "))
        print(f"Velocità iniziale: {velocita:.2f} m/s")

        delta = velocita**2 + 2 * gravity * altezza
        t_volo = (-velocita + math.sqrt(delta)) / gravity
        v_finale = velocita + gravity * t_volo
        print(f"Tempo di volo: {t_volo:.2f} s")
        print(f"Velocità finale: {v_finale:.2f} m/s")

    elif tipo == "4":
        t = float(input("Inserire tempo (s): "))
        h = 0.5 * gravity * t**2
        v = gravity * t
        print(f"Altezza raggiunta: {h:.2f} m")
        print(f"Velocità finale: {v:.2f} m/s")

elif programma == "pendolo":
    scelta = input("accellerazione o periodo? ").strip().lower()
    lunghezza = float(input("Inserisci lunghezza del filo (m): "))

    if scelta == "accellerazione":
        acc = -gravity / lunghezza
        print(f"Accelerazione angolare: {acc:.2f} m/s^2")
    elif scelta == "periodo":
        periodo = 2 * math.pi * math.sqrt(lunghezza / gravity)
        print(f"Periodo: {periodo:.2f} s")

elif programma == "moto circolare":
    scelta = input("velocità angolare - frequenza - accelerazione centripeta: ").strip().lower()
    raggio = float(input("Inserisci raggio (m): "))
    periodo = float(input("Inserisci periodo (s): "))

    if scelta == "velocità angolare":
        omega = (2 * math.pi) / periodo
        print(f"Velocità angolare: {omega:.2f} rad/s")
    elif scelta == "frequenza":
        frequenza = 1 / periodo
        print(f"Frequenza: {frequenza:.2f} Hz")
    elif scelta == "accelerazione centripeta":
        omega = (2 * math.pi) / periodo
        a_centripeta = omega**2 * raggio
        print(f"Accelerazione centripeta: {a_centripeta:.2f} m/s^2")

elif programma == "piano inclinato":
    s0 = float(input("Altezza iniziale del piano (m): "))
    h1 = float(input("Altezza massima piano inclinato (m): "))
    h2 = float(input("Altezza minima piano inclinato (m): "))

    velocita = math.sqrt((10 * gravity * (h1 - h2)) / 7)
    print(f"Velocità finale stimata: {velocita:.2f} m/s (senza attrito)")

    distanza = math.sqrt((20 * (h1 - h2) * s0 / 7) - 0.0008)
    print(f"Distanza stimata (con attrito approssimato): {distanza:.2f} m")

time.sleep(10)
print("By Matteo")
