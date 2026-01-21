import pygame  
import math  


class Car:  
    def __init__(self, x=20, y=400):  
        # ================================  
        # Parametri fisici Mazda MX-5 ND 2.0 184 CV  
        # ================================  
        self.m = 1180          # massa [kg]  
        self.g = 9.81  
        self.L = 2.31            # passo [m]  
        self.r_ruota = 0.311     # raggio ruota [m]  
        self.i_diff = 2.866      # rapporto differenziale  
        self.h_baricentro = 0.45
        self.L_f = self.L / 2  # distanza CG-asse anteriore
        self.L_r = self.L / 2  # distanza CG-asse posteriore

  
        # Rapporti cambio manuale 6M  
        self.rapporti = [5.087, 2.991, 2.035, 1.594, 1.286, 1.000]  
        self.gear = 1  
  
        # Aerodinamica  
        self.rho = 1.225         # densità aria [kg/m3]  
        self.Cd = 0.34           # coeff. resistenza aerodinamica  
        self.Cl = -0.1          # coeff. portanza   
        self.A = 1.82            # area frontale stimata [m2]  
        self.Crr = 0.015         # coeff. rotolamento  
        self.mu = 0.9           # coeff. attrito pneumatici  
  
        # Stato dinamico  
        self.vx = 0.0  
        self.vy = 0.0  
        self.v = 0.0  
        self.delta = 0.0  
        self.x = x  
        self.y = y  
        self.yaw = 0.0  
        self.space = 0.0
        self.space_1 = 0.0
  
        # Sistemi elettronici  
        self.traction_control = True  
  
        # Per pygame  
        self.surface = pygame.Surface((60, 30))  
        self.surface.fill((200, 0, 0))  
  
    # ================================  
    # Curva di coppia motore  
    # ================================  
    def coppia(self, n_rpm):
        if n_rpm < 1500: return 120
        elif n_rpm < 2500: return 150
        elif n_rpm < 3000: return 170
        elif n_rpm < 4000: return 205
        elif n_rpm < 5000: return 200
        elif n_rpm < 6000: return 190
        elif n_rpm < 6500: return 180
        elif n_rpm < 7000: return 160
        else: return 0
  
    # ================================  
    # Update dinamico  
    # ================================  
    def update(self, dt, gas, freno, sterzo):  
        # --- Sterzo ---  
        self.delta += sterzo * dt  
  
        # --- Regime motore ---  
        n_rpm = (self.v / (2 * math.pi * self.r_ruota)) * self.rapporti[self.gear - 1] * self.i_diff * 60  
        n_rpm = max(n_rpm, 1000)  # minimo motore  
  
        # --- Forza motrice ---  
        T = self.coppia(n_rpm) * gas 
        P_motore = (T * n_rpm) / 9549  # Potenza in kW 
        F_mot = (T * self.rapporti[self.gear - 1] * self.i_diff) / self.r_ruota  
  
        # --- Resistenze ---  
        F_drag = 0.5 * self.rho * self.Cd * self.A * self.v**2  
        F_down = 0.5 * self.rho * abs(self.Cl) * self.A * self.v**2  
        F_roll = self.Crr * (self.m * self.g + F_down)  
        F_brake = freno *10080    # forza frenante massima [N]  
  
        # --- Dinamica longitudinale ---  
        Fx = F_mot - F_drag - F_roll - F_brake  
        if self.vx <= 0 and Fx < 0:
            Fx = 0
        ax = Fx / self.m  
        self.vx += ax * dt  
        self.space += self.vx * dt
        self.space_1 += self.vx * dt
        self.vx = max(self.vx, 0)  
  
        # --- Dinamica laterale ---  
        Fz = (self.m * self.g + F_down) / 4  
        F_max = self.mu * Fz * 4  
  
        # Forza laterale teorica (modello semplificato)  
        Fy = self.m * self.vx**2 * math.tan(self.delta) / self.L    
  
        # Controllo limite aderenza  
        F_tot = math.sqrt(Fx**2 + Fy**2)  
        if self.traction_control and F_tot > F_max:  
            excess = F_tot - F_max  
            Fx -= excess * 0.5  
            self.vx *= 0.99  
        elif not self.traction_control and F_tot > F_max:  
            slip_ratio = (F_tot - F_max) / F_max  
            self.vy += slip_ratio * 0.05 * self.vx  
            self.vx *= 0.995  
  
        # --- Aggiornamento dinamica globale ---  
        self.v = math.sqrt(self.vx**2 + self.vy**2)  
  
        # Cinematica  
        self.x += (self.vx * math.cos(self.yaw) - self.vy * math.sin(self.yaw)) * dt  
        self.y += (self.vx * math.sin(self.yaw) + self.vy * math.cos(self.yaw)) * dt  
        self.yaw += (self.vx / self.L) * math.tan(self.delta) * dt  
        
        self.n_rpm = n_rpm
        self.T = T
        self.F_mot = F_mot
        self.F_drag=F_drag
        self.F_down=F_down
        self.Fx=Fx
        self.ax=ax
        self.F_roll=F_roll
        self.P_motore=P_motore


    # ================================  
    # Funzione disegno Pygame  
    # ================================  
    def draw(self, screen):
        color = (255, 0, 0)  
        radius = 4           
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), radius)

  
# ==============================  
# MAIN LOOP  
# ==============================  
pygame.init()  
screen = pygame.display.set_mode((1500, 800))  
pygame.display.set_caption("Simulazione Mazda MX-5 con Classe")  
clock = pygame.time.Clock()  
car = Car()  
  
font = pygame.font.SysFont(None, 30)  
running = True  
  
while running:  
    dt = clock.tick(60) / 1000.0  
  
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # Cambio marcia solo a pressione singola
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_q:
                car.gear += 1
                if car.gear > 6:
                    car.gear = 6

    # Input continuo
    keys = pygame.key.get_pressed()
    gas = 1.0 if keys[pygame.K_w] else 0.0
    freno = 1.0 if keys[pygame.K_s] else 0.0

    sterzo = 0.0
    if keys[pygame.K_a]:
        sterzo = -math.radians(40) 
    elif keys[pygame.K_d]:
        sterzo = math.radians(40) 
    else:
        car.delta *= 0.9  
    
    
    # Limite massimo angolo sterzo
    car.delta = max(-math.radians(33), min(math.radians(33), car.delta))

    # Aggiornamento auto  
    car.update(dt, gas, freno, sterzo)  
  
    # Render  
    screen.fill((30,30,30))  
    car.draw(screen)  
  
    vel = font.render(f"Velocità: {car.v * 3.6:.1f} km/h", True, (255, 255, 255))  
    screen.blit(vel, (20, 20))

    if car.n_rpm >= 6500 and car.gear < 6:
        car.gear += 1
        car.vx *= 0.98   # perdita di slancio cambio
        print("Cambiata su",car.gear)
    elif car.n_rpm <= 1500 and car.gear > 1:
        car.gear -= 1
        print("Cambiata giu",car.gear)
        car.vx *= 0.98   # perdita di slancio cambio

    rpm = font.render(f"RPM motore: {car.n_rpm:.0f}", True, (255, 255, 255))  
    screen.blit(rpm, (20, 50))

    coppia = font.render(f"Coppia: {car.T:.1f} Nm", True, (255, 255, 255))  
    screen.blit(coppia, (20, 80))

    forza = font.render(f"Forza motrice: {car.F_mot:.1f} N", True, (255, 255, 255))  
    screen.blit(forza, (20, 110))


    drag = font.render(f"forza drag: {car.F_drag:.1f} N", True, (255, 255, 255))  
    screen.blit(drag, (20, 140))

    down = font.render(f"portanza: {car.F_down:.0f} N", True, (255, 255, 255))  
    screen.blit(down, (20, 170))

    Roll = font.render(f"f_roll: {car.F_roll:.1f} N", True, (255, 255, 255))  
    screen.blit(Roll, (20, 200))

    fxs = font.render(f"fx: {car.Fx:.1f} N", True, (255, 255, 255))  
    screen.blit(fxs, (20, 230))

    axs = font.render(f"ax: {car.ax:.1f} m/s**2", True, (255, 255, 255))  
    screen.blit(axs, (20, 260))

    gear = font.render(f"marcia {car.gear:.1f}", True, (255, 255, 255))  
    screen.blit(gear, (20, 290))

    spazio = font.render(f"spazio {car.space:.1f} m", True, (255, 255, 255))  
    screen.blit(spazio, (20, 320))


    P = font.render(f"P motore {car.P_motore:.1f} KW", True, (255, 255, 255))  
    screen.blit(P, (20, 350))

    if not hasattr(car, "timer_0_100_running"):
        car.timer_0_100_running = False
        car.timer_0_100 = 0.0
        car.max_timer_0_100 = 0.0
        eseguito= False

    # Aggiornamento del timer
    if car.v > 0 and car.v < 100 / 3.6:  # velocità tra 0 e 100 km/h
        if not car.timer_0_100_running:
            car.timer_0_100_running = True
            car.timer_0_100 = 0.0  # reset al momento della partenza
            car.space_1 = 0.0
        car.timer_0_100 += dt
    elif car.timer_0_100_running:  # appena supera 100 km/h
        car.timer_0_100_running = False
        car.max_timer_0_100 = car.timer_0_100
        if not eseguito:
            print(f"spazio 0-100 km/h in {car.space_1:.2f} m")
            print(f"0-100 km/h in accelerazione :  {car.max_timer_0_100:.2f} s")
            car.space_1 = 0.0
            eseguito= True
        else:
            print(print(f"0-100 km/h in frenata :  {car.max_timer_0_100:.2f} s"))
            print(f"spazio 100-0 km/h in frenata : {car.space_1:.2f} m")
            car.space_1 = 0.0
            eseguito = False
  
    pygame.display.flip()  
  
pygame.quit()  
