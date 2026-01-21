# Drone Simulation con Sensori (Pygame) — README dettagliato

> Progetto: simulazione 2D di un "drone" con sensori radiali che rilevano ostacoli rettangolari con un semplice modello di quota (Z).

---

## Sommario
- [Overview](#overview)
- [Caratteristiche principali](#caratteristiche-principali)
- [Requisiti](#requisiti)
- [Installazione ed esecuzione](#installazione-ed-esecuzione)
- [Struttura del file e variabili globali](#struttura-del-file-e-variabili-globali)
- [Dettaglio delle classi e funzioni](#dettaglio-delle-classi-e-funzioni)
  - `GameObject` (drone)
  - `punto_collisione()`
- [Logica del loop principale](#logica-del-loop-principale)
- [Coordinate, assi e convenzioni](#coordinate-assi-e-convenzioni)
- [Sensori: algoritmi, parametri e ottimizzazioni](#sensori-algoritmi-parametri-e-ottimizzazioni)


---

## Overview
Questo script è una **simulazione didattica** che mostra un oggetto `drone` in un ambiente 2D gestito con **Pygame**. Il drone ha:
- una posizione `(x, y, z)`;
- velocità `Vx, Vy, Vz` (velocità orizzontali e variazione di quota);
- un insieme di **sensori radial**i (nell'implementazione attuale `n_sensori = 8`) che campionano la scena per verificare la presenza di ostacoli.

I sensori lavorano come raggi (ray-casting) che vengono campionati a step costante per determinare il primo punto di collisione. Ogni ostacolo è rappresentato come un rettangolo 2D con un parametro di "altezza": il drone colliderà con il rettangolo solo se la sua quota `z` è minore o uguale a quella dell'ostacolo.

Lo scopo del progetto è fornire una base per studiare:
- ray-casting semplice per sensori,
- avoidance behavior (reazioni automatiche quando un ostacolo è troppo vicino),
- interazione input-utente / simulazione, e
- possibilità di estendere con pathfinding, più droni, o fisica avanzata.


## Caratteristiche principali
- Simulazione 2D con quota semplificata (Z limitata tra 0 e 50).
- 8 sensori radiali con visualizzazione grafica (linee) e label di distanza.
- Colorazione dinamica delle linee dei sensori: verde lontano → rosso vicino.
- Rilevamento e reazione automatica quando la distanza è inferiore a soglie prefissate.
- Input da tastiera per muovere il drone e variare la quota.
- Ambientazione di ostacoli con "altezza" casuale per testare la logica di collisione.


## Requisiti
- Python 3.8+
- pygame (consigliata l'ultima versione stabile)

Installazione rapida (consigliata in virtualenv):

```bash
python -m venv venv
source venv/bin/activate    # Linux / macOS
venv\Scripts\activate     # Windows
pip install pygame
```


## Installazione ed esecuzione
1. Copia il file (ad esempio `drone_simulation.py`) nella cartella del progetto.
2. Avvia il programma:

```bash
python drone_simulation.py
```

Schermata Pygame si aprirà con dimensioni predefinite (`800x600`).


## Struttura del file e variabili globali
In cima al file trovi le costanti di configurazione:
- `SCREEN_WIDTH`, `SCREEN_HEIGHT`, `FPS` — dimensioni della finestra e frame rate.
- palette colori (tupla RGB) — usate per disegnare rettangoli e testo.
- `font = pygame.font.SysFont(None, 18)` — font per i label.
- `clock`, `screen` — oggetti Pygame principali.

Variabili globali importanti:
- `drone` — istanza di `GameObject` creata come `GameObject(20, 35, 0, 1, WHITE)`.
- `rectangles` — lista di tuple che descrivono gli ostacoli.


## Dettaglio delle classi e funzioni
### `class GameObject`
Attributi principali:
- `x, y, z` — coordinate (float)
- `massa` — attributo presente ma non usato attivamente nella fisica.
- `colore` — colore del drone
- `Vx, Vy, Vz` — componenti di velocità
- `range_sensore` — portata massima dei sensori (default: 85)
- `n_sensori` — numero di sensori (default: 8)

Metodi:
- `draw(self, rectangles)` — disegna il drone, i sensori e tutte le label informative.
  - Viene mostrata la stringa `X: Y: Z:` in alto a sinistra.
  - Per ogni sensore (indice `i` da `0` a `n_sensori-1`) calcola l'angolo in radianti: `angolo = 2 * math.pi * i / self.n_sensori`.
  - Per ogni rettangolo richiama `punto_collisione()` per ottenere il punto di collisione più vicino lungo quel raggio.
  - Se la distanza misurata è inferiore a `range_sensore` allora la linea viene colorata con un gradiente rosso→verde basato sulla proporzione `distanza_min / range_sensore`.
  - Alcuni sensori hanno comportamento speciale: (i==0, i==4, i==2, i==6, i==1, i==3, i==5, i==7). Questi indici generano label testuali sullo schermo (es. "Ostacolo davanti", "Ostacolo dietro", ecc.) e, se la distanza è < 30 o < 50 a seconda dei casi, modificano direttamente `self.x` e/o `self.y` per eseguire un semplice avoidance (scatto immediato di qualche pixel).

- `move(self)` — applica le velocità a posizione e limita `x`,`y` ai confini dello schermo e `z` nell'intervallo `[0,50]`.

**Nota sul comportamento di avoidance:** il codice attuale sposta il drone forzatamente `self.x -= 4.5` o simili quando la distanza è sotto certe soglie. Questa è una strategia semplicissima (e non molto naturale): è efficace per demo ma va raffinata per simulazioni realistiche.


### `punto_collisione(drone, rect, angolo)`
Scopo: tornare la coordinata `(x,y)` del primo punto lungo il raggio che interseca il rettangolo, oppure il punto finale del raggio se nessuna collisione.

Implementazione attuale (sintesi):
1. Si determina la fine massima del raggio `fine_x, fine_y` calcolata con `drone.range_sensore`.
2. Si itera `t` da `0` a `drone.range_sensore` con passo `2` (px) e si verifica se il punto `(px, py)` è dentro il rettangolo 2D definito da `rect[0]..rect[3]`.
3. Se è stato trovato un punto dentro il rettangolo, si imposta `distanza_min = t` e si esce dal loop.
4. Se la quota del drone `drone.z` è minore o uguale al campo `rect[5]` (l'altezza dell'ostacolo), allora viene restituito il punto di collisione; altrimenti si restituisce il `fine_x, fine_y` (nessuna collisione a causa della quota).

**Parametri impliciti del `rect`**: il rettangolo è una tupla `(x, y, w, h, colore, height)`.

**Limitazioni e note:**
- L'approccio di campionamento è semplice ma non ottimale: richiede `O(range/step)` operazioni per ogni sensore e per ogni rettangolo.
- Il passo `2` px è un compromesso tra precisione e velocità. Diminuiscilo per maggiore accuratezza (ma CPU cost), aumentalo per performance migliori.


## Logica del loop principale
1. `screen.fill(BLACK)` — pulizia frame.
2. `clock.tick(FPS)` — throttle del frame rate.
3. Disegno di ogni rettangolo di `rectangles` con `pygame.draw.rect()` e scrittura al centro del rettangolo di un label `H{height}`.
4. Gestione eventi Pygame:
   - `QUIT` -> `running = False`.
   - `KEYDOWN` / `KEYUP` per `W/A/S/D` (modificano `Vx`, `Vy`) e frecce su/giù (modificano `Vz`).
   - Nota: i tasti `W`/`S` modificano `Vy` con la logica "invertita" (perché Y cresce verso il basso nel sistema di coordinate di Pygame).
5. `drone.move()` e `drone.draw(rectangles)`.
6. `pygame.display.flip()` per aggiornare lo schermo.


## Coordinate, assi e convenzioni
- Origine `(0,0)` è in alto a sinistra.
- `x` aumenta verso destra, `y` aumenta verso il basso.
- `z` è la quota del drone: 0..50 (limite imposto nel metodo `move`).
- Quando `drone.z <= rect[5]` si considera possibile collisione (l'ostacolo è ad altezza >= quota del drone).


## Sensori: algoritmi, parametri e ottimizzazioni
**Algoritmo attuale**: per ogni sensore si campionano punti lungo il raggio (passo `2px`) e per ciascun rettangolo si verifica se il punto è contenuto nel rettangolo. Si salva il primo punto incontrato — risultato: punto di collisione più vicino.

**Complessità**: `O(n_sensori * (range/step) * n_rettangoli)` per frame.

**Problemi e ottimizzazioni consigliate**:
- **Line-rect intersection**: sostituire il campionamento con un calcolo geometrico di intersezione tra segmento e rettangolo (più preciso e notevolmente più veloce).
- **Broad-phase collision**: usare bounding boxes o un quadtree per filtrare i rettangoli lontani prima di testare l'intersezione.
- **Precalcolare trigonometria**: sin/cos degli angoli dei sensori possono essere calcolati una volta per tutti (non ad ogni frame).
- **Ridurre `n_sensori` dinamicamente**: abbassare il numero di sensori se il sistema è sotto carico.
- **Usare step più grande** se serve performance (es. 4-6 px) ma attenzione alla precisione.


## Lista ostacoli (`rectangles`) — significato dei campi
Ogni elemento è una tupla con 6 campi:

```py
(x, y, w, h, color, height)
```

- `x, y` — posizione del rettangolo (angolo in alto a sinistra)
- `w, h` — larghezza e altezza (pixel)
- `color` — colore usato per disegnare
- `height` — valore intero casuale `random.randint(20,75)` che rappresenta l'altezza reale dell'ostacolo (usato per decidere se il drone può collidere considerando la sua `z`).





