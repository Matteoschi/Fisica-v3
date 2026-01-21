# 🚀 Simulazione Missilistica 3-DOF  
**Dinamica a Massa Variabile & Guida Proporzionale (PN)**

![Status](https://img.shields.io/badge/Status-Stable-green) ![Physics](https://img.shields.io/badge/Physics-Newtonian-blue)

Questo progetto implementa una **simulazione numerica ad alta fedeltà** di un’intercettazione aria-aria (*WVR – Within Visual Range*).  
Il sistema modella la fisica di un missile a corto raggio (classe **AIM-9 / IRIS-T**) combinando:

- 🧠 **Motore fisico ad alte prestazioni in C**
- 🐍 **Controller e visualizzazione in Python**

L’obiettivo è risolvere il **Problema di Intercettazione** in uno spazio tridimensionale, rispettando vincoli **energetici, aerodinamici e dinamici reali**.

---

## 🧮 Modello Fisico-Matematico

Il simulatore non è un motore cinematico “da videogioco”, ma un **solutore numerico di ODE**.  
Il missile è modellato come **punto materiale a 3 Gradi di Libertà (3-DOF)** soggetto a forze variabili nel tempo.

---

## ⚖️ Equazione del Moto

Il moto è governato dalla seconda legge di Newton per sistemi a **massa variabile**:

$$
\vec{a}(t) = \frac{d\vec{v}}{dt} = \frac{\sum \vec{F}_{esterne}}{m(t)}
$$

con:

$$
\sum \vec{F} = \vec{F}_{spinta}(t) + \vec{F}_{drag}(\vec{v}) + \vec{F}_{gravità} + \vec{F}_{guida}(\vec{r}, \vec{v})
$$

---

## 🔥 Massa Variabile – Equazione del Razzo

Il missile consuma propellente solido, riducendo la propria massa inerziale nel tempo (*burnout effect*).

### Fasi di volo
- **Boost phase** ($t < t_{burn}$) → motore acceso  
- **Coast phase** ($t \ge t_{burn}$) → volo balistico

$$
m(t) = 
\begin{cases} 
m_{launch} - \dot{m} \cdot t & \text{se } t < t_{burn} \\
m_{dry} & \text{se } t \ge t_{burn}
\end{cases}
$$

con:

$$
\dot{m} = \frac{m_{propellente}}{t_{burn}}
$$

---

## 🌪️ Aerodinamica – Drag Supersonico

La resistenza aerodinamica è modellata con la legge quadratica:

$$
\vec{F}_{drag} = - \left( \frac{1}{2} \rho A C_d \right) \cdot ||\vec{v}||^2 \cdot \hat{v}
$$

Nel codice il termine $\frac{1}{2}\rho A C_d$ è condensato nel parametro `DRAG_COEFF`.

Dopo lo spegnimento del motore, **drag + gravità** sono le uniche forze dissipative.

---

## 🎯 Legge di Guida – Proportional Navigation (PN)

Il missile utilizza **Navigazione Proporzionale Vettoriale**, standard industriale per missili IR.

> *Se la rotazione della Linea di Vista (LOS) è nulla, la collisione è certa.*

$$
\vec{a}_{cmd} = N \cdot V_c \cdot (\vec{\Omega} \times \hat{r}_{LOS})
$$

dove:

- $N$ → costante di navigazione (3–5)
- $V_c$ → closing speed
- $\vec{\Omega}$ → velocità angolare della LOS

$$
\vec{\Omega} = \frac{\vec{r}_{rel} \times \vec{v}_{rel}}{||\vec{r}_{rel}||^2}
$$

Questa legge realizza **lead-pursuit**, più efficace del *pure pursuit* contro bersagli manovranti.