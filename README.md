# 📚 Physics Simulations – Projects by Matteo

This repository contains a collection of physics simulations and calculators developed in Python, designed for students, educators, and enthusiasts. Each script explores a specific topic in classical physics (kinematics, dynamics, gravity, friction, pendulums, multi-body systems), offering both interactive computational tools and animated visualizations with Pygame.

The aim of this project is to help learners engage with physics in a dynamic and visual way, allowing them to explore physical phenomena and observe their evolution in real-time.

---

## 💻 Requirements

* Python 3.8 or higher
* [Pygame](https://www.pygame.org/) for graphical simulations:

  ```bash
  pip install pygame
  ```

Optional but recommended:

* Python virtual environment (`venv`)
* Code editor such as VSCode or PyCharm for editing and debugging

---

## 🧩 Program Overview

### 1. `simulazioni_cli.py`

A text-based command-line tool for performing physics calculations based on classical mechanics formulas. Available modules include:

* **Projectile motion**: compute time of flight, horizontal range, maximum height, and final velocity.
* **Free fall from rest**: calculate fall time and final speed from a given height.
* **Free fall with initial velocity**: time of flight and final velocity considering initial vertical speed.
* **Height as a function of time**: compute height and velocity after a given time.
* **Pendulum**: oscillation period or centripetal acceleration.
* **Uniform circular motion**: angular velocity, frequency, and centripetal acceleration.
* **Inclined plane**: speed and distance for an object sliding without friction (or with estimated friction).

#### Run:

```bash
python simulazioni_cli.py
```

The user is guided via prompts for every required parameter. All results are printed in the terminal.

---

### 2. `proiettile_aria.py`

Animated simulation of projectile motion in two versions:

* ⚪ **Ideal trajectory** (no air resistance)
* 🔴 **Real trajectory** (with velocity-proportional drag force)

For each projectile, the following are displayed:

* Maximum height
* Horizontal range
* Time of flight
* Final speed
* Apex and impact point

In addition to the Pygame animation, results are also printed in the terminal.

#### Run:

```bash
python proiettile_aria.py
```

#### Controls:

* Press any key to close the simulation after the trajectory completes.

---

### 3. `inclinato_attrito.py`

Simulates an object sliding on an inclined plane, considering:

* Forces in action (gravity, static/dynamic friction, normal force)
* Resulting acceleration
* Motion updated frame-by-frame over time

Data is shown graphically (blocks, vectors, angle) and numerically in the console.

#### Calculations:

* If the object starts from rest, it checks whether it overcomes static friction
* Computes acceleration and motion if dynamic friction applies

#### Run:

```bash
python inclinato_attrito.py
```

#### Output:

* Pygame animation
* Console printout of main physical data (per frame)

---

### 4. `nbody_gravitazionale.py`

Interactive N-body gravity simulator. Each body has:

* Mass
* Position
* Initial velocity
* Custom color

Bodies interact according to Newton's law of universal gravitation:

```
F = G * (m1 * m2) / r^2
```

The simulation shows:

* Trajectory trails
* Force vectors
* Mass and velocity data for each body

#### Run:

```bash
python nbody_gravitazionale.py
```

#### Controls:

* **Left-click**: add a new body at the mouse position
* **ESC** or window close: end the simulation

---

## 🚀 Quick Setup

1. Clone the repository:

```bash
git clone https://github.com/your-username/FisicaSimulazioni.git
cd FisicaSimulazioni
```

2. (Optional) Create a virtual environment:

```bash
python -m venv venv
source venv/bin/activate  # Linux/macOS
venv\Scripts\activate     # Windows
```

3. Install dependencies:

```bash
pip install pygame
```

---

## 📊 Equations and Physics Background

Each simulation is based on core physics equations:

### Kinematics:

* $s = s_0 + v_0 t + \frac{1}{2} a t^2$
* $v = v_0 + a t$

### Projectile motion:

* $R = \frac{v_0^2 \sin(2\theta)}{g}$
* $H_{max} = \frac{v_0^2 \sin^2(\theta)}{2g}$

### Pendulum:

* $T = 2 \pi \sqrt{\frac{l}{g}}$

### Circular motion:

* $a_c = \omega^2 r$
* $\omega = \frac{2 \pi}{T}$

### Gravitational interaction:

* $F = G \cdot \frac{m_1 m_2}{r^2}$

These are implemented and commented directly within the code.

---

## 📄 License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---



**Developed with passion by Matteo** ✨


