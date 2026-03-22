# 🤖 ECNG 4504 – Autonomous Maze-Solving Robot
**The American University in Cairo | Spring 2026**
Department of Electronics & Communication Engineering

---

## 📌 Project Overview

An autonomous robotic vehicle that explores and solves a line-based maze with minimal user intervention. The robot uses a 5-channel IR sensor array and a differential-drive chassis to navigate a black-on-white track, detect intersections, and compute the optimal path using real-time path simplification.

The system operates in two phases:
- **Training Phase** — The robot explores the maze, records every turn, and simplifies the path using the *xBy* algorithm.
- **Solving Phase** — The robot replays the optimized path to reach the target as efficiently as possible.

Serial commands (UART) are used to control the robot in Phase 0. BLE wireless communication is added in Phase 1.

---

## 🗂️ Repository Structure

```
embedded/
│
├── docs/
│   ├── diagram.pdf
│   ├── diagram_kicad_final.pdf
│   └── (project documentation, schematics, reports)
│
├── firmware/
│   ├── maze_solver.cpp
│   ├── final_code.cpp
│   └── (embedded C/C++ source code for microcontroller/logic)
│
├── hardware/
│   ├── schematics.pdf
│   └── (hardware design files, PCB layouts, schematics)
│
├── python_app/
│   ├── main.py
│   ├── requirements.txt
│   └── (Python scripts for interfacing, testing, or data processing)
│
├── .gitignore
├── README.md
```

---

## ⚙️ Hardware

| Component | Details |
|---|---|
| Microcontroller | Arduino Mega 2560 R3 |
| Motor Driver | L298N Dual H-Bridge |
| Motors | 2× DC Gear Motors (Left & Right) |
| Sensor Array | 5-channel IR line tracker (FL, L, C, R, FR) |
| Power | Battery pack (+BATT rail, 7.4–12 V) |
| Chassis | Differential-drive, max 40 × 40 cm |

### Pin Mapping

| Signal | Arduino Pin |
|---|---|
| Left Motor PWM | 9 |
| Left Motor IN1 / IN2 | 8 / 7 |
| Right Motor PWM | 3 |
| Right Motor IN3 / IN4 | 5 / 4 |
| IR Sensors (FL, L, C, R, FR) | 6, 2, 12, 11, 10 |

---

## 🧠 Firmware Architecture

### State Machine

```
MANUAL ──(T cmd)──► TRAINING ──(target reached)──► SOLVING ──(done)──► DONE
   ▲                    │                               │
   └──────(M cmd)───────┴───────────────────────────────┘
```

| State | Description |
|---|---|
| `STATE_MANUAL` | Awaiting serial command; manual WASD movement supported |
| `STATE_TRAINING` | Autonomous maze exploration; records & simplifies path |
| `STATE_SOLVING` | Replays optimized path from EEPROM |
| `STATE_DONE` | Motors stopped; mission complete |

### Intersection Detection

The robot classifies every intersection it encounters into one of seven types:

| Type | Description |
|---|---|
| `T_INTERSECTION` | Left + Right, no straight |
| `PLUS_INTERSECTION` | All four directions available |
| `LEFT_ELBOW` | Left turn only |
| `RIGHT_ELBOW` | Right turn only |
| `FORWARD_LEFT` | Straight or Left |
| `FORWARD_RIGHT` | Straight or Right |
| `DEAD_END` | No exit; triggers 180° turn |
| `TARGET` | End-zone black box; timing-based detection (>300 ms crossing) |

Detection uses a **two-step approach**: the outer sensors (FL / FR) trigger first, then after a short advance the inner sensors determine the exact intersection type.

### Path Simplification — *xBy* Algorithm

Every time a backtrack (`B`) is recorded, the algorithm checks the three-move window `a–B–c` and replaces it with a single equivalent move:

```
L–B–R → B    S–B–L → R    L–B–S → R
L–B–L → S    S–B–S → B    R–B–L → B
S–B–R → L    R–B–S → L
```

This runs recursively so chained dead-ends collapse into a single optimal move.

### EEPROM Persistence

Trained path and intersection logs are saved to the Mega's built-in 4 KB EEPROM after training completes. The magic byte `'M'` is written **last**, so a power loss during saving cannot corrupt stored data.

| Address | Content |
|---|---|
| 0 | Magic byte (`'M'` = valid data) |
| 1 | Optimized path length |
| 2–101 | Optimized path array (`L`, `R`, `S`) |
| 102–103 | Intersection log length (int) |
| 104+ | Raw intersection log |

---

## 🎮 Serial Command Reference

| Command | Action |
|---|---|
| `T` | Start Training phase |
| `S` | Start Solving phase (loads path from EEPROM) |
| `M` | Manual mode / Emergency stop |
| `E` | Extract & print EEPROM logs over Serial |
| `W` | (Manual) Drive forward |
| `A` | (Manual) Pivot left |
| `D` | (Manual) Pivot right |
| `X` | Stop motors (works in any state) |

> **Baud rate:** 9600

---

## 🔧 Tuning Parameters

Open `final_code.cpp` and adjust these constants before each demo run:

```cpp
// Motor Speeds (0–255 PWM)
const int SPEED_FORWARD   = 80;
const int SPEED_TURN      = 110;
const int SPEED_SEARCH    = 140;

// Motor Trim (correct for physical drift)
const int MOTOR_LEFT_TRIM  = 10;   // ⚠️ Tune this
const int MOTOR_RIGHT_TRIM = 0;    // ⚠️ Tune this

// Timing
const unsigned long TIME_NUDGE_SINGLE  = 70;     // ms per nudge pulse
const unsigned long MAX_TURN_TIME      = 7000;   // turn timeout (ms)
const unsigned long TIMEOUT_TRAINING   = 300000; // 5-min charter limit
```

---

## 🚀 Getting Started

1. **Open** `final_code.cpp` in the Arduino IDE (select Board: *Arduino Mega 2560*).
2. **Connect** the hardware according to the KiCAD schematic in `diagrams/`.
3. **Upload** the firmware via USB-B cable.
4. **Open** Serial Monitor at **9600 baud**.
5. Place the robot at the start of the maze and send `T` to begin training.
6. Once training completes and the robot is repositioned, send `S` to solve.

---

## 📋 Charter Compliance

| Requirement | Status |
|---|---|
| C/C++ only for MCU code | ✅ |
| Max size 40 × 40 cm | ✅ |
| Battery-powered | ✅ |
| No ready-made line-following robot | ✅ |
| 5-minute training timeout | ✅ (`TIMEOUT_TRAINING = 300000 ms`) |
| 360° rotation avoidance | ✅ (`TIME_SEARCH_TOTAL` hard cap) |
| Real-time status output | ✅ (Serial at 9600 baud) |
| Hardware schematic in KiCAD | ✅ |

---

## 👥 Team

| Name | ID | Email |
| Ahmed Aly | 900201363 | ahmed003@aucegypt.edu |
| Michael Aziz | 900211881 | michaelaziz@aucegypt.edu |
| Yasmin Hassane | 900211856 | yasminhesham@aucegypt.edu |
| Marian Ashraf | 900223018 | marianashraf@aucegypt.edu |



---

*ECNG 4504 – Embedded Systems for Wireless Communications | Spring 2026*
