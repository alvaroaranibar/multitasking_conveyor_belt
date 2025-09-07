# Multitasking Dual Conveyor Belt and Rack System

This project implements a **dual conveyor belt system** controlled by an **ESP32** using **FreeRTOS**.  
It handles order sequences with **DC motors**, **servo actuators**, **proximity sensors**, and a **color sensor** for automatic sorting and rejection of defective products.  

---

## ⚙️ Hardware Components

- **ESP32** (microcontroller, FreeRTOS-based control)  
- **2 DC Motors** (driven via TB6612FNG motor driver)  
- **3 FS90R Continuous Rotation Servos (360°)** (used as linear actuators for object dispensing)  
- **1 MG995 Servo (180°)** (used for rejection mechanism)  
- **4 E18-D80NK Proximity Sensors**  
  - `PROX_COLOR` → Detects objects before color sensing  
  - `PROX_RECHAZO` → Detects rejected objects  
  - `PROX_CAIDA` → Detects falling objects  
  - `PROX_CAJA` → Detects presence of box on second conveyor  
- **TCS230 Color Sensor** (RGB color detection)  

---

## 🧩 Features

- **Dual Conveyor Belt Control**  
  - Conveyor 1: Dispenses requested colored cubes.  
  - Conveyor 2: Moves boxes after order completion.  

- **Order Handling with FreeRTOS Queues**  
  - User inputs order sequences via Serial Monitor.  
  - System processes orders asynchronously with FreeRTOS tasks.  

- **Color Verification & Rejection**  
  - TCS230 sensor detects cube color.  
  - Defective (white) or mismatched colors are rejected via MG995 servo actuator.  

- **Real-Time Monitoring**  
  - Serial output displays current orders, detected colors, rejected items, and system states.  

---

## 📂 Code Overview

Main modules and tasks:

- **`readSerialTask`**  
  Reads sequences of 5 numbers (`0–3`) from the serial terminal and stores them in a queue.  

- **`task_send1` & `task_receive1`**  
  - Sends and receives order sequences from FreeRTOS queues.  
  - Activates servos (R, G, B) to drop cubes according to the sequence.  

- **`task_color_y_control_faja1y2`**  
  - Controls conveyor 1 and conveyor 2.  
  - Handles color sensing, box presence, and order completion.  

- **`task_cuenta_caidos`**  
  Counts falling cubes detected by proximity sensor.  

- **`task_desactiva_rechazo`**  
  Resets the rejection servo after a rejected object passes.  

---

## 🚀 Libraries required:

- [`ESP32Servo`](https://github.com/madhephaestus/ESP32Servo)  
- [`Robojax_L298N_DC_motor`](https://github.com/robojax/Robojax-L298N-DC-motor)  
---

## 📊 Serial Input Format

To send an order sequence:  
- Enter exactly **5 numbers** (`0–3`) in the Serial Monitor, then press Enter.  

**Color codes:**  
- `1 = Red`  
- `2 = Green`  
- `3 = Blue`  
- `4 = White (defective)`  

Example:  
This will request a sequence of cubes: Red → Green → Blue → Green → Red.  

---

## 📡 System Workflow

1. User sends an order sequence through Serial.  
2. Conveyor 1 dispenses cubes using servo actuators.  
3. Color sensor verifies each cube:  
   - If correct → sent to box.  
   - If incorrect/defective → rejected.  
4. Conveyor 2 moves the box after sequence completion.  



