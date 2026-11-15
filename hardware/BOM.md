# Bill of Materials (BOM) — Hardware-in-the-Loop Steering Testbench

This document details all electromechanical, electrical, sensor, and hardware components required to reproduce the physical direct-drive steering testbench from scratch.

---

## 1. Actuation & Power Electronics

| Item | Component | Specification | Function / Description | Qty | Approx. Cost (USD) |
| :---: | :--- | :--- | :--- | :---: | :---: |
| 1.1 | **BLDC Hub Motor** | 6.5-inch 36V/350W hoverboard direct-drive hub motor, 15 pole pairs (30 poles), $K_t = 0.550\text{ N}\cdot\text{m/A}$, $R = 0.534\,\Omega$, $J = 0.0131\text{ kg}\cdot\text{m}^2$ | Direct-drive steering actuator (no reduction gearbox, low cogging torque) | 1 | \$35 – \$50 |
| 1.2 | **Motor Controller** | MKS XDrive Mini (ODrive v0.5.6 hardware architecture), STM32F405 MCU, DRV8301 gate driver, 8 kHz FOC position/current control loops | Dual-axis high-speed Field-Oriented Control inverter with ASCII CDC and USB HID | 1 | \$70 – \$95 |
| 1.3 | **Power Supply (PSU)** | Regulated industrial switching DC power supply, $24.3\text{ V}$ DC, $15\text{--}20\text{ A}$ ($350\text{--}500\text{ W}$), passive/active cooling | Inverter DC bus power delivery with high transient current margin | 1 | \$30 – \$45 |
| 1.4 | **Brake Shunt Resistor** | $50\text{ W}$, $2.0\text{--}5.0\,\Omega$ aluminum wirewound power braking resistor | Dissipates regenerative braking energy to prevent DC bus overvoltage during fast steering reversals | 1 | \$5 – \$8 |

---

## 2. Sensing & Feedback

| Item | Component | Specification | Function / Description | Qty | Approx. Cost (USD) |
| :---: | :--- | :--- | :--- | :---: | :---: |
| 2.1 | **Rotary Encoder** | Optical incremental quadrature encoder, 720 PPR (2,880 CPR in $4\times$ decoding), $0.125^\circ/\text{count}$ resolution, 5V DC supply, ABZ differential/TTL output | Handwheel angular position and velocity measurement | 1 | \$15 – \$25 |
| 2.2 | **Transmission Gears** | 1:1 gear pair (3D printed PETG/PLA or commercial nylon), mod 1.0, 40 teeth | Direct mechanical coupling between motor rotor shaft and encoder shaft | 1 pair | \$5 (printed) |
| 2.3 | **Optical Isolator / USB Hub** | USB 2.0 optical isolator / industrial shielded hub (115,200 bps CDC + HID support) | Protects host PC USB controllers against motor ground loops and EMI noise | 1 | \$15 – \$25 |

---

## 3. Mechanical Structure & User Interface

| Item | Component | Specification | Function / Description | Qty | Approx. Cost (USD) |
| :---: | :--- | :--- | :--- | :---: | :---: |
| 3.1 | **Steering Handwheel** | Custom ergonomic steering wheel, 280 mm diameter, 3D printed PETG / PLA+ (40% infill, 4 perimeters) or laser-cut acrylic/aluminum | Physical interface for manual inspection and closed-loop steering visualization | 1 | \$10 – \$20 |
| 3.2 | **Motor Hub Adapter** | Custom 4-bolt hub adapter (3D printed PETG / CNC aluminum) mounting wheel to motor shaft | Rigid mechanical torque transfer from BLDC hub to handwheel | 1 | \$5 – \$15 |
| 3.3 | **Bench Chassis / Base** | Modular aluminum extrusion (2020/2040 profile) or rigid wooden bench clamp base | Vibration-free structural mounting to desk/chassis | 1 | \$20 – \$35 |
| 3.4 | **Hardware Fasteners** | Stainless steel M3, M4, M5 hex socket head cap screws, locking nuts, and washers | Structural assembly | 1 set | \$8 – \$12 |
| 3.5 | **Emergency Stop** | 10A push-lock twist-reset mushroom emergency stop button (wired to PSU AC input or ODrive enable) | Immediate manual safety cutoff | 1 | \$6 – \$10 |

---

## 4. Host Workstation & Software Stack

| Item | Component | Specification | Role |
| :---: | :--- | :--- | :--- |
| 4.1 | **Host PC** | Windows 10/11 64-bit, Intel Core i5/i7 or AMD Ryzen 5/7, 16 GB RAM, dedicated GPU (GTX 1660 / RTX 2060 or better) | Runs vehicle physics simulation, multi-rate bridge, and OSQP solver |
| 4.2 | **Vehicle Simulator** | Assetto Corsa v1.16.4 (Kunos Simulazioni) | 14-DOF multibody chassis physics, suspension kinematics, and Pacejka Magic Formula tire models at 100 Hz |
| 4.3 | **Virtual HID Driver** | vJoy Virtual Joystick Driver (v2.1.9) | DirectInput virtual steering axis bridge without quantization |
| 4.4 | **Python Runtime** | Python 3.10+ (`osqp`, `pyvjoy`, `pyserial`, `numpy`, `scipy`) | Multi-rate communication bridge and 20 Hz convex MPC execution |
| 4.5 | **Data Processing** | MATLAB R2023b or newer (Optimization Toolbox) | Experimental parameter identification and statistical benchmark analysis |
