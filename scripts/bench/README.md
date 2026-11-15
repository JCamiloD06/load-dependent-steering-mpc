# Hardware-in-the-Loop (HIL) Execution & Actuator Bridge

This directory contains the Python drivers and real-time supervisory bridges interfacing the physical direct-drive steering testbench (ODrive v0.5.6 / MKS XDrive Mini) with the Assetto Corsa physics engine.

---

## 1. Module Overview

| Script Name | Purpose / Function | Update Rate | Communication Interface |
| :--- | :--- | :---: | :--- |
| `odrive_ascii_interface.py` | Low-level driver managing USB CDC serial telemetry, motor calibration, and closed-loop torque/position commands. | 50 Hz | Serial USB COM port (115,200 bps / 12 Mbps USB FS) |
| `run_phase1_battery.py` | Automated orchestrator executing the 5 physical identification experiments (Inertia, Transport Delay, Rate Saturation, Frequency Chirp, Loaded Torque vs Speed). | 50 Hz | ODrive ASCII protocol |
| `ffb_hid.py` | Interacts with USB DirectInput Force Feedback (FFB) commands and vJoy virtual joystick device. | 20 Hz | Windows DirectInput / pyvjoy API |
| `calibrate_finalff_torque.py` | In-situ calibration routine mapping tire self-aligning torque $\tau_a$ from the physics simulation into motor electromagnetic torque commands. | 50 Hz | Closed-loop torque calibration |
| `verify_coexistence_and_load.py` | Health-check script validating simultaneous operation of USB serial ASCII telemetry, vJoy steering input, and shared memory reading. | 20 Hz | System health check |

---

## 2. Requirements & Setup

1. **Python Virtual Environment:**
   ```bash
   python -m venv venv
   source venv/Scripts/activate  # On Windows: venv\Scripts\activate
   pip install pyserial pyvjoy numpy scipy osqp
   ```

2. **ODrive Configuration:**
   - Connect the MKS XDrive Mini via USB.
   - Run the automated calibration check:
     ```bash
     python scripts/bench/odrive_ascii_interface.py --test-connection
     ```

3. **Running the Phase 1 Identification Battery:**
   ```bash
   python scripts/bench/run_phase1_battery.py --all-tests
   ```
   *Logs are written to:* `data/raw/bench_phase1/`.
