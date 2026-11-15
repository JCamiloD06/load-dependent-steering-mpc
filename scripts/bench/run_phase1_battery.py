"""
Run Phase 1 Test Battery on Physical Steering Bench
===================================================
Phase 1 Actuator Dynamic Identification Battery (Physical Bench)
Ejecuta de forma sistemática y reproducible la batería de los 5 ensayos
físicos en banco para la caracterización de capacidad dinámica del actuador.

Ensayos:
- Ensayo 1: Identificación de inercia, fricción seca y viscosa (Escalones de par y rueda libre).
- Ensayo 2: Medición de retardo de transporte puro T_d (Pulsos de pequeña señal a alta frecuencia).
- Ensayo 3: Determinación de velocidad angular máxima en vacío (Saturación de tasa en escalones amplios).
- Ensayo 4: Caracterización de respuesta en frecuencia (Barrido sinusoidal chirp 0.2 - 12 Hz).
- Ensayo 5: Medición de velocidad angular máxima contra par dinámico resistivo (Curva par-velocidad bajo carga FFB).

Mínimo 5 repeticiones por condición para análisis de dispersión y deriva térmica.
"""

import os
import sys
import time
import math
import csv
from typing import List, Dict, Any
from odrive_ascii_interface import ODriveAsciiInterface

# Configuración de carpetas
DATA_DIR = os.path.join("data", "raw", "bench_phase1")

def ensure_dir(path: str):
    os.makedirs(path, exist_ok=True)

def write_csv(filepath: str, fieldnames: List[str], rows: List[Dict[str, Any]]):
    with open(filepath, "w", newline="", encoding="utf-8") as fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"  -> Guardado: {filepath} ({len(rows)} muestras)")

def run_test1_inertia_friction(odrv: ODriveAsciiInterface, n_reps: int = 5, out_base_dir: str = DATA_DIR):
    """
    Ensayo 1: Escalones de posición en vacío (+-30, +-60, +-90 deg)
    en lazo cerrado para identificar:
    - J: Momento de inercia a partir de la dinámica acoplada J*alpha + b*omega + tau_c*sign(omega) = Kt*Iq.
    - b y tau_c: Coeficientes de amortiguamiento viscoso y fricción de Coulomb.
    - Tiempo de establecimiento al 2% y aceleración angular máxima en vacío.
    """
    test_name = "test1_inertia_friction"
    out_dir = os.path.join(out_base_dir, test_name)
    ensure_dir(out_dir)
    print(f"\n=======================================================")
    print(f"  EJECUTANDO ENSAYO 1: Inercia y Fricción ({n_reps} repeticiones)")
    print(f"  (Escalones controlados en lazo cerrado: +-30°, +-60°, +-90°)")
    print(f"=======================================================")

    odrv.set_control_mode(3) # Modo posición siempre activo y acotado
    angles = [30.0, -30.0, 45.0, -45.0, 60.0, -60.0]
    fieldnames = ["time_s", "pos_cmd_deg", "vel_cmd_deg_s", "torque_cmd_Nm",
                  "pos_meas_deg", "vel_meas_deg_s", "torque_meas_Nm", "iq_meas_A", "vbus_V", "rep_id", "condition"]

    for angle in angles:
        cond_str = f"step_{angle:+.0f}deg"
        for rep in range(1, n_reps + 1):
            # Retornar suavemente a cero y esperar reposo absoluto
            odrv.wait_settle(0.0, timeout=1.5)

            rows = []
            t_start = time.time()
            duration = 1.0 # 1.0s por escalón

            while True:
                now = time.time()
                t_rel = now - t_start
                if t_rel >= duration:
                    break

                pos_cmd = angle if t_rel >= 0.1 else 0.0
                odrv.set_position_deg(pos_cmd)
                pos, vel, tau_meas, iq = odrv.read_state()
                vbus = odrv.read_vbus()

                # GUARDA DE SEGURIDAD INMEDIATA CONTRA CENTRIFUGADO
                if abs(pos) > 220.0 or abs(vel) > 2200.0:
                    print(f"\n[ALERTA SEGURIDAD] Desviación anómala detectada: pos={pos:.1f}°, vel={vel:.1f}°/s.")
                    print("Desenergizando motor a IDLE inmediatamente por seguridad...")
                    odrv.set_idle()
                    raise RuntimeError(f"Guarda de seguridad activada: pos={pos:.1f}°, vel={vel:.1f}°/s")

                rows.append({
                    "time_s": f"{t_rel:.4f}",
                    "pos_cmd_deg": f"{pos_cmd:.3f}",
                    "vel_cmd_deg_s": "0.000",
                    "torque_cmd_Nm": "0.000",
                    "pos_meas_deg": f"{pos:.3f}",
                    "vel_meas_deg_s": f"{vel:.3f}",
                    "torque_meas_Nm": f"{tau_meas:.4f}",
                    "iq_meas_A": f"{iq:.4f}",
                    "vbus_V": f"{vbus:.2f}",
                    "rep_id": rep,
                    "condition": cond_str
                })
                time.sleep(0.004) # 250 Hz

            odrv.wait_settle(0.0, timeout=1.5)
            filename = f"t1_{cond_str}_rep{rep:02d}.csv"
            write_csv(os.path.join(out_dir, filename), fieldnames, rows)

    odrv.wait_settle(0.0, timeout=1.5)

def run_test2_time_delay(odrv: ODriveAsciiInterface, n_reps: int = 5, out_base_dir: str = DATA_DIR):
    """
    Ensayo 2: Pulsos rápidos de pequeña señal (+-10, +-15, +-20 deg) para medir:
    - Retardo puro de transporte T_d desde el timestamp de despacho hasta el 1er movimiento.
    """
    test_name = "test2_time_delay"
    out_dir = os.path.join(out_base_dir, test_name)
    ensure_dir(out_dir)
    print(f"\n=======================================================")
    print(f"  EJECUTANDO ENSAYO 2: Retardo Puro de Transporte T_d ({n_reps} reps)")
    print(f"=======================================================")

    odrv.set_control_mode(3) # Position control
    amplitudes = [10.0, -10.0, 15.0, -15.0, 20.0, -20.0]
    fieldnames = ["time_s", "pos_cmd_deg", "vel_cmd_deg_s", "torque_cmd_Nm",
                  "pos_meas_deg", "vel_meas_deg_s", "torque_meas_Nm", "iq_meas_A", "vbus_V", "rep_id", "condition"]

    for amp in amplitudes:
        cond_str = f"pulse_{amp:+.0f}deg"
        for rep in range(1, n_reps + 1):
            # Centrar y verificar reposo
            odrv.wait_settle(0.0, timeout=1.8)

            rows = []
            t_start = time.time()
            duration = 0.5
            cmd_applied = False

            while True:
                now = time.time()
                t_rel = now - t_start
                if t_rel >= duration:
                    break

                # Aplicar escalón exactamente a los 100 ms
                if t_rel >= 0.100 and not cmd_applied:
                    pos_cmd = amp
                    cmd_applied = True
                elif not cmd_applied:
                    pos_cmd = 0.0
                else:
                    pos_cmd = amp

                odrv.set_position_deg(pos_cmd)
                pos, vel, tau_meas, iq = odrv.read_state()
                vbus = odrv.read_vbus()

                # Guarda de seguridad
                if abs(pos) > 65.0 or abs(vel) > 1600.0:
                    odrv.set_idle()
                    raise RuntimeError(f"Guarda de seguridad Ensayo 2: pos={pos:.1f}°")

                rows.append({
                    "time_s": f"{t_rel:.4f}",
                    "pos_cmd_deg": f"{pos_cmd:.3f}",
                    "vel_cmd_deg_s": "0.000",
                    "torque_cmd_Nm": "0.000",
                    "pos_meas_deg": f"{pos:.3f}",
                    "vel_meas_deg_s": f"{vel:.3f}",
                    "torque_meas_Nm": f"{tau_meas:.4f}",
                    "iq_meas_A": f"{iq:.4f}",
                    "vbus_V": f"{vbus:.2f}",
                    "rep_id": rep,
                    "condition": cond_str
                })
                time.sleep(0.002) # 500 Hz para alta resolución temporal

            odrv.wait_settle(0.0, timeout=1.0)
            filename = f"t2_{cond_str}_rep{rep:02d}.csv"
            write_csv(os.path.join(out_dir, filename), fieldnames, rows)

    odrv.wait_settle(0.0, timeout=1.2)

def run_test3_rate_saturation(odrv: ODriveAsciiInterface, n_reps: int = 5, out_base_dir: str = DATA_DIR):
    """
    Ensayo 3: Escalones amplios (+-30, +-90, +-180 deg) para medir:
    - Velocidad máxima de saturación (Omega_max en vacío).
    - Aceleración angular máxima (alpha_max).
    - Tiempo de establecimiento al 2%.
    """
    test_name = "test3_rate_saturation"
    out_dir = os.path.join(out_base_dir, test_name)
    ensure_dir(out_dir)
    print(f"\n=======================================================")
    print(f"  EJECUTANDO ENSAYO 3: Saturación de Tasa en Vacío ({n_reps} reps)")
    print(f"=======================================================")

    odrv.set_control_mode(3)
    # Ángulos acotados para garantizar que la inercia del aro nunca alcance el tope físico (~170°)
    angles = [25.0, -25.0, 40.0, -40.0, 55.0, -55.0, 70.0, -70.0]
    fieldnames = ["time_s", "pos_cmd_deg", "vel_cmd_deg_s", "torque_cmd_Nm",
                  "pos_meas_deg", "vel_meas_deg_s", "torque_meas_Nm", "iq_meas_A", "vbus_V", "rep_id", "condition"]

    for angle in angles:
        cond_str = f"step_{angle:+.0f}deg"
        for rep in range(1, n_reps + 1):
            odrv.wait_settle(0.0, timeout=1.8)

            rows = []
            t_start = time.time()
            duration = 1.0

            while True:
                now = time.time()
                t_rel = now - t_start
                if t_rel >= duration:
                    break

                pos_cmd = angle if t_rel >= 0.1 else 0.0
                odrv.set_position_deg(pos_cmd)
                pos, vel, tau_meas, iq = odrv.read_state()
                vbus = odrv.read_vbus()

                # Guarda de seguridad (tope físico rigurosamente protegido a 130°)
                if abs(pos) > 130.0 or abs(vel) > 2400.0:
                    print(f"\n[ALERTA SEGURIDAD] Desviación anómala en Ensayo 3: pos={pos:.1f}°.")
                    odrv.set_idle()
                    raise RuntimeError(f"Guarda de seguridad Ensayo 3: pos={pos:.1f}°")

                rows.append({
                    "time_s": f"{t_rel:.4f}",
                    "pos_cmd_deg": f"{pos_cmd:.3f}",
                    "vel_cmd_deg_s": "0.000",
                    "torque_cmd_Nm": "0.000",
                    "pos_meas_deg": f"{pos:.3f}",
                    "vel_meas_deg_s": f"{vel:.3f}",
                    "torque_meas_Nm": f"{tau_meas:.4f}",
                    "iq_meas_A": f"{iq:.4f}",
                    "vbus_V": f"{vbus:.2f}",
                    "rep_id": rep,
                    "condition": cond_str
                })
                time.sleep(0.004) # 250 Hz

            odrv.wait_settle(0.0, timeout=1.8)
            filename = f"t3_{cond_str}_rep{rep:02d}.csv"
            write_csv(os.path.join(out_dir, filename), fieldnames, rows)

    odrv.wait_settle(0.0, timeout=1.8)

def run_test4_frequency_chirp(odrv: ODriveAsciiInterface, n_reps: int = 5, out_base_dir: str = DATA_DIR):
    """
    Ensayo 4: Barrido sinusoidal (chirp) de 0.2 Hz a 12.0 Hz:
    - Amplitud pequeña (+-10 deg) -> Comportamiento lineal / ancho de banda -3dB.
    - Amplitud grande (+-45 deg) -> Respuesta bajo saturación de tasa de gran señal.
    """
    test_name = "test4_frequency_chirp"
    out_dir = os.path.join(out_base_dir, test_name)
    ensure_dir(out_dir)
    print(f"\n=======================================================")
    print(f"  EJECUTANDO ENSAYO 4: Respuesta en Frecuencia / Chirp ({n_reps} reps)")
    print(f"=======================================================")

    odrv.set_control_mode(3)
    chirp_configs = [("small_10deg", 10.0), ("large_20deg", 20.0)]
    fieldnames = ["time_s", "pos_cmd_deg", "vel_cmd_deg_s", "torque_cmd_Nm",
                  "pos_meas_deg", "vel_meas_deg_s", "torque_meas_Nm", "iq_meas_A", "vbus_V", "rep_id", "condition"]

    f0 = 0.2  # Hz inicial
    f1 = 12.0 # Hz final
    t_chirp = 10.0 # Segundos de barrido

    for cond_name, amp in chirp_configs:
        for rep in range(1, n_reps + 1):
            filename = f"t4_{cond_name}_rep{rep:02d}.csv"
            filepath = os.path.join(out_dir, filename)
            if os.path.exists(filepath) and os.path.getsize(filepath) > 1000:
                print(f"-> Ya existe: {filepath} (completado)")
                continue

            odrv.wait_settle(0.0, timeout=1.5)

            rows = []
            t_start = time.time()

            while True:
                now = time.time()
                t_rel = now - t_start
                if t_rel >= t_chirp:
                    break

                # Chirp lineal: f(t) = f0 + (f1 - f0) * (t / t_chirp)
                phase = 2.0 * math.pi * (f0 * t_rel + 0.5 * (f1 - f0) * (t_rel ** 2) / t_chirp)
                pos_cmd = amp * math.sin(phase)
                vel_ff = amp * 2.0 * math.pi * (f0 + (f1 - f0) * t_rel / t_chirp) * math.cos(phase)

                odrv.set_position_deg(pos_cmd, vel_ff_deg_s=vel_ff)
                pos, vel, tau_meas, iq = odrv.read_state()
                vbus = odrv.read_vbus()

                # Guarda de seguridad (tope físico rigurosamente protegido a 120°)
                if abs(pos) > 120.0 or abs(vel) > 2500.0:
                    odrv.set_idle()
                    raise RuntimeError(f"Guarda de seguridad Ensayo 4: pos={pos:.1f}°, vel={vel:.1f}°/s")

                rows.append({
                    "time_s": f"{t_rel:.4f}",
                    "pos_cmd_deg": f"{pos_cmd:.3f}",
                    "vel_cmd_deg_s": f"{vel_ff:.3f}",
                    "torque_cmd_Nm": "0.000",
                    "pos_meas_deg": f"{pos:.3f}",
                    "vel_meas_deg_s": f"{vel:.3f}",
                    "torque_meas_Nm": f"{tau_meas:.4f}",
                    "iq_meas_A": f"{iq:.4f}",
                    "vbus_V": f"{vbus:.2f}",
                    "rep_id": rep,
                    "condition": cond_name
                })
                time.sleep(0.004) # 250 Hz

            odrv.wait_settle(0.0, timeout=1.5)
            write_csv(filepath, fieldnames, rows)

    odrv.wait_settle(0.0, timeout=1.5)

def run_test5_torque_speed_load(odrv: ODriveAsciiInterface, n_reps: int = 5, out_base_dir: str = DATA_DIR):
    """
    Ensayo 5: Medición de velocidad angular máxima contra par dinámico resistivo (FFB).
    - Cargas resistivas externas: tau_load in [0.0, 0.5, 1.0, 1.5, 2.0, 2.5] N*m en sentido opuesto.
    - Demanda de giro a velocidad máxima para mapear la contracción de Omega_max(tau_load).
    """
    test_name = "test5_torque_speed_load"
    out_dir = os.path.join(out_base_dir, test_name)
    ensure_dir(out_dir)
    print(f"\n=======================================================")
    print(f"  EJECUTANDO ENSAYO 5: Curva Par-Velocidad bajo Carga FFB ({n_reps} reps)")
    print(f"=======================================================")

    load_levels = [0.0, 0.5, 1.0, 1.5, 2.0]
    fieldnames = ["time_s", "pos_cmd_deg", "vel_cmd_deg_s", "torque_cmd_Nm",
                  "pos_meas_deg", "vel_meas_deg_s", "torque_meas_Nm", "iq_meas_A", "vbus_V", "rep_id", "condition"]

    for load in load_levels:
        cond_str = f"load_{load:.1f}Nm"
        for rep in range(1, n_reps + 1):
            filename = f"t5_{cond_str}_rep{rep:02d}.csv"
            filepath = os.path.join(out_dir, filename)
            if os.path.exists(filepath) and os.path.getsize(filepath) > 1000:
                print(f"-> Ya existe: {filepath} (completado)")
                continue

            odrv.set_control_mode(3)
            odrv.set_external_load_torque(0.0)
            odrv.wait_settle(-50.0, timeout=1.8) # Posicionar suavemente en -50°

            # Activar contra-par resistivo opuesto al giro hacia +50 deg
            odrv.set_external_load_torque(load)

            rows = []
            t_start = time.time()
            duration = 1.0

            while True:
                now = time.time()
                t_rel = now - t_start
                if t_rel >= duration:
                    break

                # Demanda de paso completo de -50 a +50 grados (barrido de 100 deg contra carga resistiva)
                pos_cmd = 50.0 if t_rel >= 0.1 else -50.0
                opposing_tau = -load if (t_rel >= 0.1 and load > 0.0) else 0.0
                odrv.set_position_deg(pos_cmd, torque_ff_Nm=opposing_tau)
                pos, vel, tau_meas, iq = odrv.read_state()
                vbus = odrv.read_vbus()

                # Guarda de seguridad (tope físico rigurosamente protegido a 120°)
                if abs(pos) > 120.0 or abs(vel) > 2400.0:
                    odrv.set_external_load_torque(0.0)
                    odrv.set_idle()
                    raise RuntimeError(f"Guarda de seguridad Ensayo 5: pos={pos:.1f}°, vel={vel:.1f}°/s")

                rows.append({
                    "time_s": f"{t_rel:.4f}",
                    "pos_cmd_deg": f"{pos_cmd:.3f}",
                    "vel_cmd_deg_s": "0.000",
                    "torque_cmd_Nm": f"{load:.4f}",
                    "pos_meas_deg": f"{pos:.3f}",
                    "vel_meas_deg_s": f"{vel:.3f}",
                    "torque_meas_Nm": f"{tau_meas:.4f}",
                    "iq_meas_A": f"{iq:.4f}",
                    "vbus_V": f"{vbus:.2f}",
                    "rep_id": rep,
                    "condition": cond_str
                })
                time.sleep(0.004) # 250 Hz

            odrv.set_external_load_torque(0.0)
            odrv.wait_settle(0.0, timeout=1.8)
            write_csv(filepath, fieldnames, rows)

    odrv.set_external_load_torque(0.0)
    odrv.wait_settle(0.0, timeout=1.8)

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Batería de Ensayos de Fase 1 - Banco Físico ODrive")
    parser.add_argument("port_pos", nargs="?", default=None, help="Puerto COM posicional (opcional)")
    parser.add_argument("--port", default=None, help="Puerto serie COM (ej. COM3, COM6, o mock)")
    parser.add_argument("--out-dir", default=DATA_DIR, help="Directorio raíz para almacenar CSVs")
    parser.add_argument("--n-reps", "--reps", type=int, default=5, dest="n_reps", help="Numero de repeticiones")
    parser.add_argument("--mock", action="store_true", help="Forzar modo simulacion mock")
    parser.add_argument("--tests", default="all", help="Ensayos a ejecutar")
    args = parser.parse_args()

    port = "mock" if args.mock else (args.port or args.port_pos or "mock")
    out_dir = args.out_dir
    n_reps = args.n_reps

    tests_str = str(args.tests).strip().lower()
    if tests_str == "all":
        selected_tests = [1, 2, 3, 4, 5]
    else:
        selected_tests = []
        for x in tests_str.split(","):
            x = x.strip()
            if x.isdigit():
                selected_tests.append(int(x))

    print(f"=======================================================")
    print(f"  BATERÍA DE ENSAYOS DE FASE 1 - BANCO FÍSICO ODRIVE")
    print(f"  Puerto seleccionado : {port}")
    print(f"  Carpeta de salida   : {out_dir}")
    print(f"  Repeticiones        : {n_reps}")
    print(f"  Ensayos a ejecutar  : {selected_tests}")
    print(f"=======================================================")

    odrv = ODriveAsciiInterface(port=port)
    if not odrv.connect():
        print(f"ERROR FATAL: No se pudo conectar con ODrive en {port}. Abortando.")
        sys.exit(1)

    if not odrv.ensure_ready():
        print(f"ERROR FATAL: No se pudo armar el actuador ODrive en {port} en lazo cerrado. Abortando.")
        odrv.close()
        sys.exit(1)

    try:
        if 1 in selected_tests:
            run_test1_inertia_friction(odrv, n_reps=n_reps, out_base_dir=out_dir)
        if 2 in selected_tests:
            run_test2_time_delay(odrv, n_reps=n_reps, out_base_dir=out_dir)
        if 3 in selected_tests:
            run_test3_rate_saturation(odrv, n_reps=n_reps, out_base_dir=out_dir)
        if 4 in selected_tests:
            run_test4_frequency_chirp(odrv, n_reps=n_reps, out_base_dir=out_dir)
        if 5 in selected_tests:
            run_test5_torque_speed_load(odrv, n_reps=n_reps, out_base_dir=out_dir)

        print("\n=======================================================")
        print("  BATERÍA DE ENSAYOS DE FASE 1 COMPLETADA CON ÉXITO")
        print(f"  Datos almacenados en: {out_dir}")
        print("=======================================================\n")
    finally:
        odrv.close()

if __name__ == "__main__":
    main()
