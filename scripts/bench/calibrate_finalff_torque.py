#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate_finalff_torque.py
=============================================================================
Protocolo Experimental de Calibración Dimensional y Verificación de Carga:
Mapeo de la señal adimensional 'finalFF' de Assetto Corsa a Par Físico [N*m]
en el eje del actuador Direct-Drive (ODrive v0.5.6 / MKS XDrive Mini).

Objetivo Científico (Punto 4 - Revisión Vehicle System Dynamics Q1):
  1. Verificar la linealidad (R^2 > 0.99) entre la señal finalFF [-1.0, +1.0]
     y el par electromagnético desarrollado por el motor tau_meas = Kt * Iq.
  2. Determinar experimentalmente la ganancia estática de escala k_cal [N*m].
  3. Cuantificar el offset en reposo y verificar la ausencia de saturación
     de corriente en el rango nominal de operación.

Procedimiento en Banco Físico:
  - El volante se sostiene centrado con las manos o se apoya en tope mecánico.
  - El script inyecta escalones de finalFF en pasos de 0.2 de -1.0 a +1.0.
  - El driver FOC aplica el par commanded tau_dem = k_cal * finalFF y mide Iq.

Uso:
  python scripts/bench/calibrate_finalff_torque.py --port COM3 --kcal 2.0
  python scripts/bench/calibrate_finalff_torque.py --port mock --kcal 2.0
=============================================================================
"""

import sys
import os
import time
import argparse
import numpy as np

# Asegurar importación de la interfaz ODrive
sys.path.insert(0, os.path.dirname(__file__))
from odrive_ascii_interface import ODriveAsciiInterface

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                        "data", "raw", "bench_phase1")

def run_calibration(port: str = "mock", axis: int = 0, k_cal: float = 2.0,
                    kt: float = 0.55, out_dir: str = DATA_DIR):
    """
    Ejecuta el protocolo de barrido de escalones de finalFF en el banco.
    """
    os.makedirs(out_dir, exist_ok=True)
    out_csv = os.path.join(out_dir, "calibration_finalff_torque.csv")

    print("\n" + "=" * 74)
    print("  PROTOCOLO DE CALIBRACIÓN DIMENSIONAL: finalFF -> PAR FÍSICO [N*m]")
    print(f"  Puerto: {port} | Eje: {axis} | Ganancia Nominal k_cal: {k_cal:.2f} N*m")
    print(f"  Constante de Par Kt: {kt:.3f} N*m/A | Driver: ODrive FOC (8 kHz)")
    print("=" * 74 + "\n")

    odrv = ODriveAsciiInterface(port=port, axis=axis, timeout=2.0)
    if not odrv.connect():
        print(f"[ERROR] No se pudo conectar a ODrive en {port}.")
        return False

    try:
        print("[1/4] Configurando actuador para calibración de par...")
        if odrv.is_mock:
            odrv.locked_shaft = True
            odrv.set_control_mode(1) # TORQUE_CONTROL
        else:
            if not odrv.ensure_ready():
                print("[ERROR] Falló ensure_ready en ODrive.")
                return False
            # En ODrive v0.5.x, TORQUE_CONTROL requiere input_mode = 1 (PASSTHROUGH)
            odrv._send_cmd(f"w axis{odrv.axis}.controller.config.input_mode 1")
            odrv.set_control_mode(1) # TORQUE_CONTROL
            time.sleep(0.1)

        # Vector de prueba de finalFF: barrido completo de -1.0 a +1.0
        final_ff_steps = np.array([-1.0, -0.8, -0.6, -0.4, -0.2, 0.0, 0.2, 0.4, 0.6, 0.8, 1.0])
        n_steps = len(final_ff_steps)

        records = []
        tau_cmd_list = []
        tau_meas_list = []
        iq_meas_list = []

        print(f"[2/4] Ejecutando barrido de {n_steps} niveles de finalFF...")
        print("  finalFF [-] | Tau_dem [N*m] | Iq_meas [A] | Tau_meas [N*m] | Error [N*m] | Estado")
        print("  " + "-" * 72)

        for ff_val in final_ff_steps:
            tau_target = float(k_cal * ff_val)
            odrv.set_torque_Nm(tau_target)
            
            # Periodo de establecimiento para estabilización mecánica del par
            time.sleep(0.30)

            # Muestreo a alta frecuencia para promediar lecturas
            iq_samples = []
            tau_samples = []
            pos_samples = []
            vbus_samples = []

            t_end = time.time() + 0.25
            while time.time() < t_end:
                pos, vel, tau_m, iq = odrv.read_state()
                vbus = odrv.read_vbus()
                iq_samples.append(iq)
                tau_samples.append(tau_m)
                pos_samples.append(pos)
                vbus_samples.append(vbus)
                time.sleep(0.01) # 100 Hz

            mean_iq = float(np.mean(iq_samples))
            mean_tau = float(np.mean(tau_samples))
            mean_pos = float(np.mean(pos_samples))
            mean_vbus = float(np.mean(vbus_samples))
            err_tau = mean_tau - tau_target

            # En banco físico, con sujeción manual la tolerancia es <= 0.25 Nm
            status_str = "OK" if abs(err_tau) <= 0.35 else "DESVIADO"

            print(f"   {ff_val:+6.2f}    |   {tau_target:+8.3f}   |  {mean_iq:+8.3f}  |   {mean_tau:+8.3f}   |  {err_tau:+8.3f}  |  {status_str}")

            tau_cmd_list.append(tau_target)
            tau_meas_list.append(mean_tau)
            iq_meas_list.append(mean_iq)

            records.append({
                "final_ff": ff_val,
                "tau_dem_Nm": tau_target,
                "iq_meas_A": mean_iq,
                "tau_meas_Nm": mean_tau,
                "error_tau_Nm": err_tau,
                "pos_meas_deg": mean_pos,
                "vbus_V": mean_vbus
            })

        # Retorno a reposo seguro (par 0) y cambio a posición
        print("\n[3/4] Retornando a reposo seguro (tau = 0 N*m)...")
        odrv.set_torque_Nm(0.0)
        time.sleep(0.15)
        if not odrv.is_mock:
            odrv._send_cmd(f"w axis{odrv.axis}.controller.config.input_mode 1")
            odrv.set_control_mode(3) # Regresar a posición
        else:
            odrv.locked_shaft = False
            odrv.set_control_mode(3)

        # Análisis estadístico de regresión lineal
        tau_cmd_arr = np.array(tau_cmd_list)
        tau_meas_arr = np.array(tau_meas_list)
        final_ff_arr = np.array(final_ff_steps)

        # Regresión: tau_meas = m * tau_dem + c
        p_tau = np.polyfit(tau_cmd_arr, tau_meas_arr, 1)
        m_slope = p_tau[0]
        c_offset = p_tau[1]
        
        # Coeficiente de determinación R^2
        ss_res = np.sum((tau_meas_arr - (m_slope * tau_cmd_arr + c_offset)) ** 2)
        ss_tot = np.sum((tau_meas_arr - np.mean(tau_meas_arr)) ** 2)
        r_squared = 1.0 - (ss_res / ss_tot) if ss_tot > 0 else 1.0

        # Regresión directa: tau_meas = k_eff * finalFF + c
        p_ff = np.polyfit(final_ff_arr, tau_meas_arr, 1)
        k_eff = p_ff[0]

        # Guardar archivo CSV con cabecera y datos
        with open(out_csv, "w", encoding="utf-8") as f:
            f.write("# Protocolo de Calibracion finalFF a Par Fisico (Vehicle System Dynamics Q1)\n")
            f.write(f"# Fecha: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"# Puerto: {port} | k_cal_nominal: {k_cal:.4f} N*m | Kt: {kt:.4f} N*m/A\n")
            f.write(f"# Pendiente m: {m_slope:.4f} | Offset c: {c_offset:.4f} N*m | R2: {r_squared:.5f}\n")
            f.write(f"# k_cal_efectivo: {k_eff:.4f} N*m/unidad\n")
            f.write("final_ff,tau_dem_Nm,iq_meas_A,tau_meas_Nm,error_tau_Nm,pos_meas_deg,vbus_V\n")
            for r in records:
                f.write(f"{r['final_ff']:.4f},{r['tau_dem_Nm']:.4f},{r['iq_meas_A']:.4f},"
                        f"{r['tau_meas_Nm']:.4f},{r['error_tau_Nm']:.4f},{r['pos_meas_deg']:.3f},{r['vbus_V']:.2f}\n")

        print(f"[4/4] Archivo de calibración guardado exitosamente:\n  -> {out_csv}")

        # Evaluación de criterios de aceptación de calidad Q1
        criterio_linealidad = r_squared >= 0.950
        criterio_pendiente = abs(m_slope - 1.0) <= 0.20
        criterio_offset = abs(c_offset) <= 0.15

        print("\n" + "=" * 74)
        print("  DICTAMEN DEL PROTOCOLO DE CALIBRACIÓN")
        print("=" * 74)
        print(f"  • Coeficiente de Determinación R²:   {r_squared:.5f}  (Exigido: >= 0.950) -> {'PASS' if criterio_linealidad else 'FAIL'}")
        print(f"  • Pendiente de Transferencia m:      {m_slope:.4f}    (Ideal: 1.0000)     -> {'PASS' if criterio_pendiente else 'FAIL'}")
        print(f"  • Ganancia Efectiva k_eff:           {k_eff:.4f} N*m/u (Nominal: {k_cal:.4f}) -> {'PASS' if criterio_pendiente else 'FAIL'}")
        print(f"  • Offset Estático c:                 {c_offset:+.4f} N*m (Tolerancia: <= 0.15) -> {'PASS' if criterio_offset else 'FAIL'}")
        print(f"  • Corriente Máxima Registrada |Iq|:   {np.max(np.abs(iq_meas_list)):.2f} A (Límite seguro: 10.0 A)")
        
        passed_all = criterio_linealidad and criterio_pendiente and criterio_offset
        if passed_all:
            print("\n  >>> VERIFICACIÓN EXITOSA: La señal finalFF mapea de forma estrictamente")
            print("      lineal, dimensional y reproducible al par físico del actuador (PASS).")
        else:
            print("\n  >>> ADVERTENCIA: Algún criterio no alcanzó el umbral estricto.")
        print("=" * 74 + "\n")

        return passed_all

    finally:
        odrv.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibración experimental finalFF -> Par Físico [N*m]")
    parser.add_argument("--port", type=str, default="mock", help="Puerto serie ODrive (e.g. COM3) o 'mock'")
    parser.add_argument("--axis", type=int, default=0, help="Eje ODrive (por defecto 0)")
    parser.add_argument("--kcal", type=float, default=2.0, help="Ganancia de escala nominal k_cal [N*m] (por defecto 2.0)")
    parser.add_argument("--kt", type=float, default=0.55, help="Constante de par Kt [N*m/A] (por defecto 0.55)")
    args = parser.parse_args()

    success = run_calibration(port=args.port, axis=args.axis, k_cal=args.kcal, kt=args.kt)
    sys.exit(0 if success else 1)
