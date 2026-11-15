"""
ODrive ASCII Interface for Physical Steering Test Bench
======================================================
Physical Direct-Drive Steering Actuator Interface (ODrive ASCII Protocol)
Interfaz serie de bajo nivel para placa ODrive v0.5.6 / MKS XDrive Mini.
Soporta comunicación serie ASCII real y modo emulado ('mock') para validación.

Parámetros nominales del actuador:
- Motor: BLDC Direct-Drive (Hoverboard motor, 15 pares de polos)
- Resistencia fase-fase: R = 0.531 Ohm
- Inductancia: L = 0.780 mH
- Tensión de bus nominal: V_bus = 24.0 V
- Relación de conversión encoder: 2880 cuentas/vuelta (0.125 deg/cuenta)
- Relación angular: 1 vuelta = 360 grados = 2*pi radianes
"""

import time
import math
import collections
from typing import Tuple, Optional

try:
    import serial
except ImportError:
    serial = None


class ODriveAsciiInterface:
    def __init__(self, port: str = "mock", baudrate: int = 115200, timeout: float = 0.05, axis: int = 0):
        self.port_name = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.axis = axis
        self.is_mock = (port.lower() == "mock")
        self.ser = None
        self.connected = False
        self.pos_zero_turns = 0.0
        self.current_load_torque_Nm = 0.0
        self.ffb = None

        # Constantes electromecánicas del motor (calibradas en banco ODrive)
        self.Kt = 0.55           # Nm/A (constante de par real del motor en driver)
        self.Ke = 0.16           # V/(rad/s) (constante contra-electromotriz)
        self.Vbus_nom = 24.0     # V
        self.R = 0.531           # Ohm
        self.Iq_max = 18.0       # A (límite de corriente segura en inversor)
        self.J_true = 0.0245     # kg*m^2 (inercia rotor + eje + volante físico)
        self.b_true = 0.0182     # N*m*s/rad (fricción viscosa)
        self.tau_c_true = 0.125  # N*m (fricción de Coulomb)
        self.delay_true_s = 0.012 # 12 ms retardo de transporte puro serie/FOC

        # Estado interno para simulación 'mock'
        self.sim_time = 0.0
        self.sim_pos_rad = 0.0
        self.sim_vel_rad_s = 0.0
        self.sim_iq_meas = 0.0
        self.sim_control_mode = 3 # 3=Position, 2=Velocity, 1=Torque
        self.sim_target_pos = 0.0
        self.sim_target_vel = 0.0
        self.sim_target_torque = 0.0
        self.sim_tau_ext = 0.0    # Carga externa / FFB
        self.locked_shaft = False # Bloqueo estático para calibración de par
        
        # Buffer de retardo para modo mock
        self._cmd_history = collections.deque()

    def connect(self) -> bool:
        """Abre la conexión serie o inicializa el modo mock."""
        if self.is_mock:
            print(f"[ODrive] Modo MOCK activado. Simulación física interna activa.")
            self.connected = True
            self.reset_sim()
            return True

        if serial is None:
            if not self.is_mock:
                print("[ODrive ERROR] 'pyserial' no está instalado.")
                return False
            print("[ODrive ERROR] 'pyserial' no está instalado. Conmutando a modo mock.")
            self.is_mock = True
            self.connected = True
            return True

        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout)
            # Limpiar buffers
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.connected = True
            print(f"[ODrive] Conectado exitosamente en puerto real: {self.port_name} ({self.baudrate} bps)")
            try:
                from ffb_hid import FFBHid
                self.ffb = FFBHid()
                print(f"[ODrive OK] FFB USB HID vinculado (escala: {self.ffb.nm_por_nivel:.3f} Nm/nivel).")
            except Exception as e_ffb:
                print(f"[ODrive INFO] FFB HID no activo: {e_ffb}")
                self.ffb = None
            return True
        except Exception as e:
            if not self.is_mock:
                print(f"[ODrive ERROR] No se pudo abrir el puerto real {self.port_name}: {e}")
                self.connected = False
                return False
            print(f"[ODrive ADVERTENCIA] No se pudo abrir {self.port_name}: {e}. Conmutando a modo mock.")
            self.is_mock = True
            self.connected = True
            self.reset_sim()
            return True

    def close(self):
        """Cierra el puerto serie y libera haptics de forma segura."""
        if self.ffb:
            try:
                self.ffb.cerrar()
            except Exception:
                pass
            self.ffb = None
        if self.ser and self.ser.is_open:
            try:
                self.set_idle()
                self.ser.close()
            except Exception:
                pass
        self.connected = False
        print("[ODrive] Conexión cerrada.")

    def reset_sim(self):
        """Reinicia los estados de la simulación física."""
        self.sim_time = time.time()
        self.sim_pos_rad = 0.0
        self.sim_vel_rad_s = 0.0
        self.sim_iq_meas = 0.0
        self.sim_target_pos = 0.0
        self.sim_target_vel = 0.0
        self.sim_target_torque = 0.0
        self.sim_tau_ext = 0.0
        self._cmd_history.clear()

    # --- Comandos de control de estado y modo ---

    def ensure_ready(self) -> bool:
        """
        Verifica calibración y arma el lazo cerrado en modo posición:
        1. Limpia flags con 'sc'.
        2. Si encoder.is_ready == 0, ejecuta calibración de offset (state 7).
        3. Configura control_mode = 3 (posición), input_mode = 1 (passthrough), enable_watchdog = 0.
        4. Fija self.pos_zero_turns a la posición física actual para centrado relativo sin saltos.
        5. Entra en AXIS_STATE_CLOSED_LOOP_CONTROL (8) y confirma estado.
        """
        if self.is_mock:
            return True

        print("[ODrive] Verificando estado del actuador...")
        self._send_cmd("sc")
        time.sleep(0.1)

        # Verificar si el encoder está listo
        is_ready_str = self._query(f"r axis{self.axis}.encoder.is_ready")
        is_ready = (is_ready_str and is_ready_str.strip() in ("1", "1.0", "True", "true"))

        if not is_ready:
            print("[ODrive] Encoder no sincronizado tras encendido. Ejecutando calibración de offset...")
            print("         (El motor emitirá un tono y girará suavemente)")
            self._send_cmd(f"w axis{self.axis}.requested_state 7")
            t0 = time.time()
            while time.time() - t0 < 15.0:
                time.sleep(0.5)
                is_rdy = self._query(f"r axis{self.axis}.encoder.is_ready")
                st = self._query(f"r axis{self.axis}.current_state")
                if (is_rdy and is_rdy.strip() in ("1", "1.0", "True", "true")) or (st and st.strip() not in ("7", "7.0")):
                    break

            is_ready_str = self._query(f"r axis{self.axis}.encoder.is_ready")
            is_ready = (is_ready_str and is_ready_str.strip() in ("1", "1.0", "True", "true"))
            if not is_ready:
                err = self._query(f"r axis{self.axis}.encoder.error")
                print(f"[ODrive ERROR] Calibración falló. Error encoder: {err}")
                return False
            print("[ODrive OK] Encoder sincronizado.")

        # Configurar lazo de posición, límites de seguridad física y desactivar watchdog en RAM
        self._send_cmd(f"w axis{self.axis}.config.enable_watchdog 0")
        self._send_cmd(f"w axis{self.axis}.controller.config.vel_limit 5.5")   # Cota de seguridad: max 1980 deg/s
        self._send_cmd(f"w axis{self.axis}.motor.config.current_lim 10.0")     # Cota de seguridad: max 10.0 A
        self._send_cmd(f"w axis{self.axis}.controller.config.vel_integrator_gain 0.0") # Eliminar windup de integrador en escalones
        self._send_cmd(f"w axis{self.axis}.controller.config.input_mode 3")    # INPUT_MODE_POS_FILTER (transición rápida sin golpe inercial)
        self._send_cmd(f"w axis{self.axis}.controller.config.input_filter_bandwidth 12.0")
        self._send_cmd(f"w axis{self.axis}.controller.config.control_mode 3")
        time.sleep(0.05)

        # Leer posición actual como referencia de centro relativo
        line = self._query(f"f {self.axis}")
        if line:
            parts = line.strip().split()
            if parts:
                try:
                    self.pos_zero_turns = float(parts[0])
                except ValueError:
                    self.pos_zero_turns = 0.0

        # Sostener posición inicial antes de entrar en lazo cerrado
        self._send_cmd(f"p {self.axis} {self.pos_zero_turns:.4f}")
        self._send_cmd(f"w axis{self.axis}.requested_state 8")
        time.sleep(0.2)

        st = self._query(f"r axis{self.axis}.current_state")
        if not (st and st.strip() in ("8", "8.0")):
            axis_err = self._query(f"r axis{self.axis}.error")
            print(f"[ODrive ERROR] No se pudo entrar en lazo cerrado (estado {st}). Error eje: {axis_err}")
            return False

        print(f"[ODrive OK] Actuador armado en lazo cerrado (cero relativo en {self.pos_zero_turns:.4f} vueltas).")
        return True

    def set_closed_loop(self):
        """Pone el eje en lazo cerrado (AXIS_STATE_CLOSED_LOOP_CONTROL = 8)."""
        if self.is_mock:
            return
        self._send_cmd(f"w axis{self.axis}.requested_state 8")

    def set_idle(self):
        """Pone el eje en reposo sin par activo (AXIS_STATE_IDLE = 1)."""
        if self.ffb:
            try:
                self.ffb.fijar_torque(0.0)
            except Exception:
                pass
        if self.is_mock:
            self.sim_target_torque = 0.0
            return
        try:
            self._send_cmd(f"p {self.axis} {self.pos_zero_turns:.4f}")
            time.sleep(0.2)
        except Exception:
            pass
        self._send_cmd(f"w axis{self.axis}.requested_state 1")

    def set_control_mode(self, mode: int):
        """
        Configura el modo de control:
        1: Torque control
        2: Velocity control
        3: Position control
        """
        self.sim_control_mode = mode
        if self.is_mock:
            return
        self._send_cmd(f"w axis{self.axis}.controller.config.control_mode {mode}")

    # --- Comandos de consigna (Unidades angulares relativas en grados) ---

    def set_position_deg(self, pos_deg: float, vel_ff_deg_s: float = 0.0, torque_ff_Nm: Optional[float] = None):
        """
        Envía consigna de posición en grados relativa al cero inicial.
        Soporta inyección de par feedforward/carga (torque_ff_Nm) en la misma trama serie:
        'p <axis> <target_turns> <vel_ff_turns> <torque_ff_Nm>'
        Garantiza que el lazo FOC de posición (control_mode=3) permanezca 100% activo a 8 kHz
        mientras se superpone la carga resistiva auto-alineante de forma determinista.
        """
        eff_torque_ff = self.current_load_torque_Nm if torque_ff_Nm is None else torque_ff_Nm
        if self.is_mock:
            now = time.time()
            self._cmd_history.append((now, pos_deg, vel_ff_deg_s, eff_torque_ff))
            return

        target_turns = self.pos_zero_turns + (pos_deg / 360.0)
        cmd = f"p {self.axis} {target_turns:.4f} {vel_ff_deg_s/360.0:.4f} {eff_torque_ff:.4f}"
        self._send_cmd(cmd)

    def set_velocity_deg_s(self, vel_deg_s: float, torque_ff_Nm: float = 0.0):
        """Envía consigna de velocidad en deg/s."""
        vel_turns = vel_deg_s / 360.0
        if self.is_mock:
            now = time.time()
            self._cmd_history.append((now, None, vel_deg_s, torque_ff_Nm))
            return
        cmd = f"v {self.axis} {vel_turns:.4f} {torque_ff_Nm:.4f}"
        self._send_cmd(cmd)

    def set_torque_Nm(self, torque_Nm: float):
        """Envía consigna de par en N*m (vía USB HID FFB en hardware real)."""
        if self.is_mock:
            now = time.time()
            self._cmd_history.append((now, None, None, torque_Nm))
            return
        if self.ffb:
            self.ffb.fijar_torque(torque_Nm)
        else:
            cmd = f"c {self.axis} {torque_Nm:.4f}"
            self._send_cmd(cmd)

    def set_external_load_torque(self, tau_load_Nm: float):
        """
        Establece el par resistivo de carga.
        En hardware real se inyecta de forma unificada vía feedforward en la trama serie 'p',
        y de forma opcional/espejo vía USB HID FFB si está conectado.
        """
        self.current_load_torque_Nm = tau_load_Nm
        self.sim_tau_ext = tau_load_Nm
        if self.ffb:
            try:
                self.ffb.fijar_torque(tau_load_Nm)
            except Exception:
                pass

    def wait_settle(self, target_pos_deg: float = 0.0, vel_thresh_deg_s: float = 15.0, pos_thresh_deg: float = 4.0, timeout: float = 1.6) -> bool:
        """
        Envía consigna a target_pos_deg y espera activamente a que el motor
        alcance la posición y se estabilice en reposo (|vel| <= vel_thresh_deg_s).
        Aplica un integrador suave temporal (0.4) tras 0.3s si persiste error
        por fricción estática, asegurando centrado sub-grado exacto, y retorna
        con vel_integrator_gain = 0.0 para evitar windup en el ensayo subsiguiente.
        """
        self.set_position_deg(target_pos_deg)
        t_start = time.time()
        settled_count = 0
        integrator_applied = False

        while time.time() - t_start < timeout:
            pos, vel, _, _ = self.read_state()
            err = abs(pos - target_pos_deg)

            # Si tras 0.35s el error persiste por fricción seca, encender integrador suave para vencer el roce
            if not self.is_mock and not integrator_applied and (time.time() - t_start > 0.35) and (err > pos_thresh_deg):
                self._send_cmd(f"w axis{self.axis}.controller.config.vel_integrator_gain 0.5")
                integrator_applied = True

            if err <= pos_thresh_deg and abs(vel) <= vel_thresh_deg_s:
                settled_count += 1
                if settled_count >= 3: # 30 ms verificado en reposo
                    break
            else:
                settled_count = 0
            time.sleep(0.01)

        # Garantizar que el integrador quede en 0 para el ensayo dinámico (cero windup)
        if not self.is_mock and integrator_applied:
            self._send_cmd(f"w axis{self.axis}.controller.config.vel_integrator_gain 0.0")

        return settled_count >= 3

    # --- Lectura de telemetría física ---

    def read_state(self) -> Tuple[float, float, float, float]:
        """
        Retorna la tupla: (pos_deg_relativo, vel_deg_s, torque_est_Nm, iq_meas_A)
        """
        if self.is_mock:
            return self._step_mock()

        # En hardware real: comando ASCII 'f <axis>' retorna: '<pos_turns> <vel_turns_s>'
        line = self._query(f"f {self.axis}")
        pos_deg = 0.0
        vel_deg_s = 0.0
        if line:
            parts = line.strip().split()
            if len(parts) >= 2:
                try:
                    pos_turns = float(parts[0])
                    vel_turns = float(parts[1])
                    pos_deg = (pos_turns - self.pos_zero_turns) * 360.0
                    vel_deg_s = vel_turns * 360.0
                except ValueError:
                    pass

        # Lectura de corriente de cuadratura Iq
        iq_meas = 0.0
        iq_str = self._query(f"r axis{self.axis}.motor.current_control.Iq_measured")
        if iq_str:
            try:
                iq_meas = float(iq_str.strip())
            except ValueError:
                pass

        torque_est = iq_meas * self.Kt
        return pos_deg, vel_deg_s, torque_est, iq_meas

    def get_feedback(self) -> Tuple[float, float]:
        """Retorna (pos_deg, vel_deg_s)."""
        pos_deg, vel_deg_s, _, _ = self.read_state()
        return pos_deg, vel_deg_s

    def read_vbus(self) -> float:
        """Lee la tensión de bus DC."""
        if self.is_mock:
            return 24.0 + 0.1 * math.sin(time.time() * 2.0)
        line = self._query("r vbus_voltage")
        if line:
            try:
                return float(line.strip())
            except ValueError:
                pass
        return 24.0

    # --- Métodos auxiliares de bajo nivel y simulación ---

    def _send_cmd(self, cmd: str):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((cmd + "\n").encode("ascii"))
            except Exception:
                pass

    def _query(self, cmd: str) -> Optional[str]:
        if not self.ser or not self.ser.is_open:
            return None
        try:
            self.ser.write((cmd + "\n").encode("ascii"))
            line = self.ser.readline().decode("ascii", errors="replace")
            return line
        except Exception:
            return None

    def _step_mock(self) -> Tuple[float, float, float, float]:
        """
        Integrador físico de alta fidelidad para el modo mock:
        - Dinámica electromecánica no lineal de 2do orden.
        - Retardo puro de transporte T_d.
        - Fricción de Stribeck / Coulomb + amortiguamiento viscoso.
        - Saturación de par por tensión de bus V_bus y back-EMF (Ke * omega).
        - Cuantización de encoder (2880 cuentas/rev).
        """
        now = time.time()
        dt_total = now - self.sim_time
        if dt_total <= 0.0:
            dt_total = 0.002
        dt_total = min(dt_total, 1.0)
        self.sim_time = now

        # Sub-muestreo con paso de integración <= 2 ms para estabilidad física absoluta
        sub_dt = 0.002
        n_steps = max(1, int(math.ceil(dt_total / sub_dt)))
        actual_dt = dt_total / n_steps

        # Procesar consignas desfasadas por el retardo T_d
        effective_cmd = None
        while self._cmd_history and (now - self._cmd_history[0][0]) >= self.delay_true_s:
            effective_cmd = self._cmd_history.popleft()

        if effective_cmd:
            _, p_deg, v_deg_s, t_Nm = effective_cmd
            if p_deg is not None: self.sim_target_pos = math.radians(p_deg)
            if v_deg_s is not None: self.sim_target_vel = math.radians(v_deg_s)
            if t_Nm is not None: self.sim_target_torque = t_Nm

        tau_motor = 0.0
        for _ in range(n_steps):
            # Lazo de control FOC emulado (Kp, Ki de posición/velocidad + feedforward)
            if self.sim_control_mode == 3: # Posición
                pos_err = self.sim_target_pos - self.sim_pos_rad
                vel_target = 25.0 * pos_err # Kp_pos = 25
                vel_err = vel_target - self.sim_vel_rad_s
                if not hasattr(self, "sim_vel_integral"):
                    self.sim_vel_integral = 0.0
                self.sim_vel_integral += 1.5 * vel_err * actual_dt
                self.sim_vel_integral = max(-4.0, min(4.0, self.sim_vel_integral))
                # En ODrive v0.5.x, el par feedforward (torque_ff) entra aditivo al sumador de par
                tau_motor_dem = 0.35 * vel_err + self.sim_vel_integral + self.sim_target_torque
            elif self.sim_control_mode == 2: # Velocidad
                vel_err = self.sim_target_vel - self.sim_vel_rad_s
                tau_motor_dem = 0.40 * vel_err + self.sim_target_torque
            else: # Par directo
                tau_motor_dem = self.sim_target_torque

            # Límite por fuerza contra-electromotriz y tensión de bus (V_bus)
            omega = self.sim_vel_rad_s
            v_back_emf = self.Ke * abs(omega)
            v_avail = max(0.0, self.Vbus_nom - v_back_emf)
            iq_avail_max = min(self.Iq_max, v_avail / self.R)
            tau_max_avail = iq_avail_max * self.Kt

            tau_motor = max(-tau_max_avail, min(tau_max_avail, tau_motor_dem))
            self.sim_iq_meas = tau_motor / self.Kt

            # Fricción
            if abs(omega) > 0.01:
                tau_coulomb = self.tau_c_true * math.copysign(1.0, omega)
            else:
                tau_coulomb = 0.0
            tau_viscous = self.b_true * omega

            # Carga resistiva externa / autoalineamiento vehicular (SAT)
            if abs(self.sim_tau_ext) > 1e-4:
                # El par auto-alineante resiste la desviación angular respecto al centro (0 rad)
                if abs(self.sim_pos_rad) > 0.005:
                    tau_align = abs(self.sim_tau_ext) * math.copysign(1.0, self.sim_pos_rad)
                else:
                    tau_align = abs(self.sim_tau_ext) * math.copysign(1.0, omega) if abs(omega) > 0.01 else 0.0
            else:
                tau_align = 0.0

            # Ecuación diferencial: J * domega/dt = tau_motor - tau_viscous - tau_coulomb - tau_align
            if getattr(self, "locked_shaft", False):
                self.sim_vel_rad_s = 0.0
                self.sim_pos_rad = 0.0
            else:
                domega_dt = (tau_motor - tau_viscous - tau_coulomb - tau_align) / self.J_true
                self.sim_vel_rad_s += domega_dt * actual_dt
                self.sim_pos_rad += self.sim_vel_rad_s * actual_dt

        # Cuantización de encoder (2880 cuentas/rev = 0.125 deg)
        deg_raw = math.degrees(self.sim_pos_rad)
        counts = round(deg_raw / 0.125)
        pos_deg_quant = counts * 0.125
        vel_deg_s = math.degrees(self.sim_vel_rad_s)
        torque_meas = tau_motor

        return pos_deg_quant, vel_deg_s, torque_meas, self.sim_iq_meas
