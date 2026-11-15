"""
ffb_hid.py
==========
Torque de carga por la vía nativa de force feedback de ODrive-Wheel (USB HID, fuerza constante).
Basado en SDL2 (pysdl2).
Permite aplicar par directo o contra-par resistivo de carga para caracterización dinámica.
"""

import os
import json
from typing import Optional

AQUI = os.path.dirname(os.path.abspath(__file__))
RAIZ_WORKSPACE = os.path.dirname(os.path.dirname(AQUI))
RUTA_CALIBRACION_DEFECTO = os.path.join(RAIZ_WORKSPACE, "configs", "ffb_hid_calibracion.json")
NOMBRE_DISPOSITIVO_DEFECTO = b"ODrive-Wheel HID"


class FFBHid:
    def __init__(self, nombre: bytes = NOMBRE_DISPOSITIVO_DEFECTO,
                 calibracion_path: str = RUTA_CALIBRACION_DEFECTO,
                 requiere_calibracion: bool = False):
        import ctypes
        import sdl2
        self.sdl2 = sdl2
        self.ctypes = ctypes

        sdl2.SDL_SetHint(b"SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", b"1")
        if sdl2.SDL_Init(sdl2.SDL_INIT_JOYSTICK | sdl2.SDL_INIT_HAPTIC) != 0:
            raise RuntimeError(f"SDL_Init falló: {sdl2.SDL_GetError()}")

        self.j = None
        self.h = None
        for i in range(sdl2.SDL_NumJoysticks()):
            j = sdl2.SDL_JoystickOpen(i)
            if sdl2.SDL_JoystickName(j) == nombre:
                self.j = j
                break
            sdl2.SDL_JoystickClose(j)

        if self.j is None:
            raise RuntimeError(f"No se encontró el dispositivo HID: {nombre.decode()}")

        self.h = sdl2.SDL_HapticOpenFromJoystick(self.j)
        if not self.h or not (sdl2.SDL_HapticQuery(self.h) & sdl2.SDL_HAPTIC_CONSTANT):
            raise RuntimeError("El dispositivo HID no soporta efectos de fuerza constante (SDL_HAPTIC_CONSTANT).")

        if sdl2.SDL_HapticQuery(self.h) & sdl2.SDL_HAPTIC_GAIN:
            sdl2.SDL_HapticSetGain(self.h, 100)

        self.ef = sdl2.SDL_HapticEffect()
        self.ef.type = sdl2.SDL_HAPTIC_CONSTANT
        c = self.ef.constant
        c.type = sdl2.SDL_HAPTIC_CONSTANT
        c.direction.type = sdl2.SDL_HAPTIC_CARTESIAN
        c.direction.dir[0] = 1
        c.length = sdl2.SDL_HAPTIC_INFINITY
        c.level = 0

        self.id = sdl2.SDL_HapticNewEffect(self.h, self.ctypes.byref(self.ef))
        if self.id < 0:
            raise RuntimeError(f"SDL_HapticNewEffect falló: {sdl2.SDL_GetError()}")

        if sdl2.SDL_HapticRunEffect(self.h, self.id, 1) != 0:
            raise RuntimeError(f"SDL_HapticRunEffect falló: {sdl2.SDL_GetError()}")

        self.nivel = 0.0
        self.cal = None
        self.nm_por_nivel = 2.0  # Valor nominal por defecto

        if calibracion_path and os.path.isfile(calibracion_path):
            with open(calibracion_path, "r", encoding="utf-8") as fp:
                self.cal = json.load(fp)
                self.nm_por_nivel = float(self.cal.get("nm_volante_por_nivel", 2.0))
        elif requiere_calibracion:
            self.cerrar()
            raise RuntimeError(f"Archivo de calibración no encontrado: {calibracion_path}")

    def fijar_nivel(self, nivel: float):
        """Fija el nivel HID entre -1.0 y 1.0 (adimensional)."""
        nivel = max(-1.0, min(1.0, float(nivel)))
        if abs(nivel - self.nivel) < 1e-4:
            return
        self.ef.constant.level = int(round(nivel * 32767))
        self.sdl2.SDL_HapticUpdateEffect(self.h, self.id, self.ctypes.byref(self.ef))
        self.nivel = nivel

    def fijar_torque(self, tau_nm: float) -> float:
        """
        Fija el torque del actuador en N*m.
        Convención: tau > 0 empuja hacia la derecha (theta > 0 / vel > 0).
        Alinea el signo físico del par con el signo de posición del encoder.
        """
        signo = -1 if (self.cal and self.cal.get("signo_placa") == -1) else 1
        nivel = signo * (tau_nm / self.nm_por_nivel)
        self.fijar_nivel(nivel)
        return self.nivel * self.nm_por_nivel

    def cerrar(self):
        """Libera de forma segura todos los recursos haptics de SDL2."""
        s = self.sdl2
        try:
            if self.h:
                self.fijar_nivel(0.0)
                s.SDL_HapticStopAll(self.h)
                s.SDL_HapticDestroyEffect(self.h, self.id)
                s.SDL_HapticClose(self.h)
            if self.j:
                s.SDL_JoystickClose(self.j)
        finally:
            self.h = None
            self.j = None
