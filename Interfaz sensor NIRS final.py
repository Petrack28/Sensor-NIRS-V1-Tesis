"""
PPG/NIRS Dual Sensor — ADPD1080 + ESP32 (v4.3)
===============================================
CAMBIOS v4.3:
  - Gráfica reemplazada por Canvas nativo de Tkinter (sin matplotlib).
    Los ejes/grid se dibujan UNA VEZ. Solo las líneas de señal se
    actualizan cada frame → sin delay, actualización real en tiempo real.
  - Configuración persistente (bug fix v4.2 incluido).
  - Sin SNR (quitado para liberar recursos).
  - Chop Mode y VBias por sensor.
  - Sin time.sleep() en el hilo principal.
  - open_serial() no bloquea la UI.
"""

import serial
import serial.tools.list_ports
import threading
import queue
import time
import math
import numpy as np

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog

import os
import csv

# ──────────────────────────────────────────────────────────
# PARÁMETROS GENERALES
# ──────────────────────────────────────────────────────────
BAUD_RATE        = 115200
WINDOW_SECONDS   = 10.0
MAX_PULSES       = 255
ADC_MAX          = 65535
ADC_SAT_THRESH   = 63000
PLOT_INTERVAL_MS = 30       # ~33 FPS
WARMUP_S         = 3.0       # segundos de warm-up antes de iniciar grabación
MAX_HIST_PTS     = 90_000

# Dimensiones del canvas de cada gráfica
CANVAS_W = 860
CANVAS_H = 180
PAD_L    = 58   # espacio para eje Y
PAD_R    = 12
PAD_T    = 22
PAD_B    = 32   # espacio para eje X
PLOT_W   = CANVAS_W - PAD_L - PAD_R
PLOT_H   = CANVAS_H - PAD_T - PAD_B

# ──────────────────────────────────────────────────────────
# CONSTANTES LED
# ──────────────────────────────────────────────────────────
ILED_FINE_DEFAULT  = 0xC
FINE_FACTOR        = 0.74 + 0.022 * ILED_FINE_DEFAULT
MIN_REAL_MA_SCALE0 = (50.3 + 19.8 * 0)  * FINE_FACTOR * 0.1
MAX_REAL_MA_SCALE0 = (50.3 + 19.8 * 15) * FINE_FACTOR * 0.1
MIN_REAL_MA_SCALE1 = (50.3 + 19.8 * 0)  * FINE_FACTOR * 1.0
MAX_REAL_MA_SCALE1 = (50.3 + 19.8 * 15) * FINE_FACTOR * 1.0
REGISTER_RESERVED_BIT12 = (1 << 12)

# ──────────────────────────────────────────────────────────
# MAPEO AFE
# ──────────────────────────────────────────────────────────
TIA_RF_MAP = {"200 kΩ": 0x0, "100 kΩ": 0x1, "50 kΩ": 0x2, "25 kΩ": 0x3}
RINT_MAP   = {"400 kΩ (max SNR)": 0x000, "200 kΩ": 0x100, "100 kΩ": 0x200}
AFE_BASE_CONFIG = 0x1C38

FSAMPLE_PRESETS = {
    "1000 Hz": 0x0008, " 500 Hz": 0x0010, " 400 Hz": 0x0014,
    " 200 Hz": 0x0028, " 100 Hz": 0x0050, "  50 Hz": 0x00A0,
    "  25 Hz": 0x0140, "  10 Hz": 0x0320,
}
DEFAULT_FSAMPLE_KEY = " 100 Hz"

AVG_PRESETS = {
    "Sin promediado (1x)": 0x0000, "2x (×2 muestras)": 0x0110,
    "4x (×4 muestras)": 0x0220,   "8x (×8 muestras)": 0x0330,
    "16x (×16 muestras)": 0x0440, "32x (×32 muestras)": 0x0550,
}
DEFAULT_AVG_KEY = "Sin promediado (1x)"

SNR_TABLE = [
    (200, 400,  3.3, 0.82,  2.2, 72.1),
    (100, 200,  3.3, 1.26,  7.8, 68.4),
    ( 50, 100,  3.3, 1.85, 18.8, 65.0),
    (100, 400,  6.8, 1.38,  4.3, 73.9),
    ( 50, 200,  6.8, 2.10, 15.3, 70.2),
    ( 50, 400, 13.2, 2.70,  8.9, 73.8),
]

# ──────────────────────────────────────────────────────────
# FUNCIONES AUXILIARES
# ──────────────────────────────────────────────────────────
def get_coarse_for_current(target_ma, scale):
    sf = 0.1 + 0.9 * scale
    best_c, best_e = 0, float('inf')
    for c in range(16):
        e = abs((50.3 + 19.8 * c) * FINE_FACTOR * sf - target_ma)
        if e < best_e: best_e, best_c = e, c
    return best_c

def convert_ma_to_reg(ma):
    if ma <= 0: return 0
    scale = 0 if ma <= MAX_REAL_MA_SCALE0 else 1
    return REGISTER_RESERVED_BIT12 | (scale << 13) | get_coarse_for_current(ma, scale)

def get_actual_current_ma(reg):
    scale  = (reg >> 13) & 0x1
    coarse = reg & 0xF
    return round((50.3 + 19.8 * coarse) * FINE_FACTOR * (0.1 + 0.9 * scale), 1)

def build_tia_reg(rf_key, rint_key):
    return AFE_BASE_CONFIG | RINT_MAP.get(rint_key, 0) | TIA_RF_MAP.get(rf_key, 0)

def fsample_reg_to_hz(reg):
    return 32000.0 / (reg * 4) if reg else 0.0

def calc_bits_efectivos(pa, pb):
    ba = 14.0 + (0.5 * math.log2(pa) if pa > 1 else 0)
    bb = 14.0 + (0.5 * math.log2(pb) if pb > 1 else 0)
    return ba, bb

def calc_tasa_salida(fsample_key, avg_key, pulse_a=8, pulse_b=8):
    """
    Tasa de salida real = min(fSAMPLE, fMAX) / avg_factor
    fMAX depende de los pulsos configurados.
    """
    fs_hz = fsample_reg_to_hz(FSAMPLE_PRESETS.get(fsample_key, 0x0050))
    fmax  = calc_max_fsample_hz(pulse_a, pulse_b)
    base  = min(fs_hz, fmax)
    avg_f = 2 ** ((AVG_PRESETS.get(avg_key, 0x0000) >> 4) & 0x7)
    return base / max(avg_f, 1)

def calc_max_fsample_hz(pa, pb, period_us=24):
    total = 25 + pa*period_us + 68 + 25 + pb*period_us + 20 + 222
    return 1_000_000.0 / total if total > 0 else 0

# ──────────────────────────────────────────────────────────
# MÉTRICAS DE CALIDAD DE SEÑAL NIRS
# ──────────────────────────────────────────────────────────
def calc_cv(signal):
    """
    Coeficiente de Variación (CV).
    CV = (std / mean) * 100%
    Umbral: CV < 5% = señal estable.
    ADVERTENCIA: no distingue latido cardíaco de artefacto de movimiento.
    """
    if len(signal) < 10:
        return None
    a = np.array(signal, dtype=float)
    m = np.mean(a)
    if m == 0:
        return None
    return (np.std(a) / m) * 100.0

def calc_sci(ir_signal, red_signal, fs=None, t_arr=None):
    """
    SCI (Scalp Coupling Index) — Pollonini et al. 2014.

    Implementación CORRECTA para NIRS muscular:
      1. Filtra IR y Rojo en la banda cardíaca (0.5–3 Hz)
      2. Calcula correlación de Pearson SOLO de esa componente filtrada

    Sin filtro, el DC dominante (ratio IR/Rojo ~7x) hace que la correlación
    salga baja aunque el latido sea perfectamente visible.
    fs puede omitirse — se estima desde t_arr o desde len/duración.
    Rango: -1 a 1.   Umbral literatura: SCI >= 0.75 señal aceptable.
    """
    from scipy.signal import butter, filtfilt
    n = len(ir_signal)
    if n < 20 or len(red_signal) < 20:
        return None
    if fs is None or fs <= 0:
        if t_arr is not None and len(t_arr) >= 2:
            dur = t_arr[-1] - t_arr[0]
            fs  = n / dur if dur > 0 else 100.0
        else:
            fs = 100.0
    N_MIN = int(fs * 4)
    if n < N_MIN:
        return None
    a = np.array(ir_signal,  dtype=float)
    b = np.array(red_signal, dtype=float)
    try:
        nyq  = fs / 2.0
        low  = 0.5 / nyq
        high = min(3.0 / nyq, 0.99)
        if low >= high:   # fs demasiado baja para pasa-banda, usar solo paso-alto
            raise ValueError
        bb, ba = butter(2, [low, high], btype='band')
        a = filtfilt(bb, ba, a)
        b = filtfilt(bb, ba, b)
    except Exception:
        try:
            nyq = fs / 2.0
            low = 0.5 / nyq
            if low < 0.99:
                bb, ba = butter(2, low, btype='high')
                a = filtfilt(bb, ba, a)
                b = filtfilt(bb, ba, b)
        except Exception:
            return None
    num = np.dot(a, b)
    den = np.sqrt(np.dot(a, a) * np.dot(b, b))
    if den == 0:
        return None
    return float(np.clip(num / den, -1.0, 1.0))

def sci_label(sci):
    """Etiqueta cualitativa del SCI según umbrales de literatura."""
    if sci is None:     return "--",     "gray"
    if sci >= 0.80:     return "BUENA",  "green"
    if sci >= 0.75:     return "MEDIA",  "orange"
    return              "BAJA",          "red"

def cv_label(cv):
    """Etiqueta cualitativa del CV."""
    if cv is None:   return "--",        "gray"
    if cv < 2.0:     return "MUY ESTABLE","green"
    if cv < 5.0:     return "ESTABLE",   "blue"
    if cv < 15.0:    return "VARIABLE",  "orange"
    return           "INESTABLE",        "red"


# ══════════════════════════════════════════════════════════
# CANVAS PLOT — gráfica de dos canales sin matplotlib
# ══════════════════════════════════════════════════════════
class DualChannelPlot:
    """
    Gráfica de dos canales (IR + Rojo) dibujada con Canvas nativo Tkinter.
    Los ejes y el grid se dibujan una sola vez en _draw_static().
    Solo las líneas de señal se recrean en cada update().
    """
    N_GRIDX = 5   # líneas verticales de grid
    N_GRIDY = 4   # líneas horizontales de grid

    def __init__(self, parent, title, color_ir, color_red, label_ir, label_red):
        self.color_ir  = color_ir
        self.color_red = color_red
        self.y_min = 0
        self.y_max = ADC_MAX

        frame = ttk.LabelFrame(parent, text=title)
        frame.pack(fill=tk.X, padx=8, pady=4)

        self.cv = tk.Canvas(frame, width=CANVAS_W, height=CANVAS_H,
                            bg="#1e1e2e", highlightthickness=0)
        self.cv.pack(padx=4, pady=4)

        # IDs de los objetos dinámicos
        self._id_ir  = None
        self._id_red = None
        self._id_sat = None   # línea de saturación (se dibuja en static)

        # Labels de estado (saturación)
        info = ttk.Frame(frame)
        info.pack(fill=tk.X, padx=8, pady=(0, 4))
        self.sat_lbl = ttk.Label(info, text="", foreground="orange",
                                 font=("", 9, "bold"))
        self.sat_lbl.pack(side=tk.LEFT)
        # Leyenda
        tk.Label(info, text="■", fg=color_ir).pack(side=tk.RIGHT, padx=2)
        tk.Label(info, text=label_ir).pack(side=tk.RIGHT)
        tk.Label(info, text="■", fg=color_red).pack(side=tk.RIGHT, padx=(8,2))
        tk.Label(info, text=label_red).pack(side=tk.RIGHT)

        self._draw_static()

    # ── Coordenadas ───────────────────────────────────────
    def _px(self, t, t_min, t_max):
        """Tiempo → pixel X."""
        if t_max == t_min: return PAD_L
        return PAD_L + (t - t_min) / (t_max - t_min) * PLOT_W

    def _py(self, val):
        """Valor ADC → pixel Y."""
        val = max(self.y_min, min(self.y_max, val))
        return PAD_T + (1 - (val - self.y_min) / (self.y_max - self.y_min)) * PLOT_H

    # ── Estático (ejes + grid) ────────────────────────────
    def _draw_static(self):
        cv = self.cv
        cv.delete("static")

        # Fondo del área de plot
        cv.create_rectangle(PAD_L, PAD_T, PAD_L+PLOT_W, PAD_T+PLOT_H,
                            outline="#334155", fill="#1e1e2e", tags="static")

        # Grid horizontal
        for i in range(self.N_GRIDY + 1):
            y = PAD_T + i * PLOT_H // self.N_GRIDY
            cv.create_line(PAD_L, y, PAD_L+PLOT_W, y,
                          fill="#334155", dash=(4, 4), tags="static")
            val = self.y_max - i * (self.y_max - self.y_min) / self.N_GRIDY
            cv.create_text(PAD_L - 4, y, text=f"{val:.0f}",
                          anchor=tk.E, fill="#94a3b8", font=("Courier", 7), tags="static")

        # Grid vertical
        for i in range(self.N_GRIDX + 1):
            x = PAD_L + i * PLOT_W // self.N_GRIDX
            cv.create_line(x, PAD_T, x, PAD_T+PLOT_H,
                          fill="#334155", dash=(4, 4), tags="static")

        # Ejes
        cv.create_line(PAD_L, PAD_T, PAD_L, PAD_T+PLOT_H,
                      fill="#64748b", width=1, tags="static")
        cv.create_line(PAD_L, PAD_T+PLOT_H, PAD_L+PLOT_W, PAD_T+PLOT_H,
                      fill="#64748b", width=1, tags="static")

        # Línea de saturación (fija en ADC_SAT_THRESH)
        y_sat = self._py(ADC_SAT_THRESH)
        cv.create_line(PAD_L, y_sat, PAD_L+PLOT_W, y_sat,
                      fill="#f59e0b", dash=(6, 3), width=1, tags="static")
        cv.create_text(PAD_L+PLOT_W-2, y_sat-3, text="SAT",
                      anchor=tk.E, fill="#f59e0b", font=("Courier", 7), tags="static")

        # Label eje Y
        cv.create_text(10, PAD_T + PLOT_H//2, text="ADC",
                      angle=90, fill="#64748b", font=("Courier", 8), tags="static")

    def _redraw_y_labels(self):
        """Redibuja solo los labels del eje Y cuando cambia la escala."""
        self.cv.delete("ylabel")
        for i in range(self.N_GRIDY + 1):
            y = PAD_T + i * PLOT_H // self.N_GRIDY
            val = self.y_max - i * (self.y_max - self.y_min) / self.N_GRIDY
            self.cv.create_text(PAD_L - 4, y, text=f"{val:.0f}",
                               anchor=tk.E, fill="#94a3b8",
                               font=("Courier", 7), tags="ylabel")

    # ── X axis labels (tiempo) ────────────────────────────
    def _update_x_labels(self, t_min, t_max):
        self.cv.delete("xlabel")
        for i in range(self.N_GRIDX + 1):
            x = PAD_L + i * PLOT_W // self.N_GRIDX
            t  = t_min + i * (t_max - t_min) / self.N_GRIDX
            self.cv.create_text(x, PAD_T+PLOT_H+12, text=f"{t:.1f}s",
                               fill="#94a3b8", font=("Courier", 7), tags="xlabel")

    # ── Update (llamado cada PLOT_INTERVAL_MS) ────────────
    def update(self, t_arr, ir_arr, red_arr, t_min, t_max):
        """
        t_arr, ir_arr, red_arr: listas Python con los datos de la ventana.
        Solo se recrean los objetos de línea — O(n) pixels, sin redibujar ejes.
        """
        if not t_arr:
            return

        # Auto-escala Y suave
        all_v = ir_arr + red_arr
        if all_v:
            mn, mx = min(all_v), max(all_v)
            rng = mx - mn if mx != mn else 1000
            new_min = max(0, mn - rng * 0.08)
            new_max = min(ADC_MAX, mx + rng * 0.08)
            if abs(new_min - self.y_min) > rng * 0.05 or \
               abs(new_max - self.y_max) > rng * 0.05:
                self.y_min = new_min
                self.y_max = new_max
                self._draw_static()   # redibuja estático solo cuando cambia escala

        # Actualizar labels X
        self._update_x_labels(t_min, t_max)

        # Construir coordenadas de puntos
        def make_coords(vals):
            coords = []
            for t, v in zip(t_arr, vals):
                coords.append(self._px(t, t_min, t_max))
                coords.append(self._py(v))
            return coords

        coords_ir  = make_coords(ir_arr)
        coords_red = make_coords(red_arr)

        # Borrar y recrear solo las líneas de señal
        if self._id_ir:  self.cv.delete(self._id_ir)
        if self._id_red: self.cv.delete(self._id_red)

        if len(coords_ir) >= 4:
            self._id_ir = self.cv.create_line(
                coords_ir,  fill=self.color_ir,  width=1, smooth=False)
        if len(coords_red) >= 4:
            self._id_red = self.cv.create_line(
                coords_red, fill=self.color_red, width=1, smooth=False)

        # Saturación
        msgs = []
        if ir_arr[-10:]  and max(ir_arr[-10:])  >= ADC_SAT_THRESH: msgs.append("⚠ IR SAT")
        if red_arr[-10:] and max(red_arr[-10:]) >= ADC_SAT_THRESH: msgs.append("⚠ ROJO SAT")
        self.sat_lbl.config(text=" | ".join(msgs))

    def clear(self):
        if self._id_ir:  self.cv.delete(self._id_ir);  self._id_ir  = None
        if self._id_red: self.cv.delete(self._id_red); self._id_red = None
        self.cv.delete("xlabel")
        self.y_min = 0; self.y_max = ADC_MAX
        self._draw_static()


# ══════════════════════════════════════════════════════════
# CONFIGURACIÓN DE SENSOR
# ══════════════════════════════════════════════════════════
class SensorConfig:
    def __init__(self, suffix, label):
        self.suffix      = suffix
        self.label       = label
        self.ir_ma       = 200.0
        self.red_ma      = 200.0
        self.reg_ir      = convert_ma_to_reg(200.0)
        self.reg_red     = convert_ma_to_reg(200.0)
        self.rf_a_key    = "100 kΩ"
        self.rint_a_key  = "400 kΩ (max SNR)"
        self.rf_b_key    = "100 kΩ"
        self.rint_b_key  = "400 kΩ (max SNR)"
        self.pulse_a     = 8
        self.pulse_b     = 8
        self.fsample_key = DEFAULT_FSAMPLE_KEY
        self.avg_key     = DEFAULT_AVG_KEY
        self.chop_mode   = False   # Chop Mode OFF — no implementable de forma estable
        self.vbias       = False   # VBias OFF por default


# ══════════════════════════════════════════════════════════
# APP PRINCIPAL
# ══════════════════════════════════════════════════════════
class PPGDualApp:
    def __init__(self, root):
        self.root = root
        self.root.title("PPG/NIRS Dual Sensor — ADPD1080 × 2 + ESP32 (v4.3-BT)")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.serial_port         = None
        self.read_thread         = None
        self.read_thread_running = False
        self.paused              = False
        self.start_time          = None
        self.mode                = "realtime"
        self.acq_duration        = None
        self.save_path           = None

        self.cmd_queue  = queue.Queue()
        self.data_queue = queue.Queue()
        self._drain_running = True
        self._clock_running = False
        self._warming_up   = False

        self.s1 = SensorConfig("_S1", "Sensor 1")
        self.s2 = SensorConfig("_S2", "Sensor 2")

        self.times     = []
        self.ir1_data  = []; self.red1_data = []
        self.ir2_data  = []; self.red2_data = []

        self._build_ui()
        self.update_ports()
        threading.Thread(target=self._serial_write_loop, daemon=True).start()
        threading.Thread(target=self._serial_drain_loop,  daemon=True).start()
        self.update_plot()

    # ──────────────────────────────────────────────────────
    # HILO ESCRITOR SERIAL
    # ──────────────────────────────────────────────────────
    def _serial_write_loop(self):
        while True:
            cmd = self.cmd_queue.get()
            if cmd is None: break
            if self.serial_port and self.serial_port.is_open:
                try: self.serial_port.write(cmd.encode())
                except Exception: pass
            time.sleep(0.04)

    def _serial_drain_loop(self):
        """
        Drena continuamente el buffer de entrada del puerto serial/BT.
        Necesario porque el ESP32 envía datos a 250 Hz aunque no estemos
        graficando — si nadie lee el buffer, el stack BT se llena y corta
        la conexión, dejando de responder a comandos.
        Si read_thread_running está activo, el hilo lector ya consume los
        datos y este hilo solo hace flush como respaldo.
        """
        while self._drain_running:
            if self.serial_port and self.serial_port.is_open                and not self.read_thread_running:
                try:
                    # Leer y descartar hasta vaciar el buffer
                    waiting = self.serial_port.in_waiting
                    if waiting > 0:
                        self.serial_port.read(waiting)
                except Exception:
                    pass
            time.sleep(0.05)   # cada 50ms es suficiente — a 250Hz acumula ~12 líneas

    def _send_raw(self, cmd):
        if not (self.serial_port and self.serial_port.is_open):
            # Mostrar error solo si el usuario intentó algo manualmente
            # (no durante on_play que maneja su propia lógica)
            self.status_lbl.config(
                text="⚠ Conecta primero antes de configurar", foreground="orange")
            self.root.after(3000, lambda: self.status_lbl.config(
                text="Desconectado", foreground="red")
                if not (self.serial_port and self.serial_port.is_open) else None)
            return False
        self.cmd_queue.put(cmd)
        return True

    # ──────────────────────────────────────────────────────
    # BUILD UI
    # ──────────────────────────────────────────────────────
    def _build_ui(self):
        nb = ttk.Notebook(self.root)
        nb.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        tab_main = ttk.Frame(nb); tab_s1 = ttk.Frame(nb)
        tab_s2   = ttk.Frame(nb); tab_info = ttk.Frame(nb)
        nb.add(tab_main, text="  Principal  ")
        nb.add(tab_s1,   text="  Sensor 1  ")
        nb.add(tab_s2,   text="  Sensor 2  ")
        nb.add(tab_info, text="  Referencia  ")
        self._build_tab_main(tab_main)
        self._build_tab_sensor(tab_s1, self.s1)
        self._build_tab_sensor(tab_s2, self.s2)
        self._build_tab_info(tab_info)

    # ── Pestaña Principal ─────────────────────────────────
    def _build_tab_main(self, parent):
        # Conexión serial
        conn = ttk.LabelFrame(parent, text="Conexión — Bluetooth SPP (COMxx asignado al parear NIRS_ESP32)")
        conn.pack(fill=tk.X, padx=8, pady=4)
        ttk.Label(conn, text="Puerto:").pack(side=tk.LEFT, padx=4)
        self.port_var   = tk.StringVar()
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var,
                                       width=14, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)
        ttk.Label(conn, text="← Puerto COM asignado al BT",
                  foreground="gray", font=("", 8)).pack(side=tk.LEFT, padx=2)
        self.connect_btn = ttk.Button(conn, text="Conectar", command=self.open_serial)
        self.connect_btn.pack(side=tk.LEFT, padx=4)
        self.disconnect_btn = ttk.Button(conn, text="Desconectar",
                                         command=self.manual_disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=2)
        ttk.Button(conn, text="↻", command=self.update_ports).pack(side=tk.LEFT)
        self.status_lbl = ttk.Label(conn, text="Desconectado",
                                    foreground="red", font=("", 9, "bold"))
        self.status_lbl.pack(side=tk.LEFT, padx=12)

        # Botones adquisición
        acq = ttk.Frame(parent); acq.pack(fill=tk.X, padx=8, pady=3)
        ttk.Button(acq, text="Tiempo Real",
                   command=self.set_realtime_mode).pack(side=tk.LEFT, padx=4)
        ttk.Button(acq, text="Temporizado (CSV)",
                   command=self.configure_timed_mode).pack(side=tk.LEFT, padx=4)
        self.play_btn  = ttk.Button(acq, text="▶ Play",    command=self.on_play,  state=tk.DISABLED)
        self.pause_btn = ttk.Button(acq, text="⏸ Pausa",   command=self.on_pause, state=tk.DISABLED)
        self.stop_btn  = ttk.Button(acq, text="⏹ Detener", command=self.on_stop,  state=tk.DISABLED)
        self.play_btn.pack(side=tk.LEFT, padx=10)
        self.pause_btn.pack(side=tk.LEFT, padx=4)
        self.stop_btn.pack(side=tk.LEFT, padx=4)

        # Paneles config activa
        cfg_outer = ttk.Frame(parent); cfg_outer.pack(fill=tk.X, padx=8, pady=2)
        self.cfg_panel_s1 = self._build_cfg_panel(
            cfg_outer, "Config activa — Sensor 1", "purple", "darkred")
        self.cfg_panel_s1.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 4))
        self.cfg_panel_s2 = self._build_cfg_panel(
            cfg_outer, "Config activa — Sensor 2", "royalblue", "darkorange")
        self.cfg_panel_s2.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self._update_cfg_panel(self.s1)
        self._update_cfg_panel(self.s2)

        # Panel de calidad de señal (SCI + CV) — ventana deslizante 10 s
        sq = ttk.LabelFrame(parent, text="Calidad de Señal (ventana 10 s)")
        sq.pack(fill=tk.X, padx=8, pady=2)
        for col, text in enumerate(["Sensor", "SCI IR↔Rojo", "Calidad SCI",
                                    "CV IR (%)", "CV Rojo (%)", "Estado CV"]):
            ttk.Label(sq, text=text, font=("", 8, "bold"),
                      foreground="gray").grid(row=0, column=col, padx=8, pady=1, sticky=tk.W)

        # Sensor 1
        ttk.Label(sq, text="Sensor 1", font=("", 9, "bold"),
                  foreground="#a855f7").grid(row=1, column=0, padx=8, sticky=tk.W)
        self.sci1_val  = ttk.Label(sq, text="--",  width=7, foreground="gray")
        self.sci1_val.grid(row=1, column=1, padx=8)
        self.sci1_lbl  = ttk.Label(sq, text="--",  width=10, font=("", 9, "bold"))
        self.sci1_lbl.grid(row=1, column=2, padx=8)
        self.cv1_ir    = ttk.Label(sq, text="--",  width=7, foreground="gray")
        self.cv1_ir.grid(row=1, column=3, padx=8)
        self.cv1_red   = ttk.Label(sq, text="--",  width=7, foreground="gray")
        self.cv1_red.grid(row=1, column=4, padx=8)
        self.cv1_lbl   = ttk.Label(sq, text="--",  width=12, font=("", 9, "bold"))
        self.cv1_lbl.grid(row=1, column=5, padx=8)

        # Sensor 2
        ttk.Label(sq, text="Sensor 2", font=("", 9, "bold"),
                  foreground="#3b82f6").grid(row=2, column=0, padx=8, sticky=tk.W)
        self.sci2_val  = ttk.Label(sq, text="--",  width=7, foreground="gray")
        self.sci2_val.grid(row=2, column=1, padx=8)
        self.sci2_lbl  = ttk.Label(sq, text="--",  width=10, font=("", 9, "bold"))
        self.sci2_lbl.grid(row=2, column=2, padx=8)
        self.cv2_ir    = ttk.Label(sq, text="--",  width=7, foreground="gray")
        self.cv2_ir.grid(row=2, column=3, padx=8)
        self.cv2_red   = ttk.Label(sq, text="--",  width=7, foreground="gray")
        self.cv2_red.grid(row=2, column=4, padx=8)
        self.cv2_lbl   = ttk.Label(sq, text="--",  width=12, font=("", 9, "bold"))
        self.cv2_lbl.grid(row=2, column=5, padx=8)

        ttk.Label(sq, text="SCI ≥ 0.80 = Buena  |  SCI 0.75–0.80 = Media  |  SCI < 0.75 = Baja  "
                           "||  CV < 5% = Estable  |  CV < 15% = Variable  |  CV ≥ 15% = Inestable",
                  foreground="gray", font=("", 7)).grid(
                  row=3, column=0, columnspan=6, padx=8, pady=(0, 4), sticky=tk.W)

        # Reloj de tiempo transcurrido — se actualiza cada segundo
        clock_frame = ttk.LabelFrame(sq, text="Tiempo transcurrido")
        clock_frame.grid(row=0, column=6, rowspan=4, padx=12, pady=4, sticky=tk.NSEW)
        self.clock_lbl = tk.Label(clock_frame, text="0 s",
                                  font=("Courier", 26, "bold"),
                                  foreground="#22c55e", background="#1e1e2e",
                                  width=8)
        self.clock_lbl.pack(expand=True, padx=10, pady=8)

        # Gráficas Canvas nativo
        self.plot1 = DualChannelPlot(parent,
            title="Sensor 1 — IR & Rojo",
            color_ir="#a855f7", color_red="#ef4444",
            label_ir="IR S1", label_red="Rojo S1")
        self.plot2 = DualChannelPlot(parent,
            title="Sensor 2 — IR & Rojo",
            color_ir="#3b82f6", color_red="#f97316",
            label_ir="IR S2", label_red="Rojo S2")

    def _build_cfg_panel(self, parent, title, color_ir, color_red):
        f = ttk.LabelFrame(parent, text=title)
        ttk.Label(f, text="Frec. salida:").grid(row=0, column=0, padx=6, pady=2, sticky=tk.W)
        lbl_tasa = ttk.Label(f, text="-- Hz", foreground="blue",
                             font=("", 9, "bold"), width=9)
        lbl_tasa.grid(row=0, column=1, padx=4, sticky=tk.W)
        ttk.Label(f, text="fSAMPLE:").grid(row=0, column=2, padx=6, sticky=tk.W)
        lbl_fs  = ttk.Label(f, text="-- Hz", foreground="gray", width=9)
        lbl_fs.grid(row=0, column=3, padx=4, sticky=tk.W)
        ttk.Label(f, text="Avg:").grid(row=0, column=4, padx=6, sticky=tk.W)
        lbl_avg = ttk.Label(f, text="--", foreground="gray", width=5)
        lbl_avg.grid(row=0, column=5, padx=4, sticky=tk.W)
        ttk.Label(f, text="Bits IR:").grid(row=1, column=0, padx=6, pady=2, sticky=tk.W)
        lbl_ba  = ttk.Label(f, text="-- bits", foreground=color_ir,
                            font=("", 9, "bold"), width=9)
        lbl_ba.grid(row=1, column=1, padx=4, sticky=tk.W)
        ttk.Label(f, text="Bits Rojo:").grid(row=1, column=2, padx=6, sticky=tk.W)
        lbl_bb  = ttk.Label(f, text="-- bits", foreground=color_red,
                            font=("", 9, "bold"), width=9)
        lbl_bb.grid(row=1, column=3, padx=4, sticky=tk.W)
        f._lbl_tasa = lbl_tasa; f._lbl_fs = lbl_fs; f._lbl_avg = lbl_avg
        f._lbl_ba = lbl_ba; f._lbl_bb = lbl_bb
        f._color_ir = color_ir; f._color_red = color_red
        return f

    # ── Pestaña Sensor ────────────────────────────────────
    def _build_tab_sensor(self, parent, sc):
        # Scroll
        c  = tk.Canvas(parent)
        sb = ttk.Scrollbar(parent, orient="vertical", command=c.yview)
        sf = ttk.Frame(c)
        sf.bind("<Configure>", lambda e: c.configure(scrollregion=c.bbox("all")))
        c.create_window((0, 0), window=sf, anchor="nw")
        c.configure(yscrollcommand=sb.set)
        c.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        p = sf

        ttk.Label(p, text=f"Control independiente de {sc.label}  (sufijo: {sc.suffix})",
                  font=("", 9, "bold"), foreground="gray").pack(anchor=tk.W, padx=10, pady=4)

        # Corriente LED
        cur = ttk.LabelFrame(p, text="Corriente LED"); cur.pack(fill=tk.X, padx=10, pady=5)
        for ri, (ln, ae, ar, areg) in enumerate([
            ("IR (Slot A)",   "ir_entry",  "ir_real_lbl",  "ir_reg_lbl"),
            ("Rojo (Slot B)", "red_entry", "red_real_lbl", "red_reg_lbl"),
        ]):
            ttk.Label(cur, text=f"{ln}:", width=14).grid(row=ri, column=0, padx=6, pady=4, sticky=tk.W)
            ent = ttk.Entry(cur, width=10)
            ent.insert(0, str(sc.ir_ma if ri == 0 else sc.red_ma))
            ent.grid(row=ri, column=1, padx=6)
            setattr(sc, ae, ent)
            lk = 'IR' if ri == 0 else 'Rojo'
            fb = ttk.Frame(cur); fb.grid(row=ri, column=2)
            ttk.Button(fb, text="Aplicar",
                       command=lambda l=lk, s=sc: self.apply_current(s, l)).pack(side=tk.LEFT, padx=2)
            ttk.Button(fb, text="OFF",
                       command=lambda l=lk, s=sc: self.send_led_cmd(s, l, 0)).pack(side=tk.LEFT)
            lr   = ttk.Label(cur, text=f"{get_actual_current_ma(sc.reg_ir if ri==0 else sc.reg_red)} mA",
                             foreground="purple", width=12)
            lr.grid(row=ri, column=3, padx=6)
            setattr(sc, ar, lr)
            lreg = ttk.Label(cur, text=hex(sc.reg_ir if ri==0 else sc.reg_red),
                             foreground="gray", width=10)
            lreg.grid(row=ri, column=4, padx=6)
            setattr(sc, areg, lreg)
        ttk.Label(cur, text=f"Rango: {MIN_REAL_MA_SCALE0:.0f}–{MAX_REAL_MA_SCALE0:.0f} mA (10%) | "
                            f"{MIN_REAL_MA_SCALE1:.0f}–{MAX_REAL_MA_SCALE1:.0f} mA (100%)",
                  foreground="gray").grid(row=2, column=0, columnspan=5, pady=2)

        # AFE slots
        fa = ttk.LabelFrame(p, text="Slot A — IR — Ganancia AFE"); fa.pack(fill=tk.X, padx=10, pady=5)
        self._build_afe_slot(fa, sc, 'A')
        fb2 = ttk.LabelFrame(p, text="Slot B — Rojo — Ganancia AFE"); fb2.pack(fill=tk.X, padx=10, pady=5)
        self._build_afe_slot(fb2, sc, 'B')
        ttk.Button(p, text=f"Aplicar AMBOS Slots — {sc.label}",
                   command=lambda s=sc: self.apply_afe_both(s)).pack(pady=6)

        # Frecuencia y Promediado
        fsf = ttk.LabelFrame(p, text="Frecuencia de Muestreo y Promediado")
        fsf.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(fsf, text="Frecuencia:").grid(row=0, column=0, padx=8, pady=4, sticky=tk.W)
        fsv = tk.StringVar(value=sc.fsample_key); setattr(sc, 'fsample_var', fsv)
        ttk.Combobox(fsf, textvariable=fsv, values=list(FSAMPLE_PRESETS.keys()),
                     width=12, state="readonly").grid(row=0, column=1, padx=8)
        fml = ttk.Label(fsf, text="", foreground="gray")
        fml.grid(row=0, column=2, padx=8, sticky=tk.W)
        setattr(sc, 'fsmax_lbl', fml)
        ttk.Label(fsf, text="Promediado:").grid(row=1, column=0, padx=8, pady=4, sticky=tk.W)
        avgv = tk.StringVar(value=sc.avg_key); setattr(sc, 'avg_var', avgv)
        ttk.Combobox(fsf, textvariable=avgv, values=list(AVG_PRESETS.keys()),
                     width=22, state="readonly").grid(row=1, column=1, padx=8)
        ttk.Label(fsf, text="Si promedias Nx, sube fSAMPLE Nx para mantener tasa",
                  foreground="gray").grid(row=2, column=0, columnspan=3, padx=8, sticky=tk.W)
        ttk.Button(fsf, text="Aplicar Frecuencia y Promediado",
                   command=lambda s=sc: self.apply_fs_avg(s)).grid(row=3, column=0, columnspan=3, pady=6)

        sc.pulse_a_entry.bind("<FocusOut>", lambda e, s=sc: self._update_fsmax_label(s))
        sc.pulse_b_entry.bind("<FocusOut>", lambda e, s=sc: self._update_fsmax_label(s))
        self._update_fsmax_label(sc)

        # Chop Mode y VBias — activados por default, sin controles en UI
        # Se envían automáticamente al ESP32 en cada Play
        chv = tk.BooleanVar(value=sc.chop_mode); setattr(sc, 'chop_var', chv)
        vbv = tk.BooleanVar(value=sc.vbias);     setattr(sc, 'vbias_var', vbv)

        # Comando manual
        man = ttk.LabelFrame(p, text=f"Comando Manual — {sc.label}"); man.pack(fill=tk.X, padx=10, pady=5)
        ttk.Label(man, text="Comando:").grid(row=0, column=0, padx=8)
        me = ttk.Entry(man, width=36); me.grid(row=0, column=1, padx=8, pady=4)
        me.bind("<Return>", lambda e, s=sc, m=me: self._send_manual_sensor(s, m))
        setattr(sc, 'manual_entry', me)
        ttk.Button(man, text="Enviar",
                   command=lambda s=sc, m=me: self._send_manual_sensor(s, m)).grid(row=0, column=2, padx=8)
        ttk.Button(man, text="STATUS",
                   command=lambda s=sc: self._send_raw(f"STATUS{s.suffix}\n")).grid(row=0, column=3, padx=8)
        mr = ttk.Label(man, text="", foreground="blue", wraplength=440)
        mr.grid(row=1, column=0, columnspan=4, padx=8, pady=4, sticky=tk.W)
        setattr(sc, 'manual_resp_lbl', mr)

    def _build_afe_slot(self, parent, sc, slot):
        sl = slot.lower()
        ttk.Label(parent, text="R_F (TIA):").grid(row=0, column=0, padx=8, pady=4, sticky=tk.W)
        rfv = tk.StringVar(value=getattr(sc, f'rf_{sl}_key')); setattr(sc, f'rf_{sl}_var', rfv)
        ttk.Combobox(parent, textvariable=rfv, values=list(TIA_RF_MAP.keys()),
                     width=12, state="readonly").grid(row=0, column=1, padx=8)
        ttk.Label(parent, text="R_INT:").grid(row=0, column=2, padx=8, sticky=tk.W)
        rv = tk.StringVar(value=getattr(sc, f'rint_{sl}_key')); setattr(sc, f'rint_{sl}_var', rv)
        ttk.Combobox(parent, textvariable=rv, values=list(RINT_MAP.keys()),
                     width=20, state="readonly").grid(row=0, column=3, padx=8)
        ttk.Label(parent, text="Pulsos:").grid(row=1, column=0, padx=8, pady=4, sticky=tk.W)
        pe = ttk.Entry(parent, width=8)
        pe.insert(0, str(getattr(sc, f'pulse_{sl}')))
        pe.grid(row=1, column=1, padx=8, sticky=tk.W)
        setattr(sc, f'pulse_{sl}_entry', pe)
        rl = ttk.Label(parent, text="Reg TIA: --", foreground="gray")
        rl.grid(row=2, column=0, columnspan=2, padx=8, sticky=tk.W)
        setattr(sc, f'reg_{sl}_lbl', rl)
        ttk.Button(parent, text=f"Aplicar Slot {slot} →",
                   command=lambda s=sc, sl_=slot: self.apply_afe_slot(s, sl_)
                   ).grid(row=2, column=3, padx=8, pady=4)

    # ── Referencia ────────────────────────────────────────
    def _build_tab_info(self, parent):
        ttk.Label(parent, text="Tabla 28 del datasheet: SNR vs R_F + R_INT",
                  font=("", 10, "bold")).pack(anchor=tk.W, padx=10, pady=6)
        ttk.Label(parent, text="Condiciones: pulso LED 2µs, CPD = 30 pF",
                  foreground="gray").pack(anchor=tk.W, padx=10)
        cols = ("R_F (kΩ)", "R_INT (kΩ)", "I@70%FS (µA)",
                "Ruido (nA rms)", "I_amb máx (µA)", "SNR (dB)")
        tree = ttk.Treeview(parent, columns=cols, show="headings", height=7)
        for c in cols:
            tree.heading(c, text=c); tree.column(c, width=120, anchor=tk.CENTER)
        best_snr = max(r[5] for r in SNR_TABLE)
        for row in SNR_TABLE:
            tag = "best" if row[5] == best_snr else ""
            tree.insert("", tk.END, values=(
                f"{row[0]} kΩ", f"{row[1]} kΩ", f"{row[2]} µA",
                f"{row[3]} nA",  f"{row[4]} µA",  f"{row[5]} dB"), tags=(tag,))
        tree.tag_configure("best", background="#d4edda")
        tree.pack(fill=tk.X, padx=10, pady=6)
        ttk.Label(parent, text="★ Fila verde: mejor SNR general.",
                  foreground="green").pack(anchor=tk.W, padx=10)
        ttk.Separator(parent).pack(fill=tk.X, padx=10, pady=6)
        ttk.Label(parent, text="Pines I2C ESP32:",
                  font=("", 9, "bold")).pack(anchor=tk.W, padx=10)
        for line in ["  Sensor 1 → Wire:  SDA=GPIO21, SCL=GPIO22",
                     "  Sensor 2 → Wire1: SDA=GPIO18, SCL=GPIO19",
                     "  Ambos en 0x64 — buses físicamente separados",
                     "  Comunicación: Bluetooth SPP — parear NIRS_ESP32 en Windows"]:
            ttk.Label(parent, text=line, foreground="gray").pack(anchor=tk.W, padx=20)

    # ──────────────────────────────────────────────────────
    # HELPERS UI
    # ──────────────────────────────────────────────────────
    def _update_cfg_panel(self, sc):
        panel = self.cfg_panel_s1 if sc.suffix == "_S1" else self.cfg_panel_s2
        try:
            ba, bb = calc_bits_efectivos(sc.pulse_a, sc.pulse_b)
            tasa   = calc_tasa_salida(sc.fsample_key, sc.avg_key, sc.pulse_a, sc.pulse_b)
            fs_hz  = fsample_reg_to_hz(FSAMPLE_PRESETS.get(sc.fsample_key, 0x0050))
            avg_f  = 2 ** ((AVG_PRESETS.get(sc.avg_key, 0x0000) >> 4) & 0x7)
            ct = "blue" if tasa >= 50 else ("orange" if tasa >= 25 else "red")
            panel._lbl_tasa.config(text=f"{tasa:.0f} Hz", foreground=ct)
            panel._lbl_fs.config(text=f"{fs_hz:.0f} Hz")
            panel._lbl_avg.config(text=f"{avg_f}x")
            ca = panel._color_ir  if ba >= 16 else ("orange" if ba >= 15 else "red")
            cb = panel._color_red if bb >= 16 else ("orange" if bb >= 15 else "red")
            panel._lbl_ba.config(text=f"{ba:.1f} bits", foreground=ca)
            panel._lbl_bb.config(text=f"{bb:.1f} bits", foreground=cb)
        except Exception:
            pass

    def _update_fsmax_label(self, sc):
        try:
            pa = int(sc.pulse_a_entry.get().strip())
            pb = int(sc.pulse_b_entry.get().strip())
        except (ValueError, AttributeError):
            return
        fmax = calc_max_fsample_hz(pa, pb)
        sc.fsmax_lbl.config(text=f"fMAX ({pa}+{pb} pulsos): {fmax:.0f} Hz",
                            foreground="blue" if fmax > 100 else "orange")

    def _send_manual_sensor(self, sc, entry):
        cmd = entry.get().strip()
        if not cmd: return
        if self._send_raw(cmd + "\n"):
            sc.manual_resp_lbl.config(text=f"Enviado: {cmd}")

    # ──────────────────────────────────────────────────────
    # COMANDOS
    # ──────────────────────────────────────────────────────
    def send_led_cmd(self, sc, led, ma):
        reg = convert_ma_to_reg(ma)
        if led == 'IR':
            sc.reg_ir = reg
            sc.ir_real_lbl.config(text="OFF" if ma==0 else f"{get_actual_current_ma(reg)} mA")
            sc.ir_reg_lbl.config(text="0x0000" if ma==0 else hex(reg))
        else:
            sc.reg_red = reg
            sc.red_real_lbl.config(text="OFF" if ma==0 else f"{get_actual_current_ma(reg)} mA")
            sc.red_reg_lbl.config(text="0x0000" if ma==0 else hex(reg))
        self._send_raw(f"{led}{sc.suffix}={hex(reg)}\n")

    def apply_current(self, sc, led):
        entry = sc.ir_entry if led == 'IR' else sc.red_entry
        try:
            ma = float(entry.get().strip())
        except ValueError:
            messagebox.showerror("Error", "Ingresa un número válido en mA."); return
        if ma < 0:
            messagebox.showerror("Error", "La corriente no puede ser negativa."); return
        if led == 'IR':  sc.ir_ma  = ma
        else:            sc.red_ma = ma
        self.send_led_cmd(sc, led, ma)

    def apply_afe_slot(self, sc, slot):
        sl = slot.lower()
        pe = getattr(sc, f'pulse_{sl}_entry')
        try:
            p = max(1, min(int(pe.get().strip()), MAX_PULSES))
        except ValueError:
            messagebox.showerror("Error", f"Pulsos {slot}: ingresa un entero."); return
        rf_key   = getattr(sc, f'rf_{sl}_var').get()
        rint_key = getattr(sc, f'rint_{sl}_var').get()
        setattr(sc, f'rf_{sl}_key',   rf_key)
        setattr(sc, f'rint_{sl}_key', rint_key)
        setattr(sc, f'pulse_{sl}', p)
        tia_reg   = build_tia_reg(rf_key, rint_key)
        pulse_reg = (p << 8) | 0x18
        getattr(sc, f'reg_{sl}_lbl').config(
            text=f"Reg TIA_{slot}: 0x{tia_reg:04X}  |  PULSES_{slot}: 0x{pulse_reg:04X}")
        self._send_raw(f"TIA_{slot}{sc.suffix}=0x{tia_reg:04X}\n")
        self._send_raw(f"PULSES_{slot}{sc.suffix}=0x{pulse_reg:04X}\n")
        self._update_fsmax_label(sc)
        self._update_cfg_panel(sc)

    def apply_afe_both(self, sc):
        self.apply_afe_slot(sc, 'A')
        self.apply_afe_slot(sc, 'B')

    def apply_fs_avg(self, sc):
        fs_key  = sc.fsample_var.get()
        avg_key = sc.avg_var.get()
        sc.fsample_key = fs_key
        sc.avg_key     = avg_key
        self._send_raw(f"FSAMPLE{sc.suffix}=0x{FSAMPLE_PRESETS.get(fs_key, 0x0050):04X}\n")
        self._send_raw(f"AVG{sc.suffix}=0x{AVG_PRESETS.get(avg_key, 0x0000):04X}\n")
        self._update_cfg_panel(sc)

    def apply_chop(self, sc):
        val = 1 if sc.chop_var.get() else 0
        sc.chop_mode = bool(val)
        self._send_raw(f"CHOP{sc.suffix}={val}\n")

    def apply_vbias(self, sc):
        val = 1 if sc.vbias_var.get() else 0
        sc.vbias = bool(val)
        self._send_raw(f"VBIAS{sc.suffix}={val}\n")

    # ──────────────────────────────────────────────────────
    # MODOS
    # ──────────────────────────────────────────────────────
    def set_realtime_mode(self):
        self.mode = "realtime"; self.acq_duration = None; self.save_path = None
        messagebox.showinfo("Modo", "Modo: Tiempo Real activado.")

    def configure_timed_mode(self):
        dur = simpledialog.askinteger("Duración", "Tiempo (seg):", minvalue=1)
        if not dur: return
        folder = filedialog.askdirectory(title="Carpeta CSV")
        if not folder: return
        name = simpledialog.askstring("Nombre", "Nombre base (sin extensión):")
        if not name: return
        name = name.strip().replace(" ", "_")
        self.mode = "timed"; self.acq_duration = dur
        self.save_path = os.path.join(folder, name)
        messagebox.showinfo("Configurado",
                            f"Temporizado {dur}s\n"
                            f"Archivos: {name}_S1.csv y {name}_S2.csv\n"
                            f"Presiona ▶ Play")

    # ──────────────────────────────────────────────────────
    # ADQUISICIÓN
    # ──────────────────────────────────────────────────────
    def on_play(self):
        if not (self.serial_port and self.serial_port.is_open):
            messagebox.showerror("Sin conexión",
                                 "Conecta al puerto primero usando el botón Conectar.")
            return
        if not self.read_thread_running:
            self.times     = []
            self.ir1_data  = []; self.red1_data = []
            self.ir2_data  = []; self.red2_data = []
            self.read_thread_running = True
            self.paused = False
            self._warming_up = True   # flag: descartando datos de warm-up
            self.start_time = time.time() + WARMUP_S  # start_time en el futuro
            with self.data_queue.mutex: self.data_queue.queue.clear()
            self.plot1.clear(); self.plot2.clear()

            # Enviar configuración al sensor
            for sc in (self.s1, self.s2):
                self.send_led_cmd(sc, 'IR',   sc.ir_ma)
                self.send_led_cmd(sc, 'Rojo', sc.red_ma)
                self.apply_afe_both(sc)
                self.apply_fs_avg(sc)
                if sc.chop_mode: self.apply_chop(sc)
                if sc.vbias:     self.apply_vbias(sc)

            self.read_thread = threading.Thread(
                target=self.read_serial_loop, daemon=True)
            self.read_thread.start()

            # Después de WARMUP_S segundos, resetear y empezar a graficar desde 0
            self.root.after(int(WARMUP_S * 1000), self._end_warmup)
        else:
            self.paused = False
        self._clock_running = True

        self.play_btn.config(state=tk.DISABLED)
        self.pause_btn.config(text="⏸ Pausa", state=tk.NORMAL)
        self.stop_btn.config(state=tk.NORMAL)
        if self._warming_up:
            self.status_lbl.config(text=f"Estabilizando sensor ({int(WARMUP_S)}s)...", foreground="orange")
        else:
            self.status_lbl.config(text="Adquiriendo...", foreground="green")
        self._clock_running = True

    def on_pause(self):
        self.paused = True
        self.play_btn.config(state=tk.NORMAL)
        self.pause_btn.config(text="▶ Reanudar", state=tk.DISABLED)
        self.status_lbl.config(text="Pausado", foreground="orange")

    def on_stop(self):
        self.read_thread_running = False; self.paused = True
        self._clock_running = False
        self._warming_up   = False
        self.clock_lbl.config(text="0 s", foreground="#22c55e")
        # NO cerrar el puerto — solo detener adquisición
        # El puerto queda abierto para seguir enviando comandos de configuración
        # Vaciar buffer de entrada para que no queden datos acumulados
        if self.serial_port and self.serial_port.is_open:
            try: self.serial_port.flushInput()
            except: pass
        self.play_btn.config(state=tk.NORMAL)
        self.pause_btn.config(text="⏸ Pausa", state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        self.status_lbl.config(text="Conectado — adquisición detenida", foreground="blue")

    # ──────────────────────────────────────────────────────
    # WARM-UP
    # ──────────────────────────────────────────────────────
    def _end_warmup(self):
        """Llamado después de WARMUP_S segundos — resetea el tiempo y empieza a graficar."""
        if not self.read_thread_running:
            return  # si ya se detuvo, no hacer nada
        # Resetear datos acumulados durante warm-up
        self.times     = []
        self.ir1_data  = []; self.red1_data = []
        self.ir2_data  = []; self.red2_data = []
        with self.data_queue.mutex: self.data_queue.queue.clear()
        self.plot1.clear(); self.plot2.clear()
        # Ahora start_time es el momento actual → datos empiezan desde t=0
        self.start_time  = time.time()
        self._warming_up = False
        self._clock_running = True
        self.status_lbl.config(text="Adquiriendo...", foreground="green")

    # ──────────────────────────────────────────────────────
    # RELOJ DE TIEMPO TRANSCURRIDO
    # ──────────────────────────────────────────────────────
    def _update_clock(self, elapsed_s):
        """Actualiza el reloj sincronizado con el tiempo real de la gráfica."""
        s = int(elapsed_s)
        if s < 300:
            color = "#22c55e"   # verde
        elif s < 600:
            color = "#f59e0b"   # naranja
        else:
            color = "#ef4444"   # rojo
        self.clock_lbl.config(text=f"{s:d} s", foreground=color)

    # ──────────────────────────────────────────────────────
    # SERIAL
    # ──────────────────────────────────────────────────────
    def update_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def open_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self._on_serial_ready(); return
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Selecciona un puerto."); return
        try:
            self.serial_port = serial.Serial(port, BAUD_RATE, timeout=0.1)
            self.connect_btn.config(text="Conectando...", state=tk.DISABLED)
            self.status_lbl.config(text="Esperando ESP32 BT (3s)...", foreground="orange")
            def _wait():
                time.sleep(3)
                self.serial_port.flushInput()
                self.root.after(0, self._on_serial_ready)
            threading.Thread(target=_wait, daemon=True).start()
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo abrir {port}:\n{e}")
            self.serial_port = None

    def _on_serial_ready(self):
        self.connect_btn.config(text="Conectado ✓", state=tk.DISABLED)
        self.disconnect_btn.config(state=tk.NORMAL)
        self.play_btn.config(state=tk.NORMAL)
        self.status_lbl.config(text="Conectado — listo para configurar y adquirir",
                               foreground="blue")

    def _on_serial_lost(self):
        """Llamado cuando se pierde la conexión serial inesperadamente."""
        self.close_serial()
        self.play_btn.config(state=tk.DISABLED)
        self.pause_btn.config(state=tk.DISABLED)
        self.stop_btn.config(state=tk.DISABLED)
        messagebox.showerror("Desconexión", "Puerto serial desconectado.")

    def manual_disconnect(self):
        """Desconectar manualmente sin detener adquisición si está corriendo."""
        if self.read_thread_running:
            self.on_stop()
        self.close_serial()

    def close_serial(self):
        if self.serial_port and self.serial_port.is_open:
            try: self.serial_port.close()
            except: pass
        self.serial_port = None
        self.connect_btn.config(text="Conectar", state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.play_btn.config(state=tk.DISABLED)
        self.status_lbl.config(text="Desconectado", foreground="red")

    def on_close(self):
        self.read_thread_running = False
        self._drain_running = False
        self.cmd_queue.put(None)
        self.close_serial()
        self.root.destroy()

    # ──────────────────────────────────────────────────────
    # HILO LECTOR SERIAL
    # ──────────────────────────────────────────────────────
    def read_serial_loop(self):
        while self.read_thread_running:
            if self.paused or not (self.serial_port and self.serial_port.is_open):
                time.sleep(0.02); continue
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()
                    if line.count(",") != 3: continue
                    try:
                        ir1, red1, ir2, red2 = (int(x) for x in line.split(","))
                    except ValueError:
                        continue
                    self.data_queue.put(
                        (time.time() - self.start_time, ir1, red1, ir2, red2))
            except serial.SerialException:
                self.read_thread_running = False
                self.root.after(100, self._on_serial_lost)
            except Exception:
                continue

    # ──────────────────────────────────────────────────────
    # CSV  — con bloque de metadata de configuración
    # ──────────────────────────────────────────────────────
    def _build_metadata(self, sc):
        """
        Devuelve un dict con toda la configuración del sensor.
        Se guarda como config.json junto al datos.csv.

        Leer en Python:
            import json, pandas as pd
            meta = json.load(open("experimento_S1_config.json"))
            df   = pd.read_csv("experimento_S1_datos.csv")
            # df["IR_A"] y df["Rojo_A"] ya están convertidos a amperios
        """
        import datetime, math as _math

        RF_MAP   = {"200 kΩ": 200e3, "100 kΩ": 100e3, "50 kΩ": 50e3, "25 kΩ": 25e3}
        RINT_MAP2 = {"400 kΩ (max SNR)": 400e3, "200 kΩ": 200e3, "100 kΩ": 100e3}
        VREF = 1.0

        rf_a   = RF_MAP.get(sc.rf_a_key, 0)
        rf_b   = RF_MAP.get(sc.rf_b_key, 0)
        rint_a = RINT_MAP2.get(sc.rint_a_key, 0)
        rint_b = RINT_MAP2.get(sc.rint_b_key, 0)

        # Factor ADC → fotocorriente (A): I = ADC * factor
        # Derivado de: V_out = I_foto * RF * N_pulsos  →  ADC = V_out/VREF * 65535
        # ∴  I_foto(A) = ADC * VREF / (65535 * RF * N_pulsos)
        f_ir   = VREF / (65535 * rf_a * sc.pulse_a) if rf_a and sc.pulse_a else 0
        f_rojo = VREF / (65535 * rf_b * sc.pulse_b) if rf_b and sc.pulse_b else 0

        # Corrección espectral VEMD5010X01 (Vishay) — Fig. 6 datasheet
        # Sensibilidad relativa leída de la curva:
        #   850 nm (IR)   → S_rel = 0.95
        #   660 nm (Rojo) → S_rel = 0.60
        # Nota: S_rel se cancela en ΔOD = -log10(I/I₀), NO afecta MBLL ni HbO₂/HHb
        S_rel_IR   = 0.95   # VEMD5010X01 a 850 nm
        S_rel_Rojo = 0.60   # VEMD5010X01 a 660 nm
        f_ir_corr   = f_ir   / S_rel_IR   if f_ir   > 0 else 0
        f_rojo_corr = f_rojo / S_rel_Rojo if f_rojo > 0 else 0

        fs_hz = fsample_reg_to_hz(FSAMPLE_PRESETS.get(sc.fsample_key, 0x0050))
        avg_f = 2 ** ((AVG_PRESETS.get(sc.avg_key, 0x0000) >> 4) & 0x7)
        tasa  = calc_tasa_salida(sc.fsample_key, sc.avg_key, sc.pulse_a, sc.pulse_b)
        bits_ir   = 14.0 + (0.5 * _math.log2(sc.pulse_a) if sc.pulse_a > 1 else 0)
        bits_rojo = 14.0 + (0.5 * _math.log2(sc.pulse_b) if sc.pulse_b > 1 else 0)

        now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # ── Fila de encabezados de metadata ───────────────
        header = [
            "sensor", "sufijo_comando", "fecha_hora",
            "canal_IR_nm", "IR_RF_A_ohm", "IR_RINT_A_ohm",
            "IR_pulsos_N", "IR_corriente_LED_mA", "IR_bits_efectivos",
            "IR_factor_escala_A_por_cuenta",
            "canal_Rojo_nm", "Rojo_RF_B_ohm", "Rojo_RINT_B_ohm",
            "Rojo_pulsos_N", "Rojo_corriente_LED_mA", "Rojo_bits_efectivos",
            "Rojo_factor_escala_A_por_cuenta",
            "fSAMPLE_Hz", "promediado_factor", "tasa_salida_Hz",
            "chop_mode", "vbias_inverso",
            "ADC_max_cuentas", "ADC_saturacion_umbral", "VREF_V",
            "BL_eps_HbO2_IR", "BL_eps_HbO2_Rojo",
            "BL_eps_HHb_IR",  "BL_eps_HHb_Rojo",
            "BL_DPF_musculo",
            "fotodiodo", "VEMD5010_S_rel_850nm", "VEMD5010_S_rel_660nm",
            "IR_factor_corregido_A_por_cuenta",
            "Rojo_factor_corregido_A_por_cuenta",
        ]

        # ── Fila de valores de metadata ───────────────────
        values = [
            sc.label, sc.suffix, now,
            850, rf_a, rint_a,
            sc.pulse_a, sc.ir_ma, f"{bits_ir:.2f}",
            f"{f_ir:.6e}",
            660, rf_b, rint_b,
            sc.pulse_b, sc.red_ma, f"{bits_rojo:.2f}",
            f"{f_rojo:.6e}",
            f"{fs_hz:.1f}", avg_f, f"{tasa:.1f}",
            "ON" if sc.chop_mode else "OFF",
            "ON" if sc.vbias     else "OFF",
            65535, ADC_SAT_THRESH, VREF,
            0.28, 1.48, 1.84, 3.66, 5.0,
            "VEMD5010X01", S_rel_IR, S_rel_Rojo,
            f"{f_ir_corr:.6e}", f"{f_rojo_corr:.6e}",
        ]

        return dict(zip(header, values))

    def save_to_csv(self):
        import json
        try:
            saved = []
            for sc, suf, ir_d, red_d in [
                (self.s1, "_S1", self.ir1_data, self.red1_data),
                (self.s2, "_S2", self.ir2_data, self.red2_data),
            ]:
                meta = self._build_metadata(sc)
                f_ir   = float(meta["IR_factor_escala_A_por_cuenta"])
                f_rojo = float(meta["Rojo_factor_escala_A_por_cuenta"])

                # ── config.json ───────────────────────────
                json_path = self.save_path + suf + "_config.json"
                with open(json_path, "w", encoding="utf-8") as jf:
                    json.dump(meta, jf, indent=4, ensure_ascii=False)

                # ── datos.csv ─────────────────────────────
                csv_path = self.save_path + suf + "_datos.csv"
                with open(csv_path, "w", newline="", encoding="utf-8") as cf:
                    w = csv.writer(cf)
                    w.writerow(["tiempo_s",
                                "IR_ADC", "IR_A",
                                "Rojo_ADC", "Rojo_A"])
                    for t, ir, rd in zip(self.times, ir_d, red_d):
                        w.writerow([
                            round(t, 4),
                            ir,   round(ir   * f_ir,   14),
                            rd,   round(rd   * f_rojo, 14),
                        ])
                saved.append(f"{suf}: {csv_path}\n     {json_path}")

            messagebox.showinfo("Archivos guardados",
                                "Guardados:\n" + "\n".join(saved))
        except Exception as e:
            messagebox.showerror("Error al guardar", str(e))

    # ──────────────────────────────────────────────────────
    # ACTUALIZACIÓN DE GRÁFICA — Canvas nativo, ~33 FPS
    # ──────────────────────────────────────────────────────
    def update_plot(self):
        # Vaciar cola de golpe
        batch = []
        try:
            while True: batch.append(self.data_queue.get_nowait())
        except queue.Empty:
            pass

        if batch:
            for t, ir1, red1, ir2, red2 in batch:
                self.times.append(t)
                self.ir1_data.append(ir1);  self.red1_data.append(red1)
                self.ir2_data.append(ir2);  self.red2_data.append(red2)

            # Recortar histórico
            if len(self.times) > MAX_HIST_PTS:
                trim = len(self.times) - MAX_HIST_PTS
                self.times     = self.times[trim:]
                self.ir1_data  = self.ir1_data[trim:];  self.red1_data = self.red1_data[trim:]
                self.ir2_data  = self.ir2_data[trim:];  self.red2_data = self.red2_data[trim:]

            t_now = self.times[-1]
            t_min = max(0.0, t_now - WINDOW_SECONDS)
            idx0  = next((i for i, v in enumerate(self.times) if v >= t_min), 0)

            t_p    = self.times[idx0:]
            ir1_p  = self.ir1_data[idx0:];  red1_p = self.red1_data[idx0:]
            ir2_p  = self.ir2_data[idx0:];  red2_p = self.red2_data[idx0:]

            # Actualizar reloj sincronizado con tiempo real de datos
            if self._clock_running:
                self._update_clock(t_now)

            # Decimación — max 800 puntos por canvas (suficiente para ver la forma)
            n = len(t_p)
            if n > 800:
                step   = n // 800
                t_p    = t_p[::step]
                ir1_p  = ir1_p[::step];  red1_p = red1_p[::step]
                ir2_p  = ir2_p[::step];  red2_p = red2_p[::step]

            self.plot1.update(t_p, ir1_p, red1_p, t_min, t_now)
            self.plot2.update(t_p, ir2_p, red2_p, t_min, t_now)

            # Calidad de señal — calcular con la ventana visible completa
            # SCI usa la señal entera de la ventana (max 10s)
            # CV usa los últimos 5s para ser más reactivo a cambios
            n5 = max(0, len(t_p) - len(t_p) // 2)  # últimos ~5s

            sci1 = calc_sci(ir1_p, red1_p, t_arr=t_p)
            cv1_ir  = calc_cv(ir1_p[n5:])
            cv1_red = calc_cv(red1_p[n5:])
            sl1, sc1 = sci_label(sci1)
            cl1, cc1 = cv_label(max(cv1_ir or 0, cv1_red or 0))

            self.sci1_val.config(text=f"{sci1:.3f}" if sci1 is not None else "--")
            self.sci1_lbl.config(text=sl1, foreground=sc1)
            self.cv1_ir.config( text=f"{cv1_ir:.2f}%"  if cv1_ir  is not None else "--")
            self.cv1_red.config(text=f"{cv1_red:.2f}%" if cv1_red is not None else "--")
            self.cv1_lbl.config(text=cl1, foreground=cc1)

            sci2 = calc_sci(ir2_p, red2_p, t_arr=t_p)
            cv2_ir  = calc_cv(ir2_p[n5:])
            cv2_red = calc_cv(red2_p[n5:])
            sl2, sc2 = sci_label(sci2)
            cl2, cc2 = cv_label(max(cv2_ir or 0, cv2_red or 0))

            self.sci2_val.config(text=f"{sci2:.3f}" if sci2 is not None else "--")
            self.sci2_lbl.config(text=sl2, foreground=sc2)
            self.cv2_ir.config( text=f"{cv2_ir:.2f}%"  if cv2_ir  is not None else "--")
            self.cv2_red.config(text=f"{cv2_red:.2f}%" if cv2_red is not None else "--")
            self.cv2_lbl.config(text=cl2, foreground=cc2)

            # Modo temporizado
            if (self.mode == "timed" and self.acq_duration
                    and t_now >= self.acq_duration and self.read_thread_running):
                self.read_thread_running = False; self.paused = True
                self.close_serial()
                self.play_btn.config(state=tk.NORMAL)
                self.pause_btn.config(state=tk.DISABLED)
                self.stop_btn.config(state=tk.DISABLED)
                self.status_lbl.config(text="Guardando CSV...", foreground="orange")
                self.save_to_csv()
                self.mode = "realtime"; self.acq_duration = None; self.save_path = None
                self.status_lbl.config(text="Detenido", foreground="red")

        self.root.after(PLOT_INTERVAL_MS, self.update_plot)


# ══════════════════════════════════════════════════════════
# PUNTO DE ENTRADA
# ══════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    root.minsize(960, 700)
    app = PPGDualApp(root)
    root.mainloop()