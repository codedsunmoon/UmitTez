"""
╔══════════════════════════════════════════════════════════════════╗
║          ÜmitTez — SIAM SDP KOMPRESÖR KONTROL PANELİ  v2.1      ║
║          MODBUS RTU  |  SDP-Series Power Driver                  ║
║          TDID21A008 RevD                                         ║
╠══════════════════════════════════════════════════════════════════╣
║  v2.1 Değişiklikler:                                             ║
║  • Output Voltage (0208) → Gauge + Grafik + Duty-Cycle tahmini  ║
║  • Tüm Modbus tablosunda yazılabilir register'lar için           ║
║    inline giriş kutusu + YAZ butonu                             ║
║  • Slider anlık Hz komutu gönderir (debounce 400ms)             ║
║  • Güç W ve kW birlikte gösterilir                               ║
║  • Çoklu Y eksenli otomatik ölçekli grafik                       ║
╚══════════════════════════════════════════════════════════════════╝
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import threading
import time
import csv
import os
import math
import subprocess
import sys
from datetime import datetime
from collections import deque

try:
    from pymodbus.client import ModbusSerialClient as ModbusClient
    MODBUS_OK = True
except ImportError:
    MODBUS_OK = False

# ── Renk paleti ───────────────────────────────────────────────────────────────
BG      = "#0b0f14"
PANEL   = "#131820"
PANEL2  = "#1a2232"
BORDER  = "#253040"
ACCENT  = "#00e5c0"
ACCENT2 = "#ff5e3a"
WARN    = "#ffb300"
DANGER  = "#ff2d55"
GREEN   = "#3ddc84"
BLUE    = "#4fc3f7"
PURPLE  = "#ce93d8"
CYAN    = "#00bcd4"   # Output Volt için
TEXT    = "#dce8f5"
DIM     = "#5a7080"
BLACK   = "#060a0e"

CH_COLOR = {"Hz": ACCENT, "V": BLUE, "A": WARN,
            "TC": ACCENT2, "TH": PURPLE, "W": GREEN, "Vout": CYAN}
CH_LABEL = {"Hz": "Frekans Hz", "V": "DC Bus V", "A": "Akım A",
            "TC": "Basma °C", "TH": "Soğutucu °C", "W": "Güç W",
            "Vout": "Çıkış V (LL)"}
CH_SCALE = {"Hz": (0, 400), "V": (0, 800), "A": (0, 100),
            "TC": (-30, 130), "TH": (-30, 130), "W": (0, 25000),
            "Vout": (0, 500)}

FT  = ("Consolas", 13, "bold")
FB  = ("Consolas", 19, "bold")
FM  = ("Consolas", 10)
FS  = ("Consolas", 8)
FT2 = ("Consolas", 7)
FLB = ("Consolas", 9)
FBT = ("Consolas", 9, "bold")

# ── MODBUS adresleri ──────────────────────────────────────────────────────────
A_FREQ_CMD   = 101
A_CTRL_CMD   = 102
A_OP_STAT    = 201
A_PROT_STAT  = 202
A_TRIP       = 203
A_FREQ_OUT   = 204
A_SPEED_EST  = 205
A_DC_BUS     = 206
A_OUT_CURR   = 207
A_OUT_VOLT   = 208
A_HEATSINK   = 209
A_DISCHARGE  = 210
A_FREQ_RO    = 211
A_OP_MODE    = 301
A_COMP_MODEL = 302
A_HOLD_FREQ  = 303
A_HOLD_TIME  = 304
A_MAX_FREQ   = 305
A_MIN_FREQ   = 306
A_STOP_FREQ  = 309
A_CTRL_TEMP  = 310
A_PROT_TEMP  = 311
A_TRIP_TEMP  = 312
A_SAVE       = 313
A_STALL      = 315
A_HEAT_EN    = 325
A_HEAT_TEMP  = 326
A_HEAT_POW   = 327
A_BAUD       = 501
A_FORMAT     = 502
A_TIMEOUT    = 503
A_SLAVE      = 504
A_SW_VER     = 9901
A_PARAM_TBL  = 9902

TRIP_NAMES = {
    0:"Normal (Trip Yok)", 1:"Soğutucu Aşırı Isınma (HW)",
    2:"Aşırı Akım Hızlanma (HW)", 3:"Aşırı Akım Sabit (HW)",
    4:"Aşırı Akım Yavaşlama (HW)", 5:"DC Bus Düşük Gerilim",
    6:"DC Bus Aşırı Gerilim", 10:"Aşırı Akım Hızlanma (SW)",
    12:"Aşırı Akım Sabit (SW)", 13:"Aşırı Akım Yavaşlama (SW)",
    15:"MODBUS Zaman Aşımı", 16:"Soğutucu Sensör Hatası",
    17:"Basma Sensör Hatası", 18:"Basma Aşırı Isınma (HW)",
    19:"Motor Başlatma Başarısız", 20:"Motor Pozisyon Kaybı",
    24:"Acil Durdurma", 25:"Soğutucu Aşırı Isınma (SW)",
    26:"Basma Aşırı Isınma (SW)", 30:"İç Haberleşme Kaybı",
    31:"Boost PFC Arızası",
}
PROT_NAMES = {
    0:"Başlatma", 1:"Normal", 2:"Kontrol", 3:"Koruma",
    4:"Kapatma", 5:"Sensör Arıza", 6:"Durdurma", 7:"Isıtma",
}

GRAPH_N = 120

# ─────────────────────────────────────────────────────────────────────────────
# GaugeMeter
# ─────────────────────────────────────────────────────────────────────────────
class GaugeMeter(tk.Canvas):
    def __init__(self, master, label, unit, vmin, vmax,
                 warn=None, danger=None, size=110, color=ACCENT, **kw):
        super().__init__(master, width=size, height=size // 2 + 28,
                         bg=PANEL2, highlightthickness=0, **kw)
        self._label  = label; self._unit = unit
        self._vmin   = vmin;  self._vmax = vmax
        self._warn   = warn;  self._danger = danger
        self._size   = size;  self._color = color
        self._val    = vmin
        self._draw(vmin)

    def set(self, v):
        if v != self._val:
            self._val = v
            self._draw(v)

    def _draw(self, v):
        self.delete("all")
        s = self._size; cx = s // 2; cy = s // 2; r = s // 2 - 8
        self.create_arc(cx-r, cy-r, cx+r, cy+r, start=0, extent=180,
                        style="arc", outline=BORDER, width=7)
        frac  = max(0.0, min(1.0, (v-self._vmin)/((self._vmax-self._vmin) or 1)))
        color = self._color
        if self._warn   and v >= self._warn:   color = WARN
        if self._danger and v >= self._danger: color = DANGER
        ext = frac * 180.0
        if ext > 0.5:
            self.create_arc(cx-r, cy-r, cx+r, cy+r,
                            start=180-ext, extent=ext, style="arc",
                            outline=color, width=7)
        ang = math.radians(180.0 - frac * 180.0)
        nx = cx + (r-12) * math.cos(ang); ny = cy - (r-12) * math.sin(ang)
        self.create_line(cx, cy, nx, ny, fill=color, width=2)
        self.create_oval(cx-4, cy-4, cx+4, cy+4, fill=color, outline="")
        if abs(v) >= 1000:
            txt = f"{v/1000:.2f}k"
        else:
            txt = f"{v:.1f}" if isinstance(v, float) else str(v)
        self.create_text(cx, cy+10, text=txt, fill=TEXT,
                         font=("Consolas", 10, "bold"))
        self.create_text(cx, cy+24, text=f"{self._label} [{self._unit}]",
                         fill=DIM, font=("Consolas", 7))


# ─────────────────────────────────────────────────────────────────────────────
# DutyGauge — Duty cycle + Vout özel göstergesi
# ─────────────────────────────────────────────────────────────────────────────
class DutyGauge(tk.Canvas):
    """
    Output Voltage + Duty Cycle tahmini.
    Duty = Vout_LL / (Vdc_bus * sqrt(3)/2) — SPVM (Sinüsoidal PWM) yaklaşımı.
    Alternatif: basit oran Vout_LN / (Vdc_bus / 2)
    Göstergede hem Volt hem de % duty gösterilir.
    """
    def __init__(self, master, size=118, **kw):
        super().__init__(master, width=size, height=size // 2 + 42,
                         bg=PANEL2, highlightthickness=0, **kw)
        self._size = size
        self._vout = 0.0
        self._vdc  = 0.0
        self._draw()

    def set(self, vout, vdc):
        if vout != self._vout or vdc != self._vdc:
            self._vout = vout
            self._vdc  = vdc
            self._draw()

    def _calc_duty(self):
        """
        3-fazlı SPWM için teorik modülasyon oranı (m_a):
          Vout_LL_peak = Vdc_bus * m_a * sqrt(3)/2  (doğrusal bölge, m_a ≤ 1)
          Vout_LL_rms  = Vout_LL_peak / sqrt(2)
        Dolayısıyla:
          m_a = Vout_LL_rms * sqrt(2) / (Vdc_bus * sqrt(3)/2)
              = Vout_LL_rms * 2*sqrt(2) / (Vdc_bus * sqrt(3))
        m_a = 1 → %100 duty (lineer bölge sınırı)
        %duty (anahtarlama) ≈ m_a * 100 (yaklaşık, gerçek duty switch periyoduna göre değişir)
        """
        if self._vdc < 10:
            return 0.0
        ma = (self._vout * 2.0 * math.sqrt(2)) / (self._vdc * math.sqrt(3))
        return min(ma, 1.5)  # aşırı modülasyon görünür olsun

    def _draw(self):
        self.delete("all")
        s = self._size; cx = s // 2; cy = s // 2; r = s // 2 - 8

        duty = self._calc_duty()
        frac = max(0.0, min(1.0, duty))
        color = CYAN
        if duty > 1.0:   color = ACCENT2   # aşırı modülasyon
        elif duty > 0.9: color = WARN

        # Arka plan yay
        self.create_arc(cx-r, cy-r, cx+r, cy+r, start=0, extent=180,
                        style="arc", outline=BORDER, width=7)
        # Duty yayı
        ext = frac * 180.0
        if ext > 0.5:
            self.create_arc(cx-r, cy-r, cx+r, cy+r,
                            start=180-ext, extent=ext, style="arc",
                            outline=color, width=7)
        # İbre
        ang = math.radians(180.0 - frac * 180.0)
        nx = cx + (r-12) * math.cos(ang); ny = cy - (r-12) * math.sin(ang)
        self.create_line(cx, cy, nx, ny, fill=color, width=2)
        self.create_oval(cx-4, cy-4, cx+4, cy+4, fill=color, outline="")

        # Volt değeri
        self.create_text(cx, cy+8, text=f"{self._vout:.0f}",
                         fill=TEXT, font=("Consolas", 10, "bold"))
        # Duty %
        duty_pct = duty * 100.0
        duty_txt = f"m={duty_pct:.1f}%" if duty <= 1.0 else f"AŞIRI m={duty_pct:.0f}%"
        self.create_text(cx, cy+20, text=duty_txt,
                         fill=color, font=("Consolas", 7, "bold"))
        # Etiket
        self.create_text(cx, cy+32, text="Çıkış V [Vrms LL]",
                         fill=DIM, font=("Consolas", 7))


# ─────────────────────────────────────────────────────────────────────────────
# MultiAxisChart
# ─────────────────────────────────────────────────────────────────────────────
class MultiAxisChart(tk.Canvas):
    def __init__(self, master, channels, n=GRAPH_N, **kw):
        super().__init__(master, bg=BLACK, highlightthickness=1,
                         highlightbackground=BORDER, **kw)
        self.channels = channels
        self.n        = n
        self.bind("<Configure>", lambda e: self._redraw())

    def push(self, name, value):
        if name in self.channels:
            self.channels[name]["data"].append(value)

    def _auto_scale(self, data, default_scale):
        if len(data) < 2:
            return default_scale
        lo = min(data); hi = max(data)
        span = hi - lo
        if span < 1e-6:
            margin = max(abs(lo) * 0.1, 1.0)
            lo -= margin; hi += margin
        else:
            margin = span * 0.08
            lo -= margin; hi += margin
        return (lo, hi)

    def _redraw(self):
        self.delete("all")
        W = self.winfo_width(); H = self.winfo_height()
        if W < 20 or H < 20:
            return

        visible = [name for name, ch in self.channels.items()
                   if ch["visible"].get()]
        n_axes  = len(visible)
        PL = max(46, n_axes * 46)
        PR = 20; PT = 12; PB = 28
        gw = W - PL - PR; gh = H - PT - PB
        if gw < 20:
            return

        for i in range(5):
            y = PT + i * gh // 4
            self.create_line(PL, y, W-PR, y, fill="#192232", dash=(2,4))
        for i in range(7):
            x = PL + i * gw // 6
            self.create_line(x, PT, x, H-PB, fill="#192232", dash=(2,4))

        self.create_line(PL, PT, PL, H-PB, fill=BORDER, width=1)
        self.create_line(PL, H-PB, W-PR, H-PB, fill=BORDER, width=1)

        for ax_idx, name in enumerate(visible):
            ch    = self.channels[name]
            data  = list(ch["data"])
            color = ch["color"]
            scale = self._auto_scale(data, CH_SCALE.get(name, (0, 100)))
            vmin, vmax = scale
            span = (vmax - vmin) or 1

            ax_x = PL - ax_idx * 46
            self.create_line(ax_x, PT, ax_x, H-PB, fill=color, width=1)

            for i in range(5):
                y = H - PB - i * gh // 4
                v = vmin + i * (vmax - vmin) / 4
                self.create_line(ax_x-4, y, ax_x, y, fill=color, width=1)
                if abs(v) >= 1000:
                    lbl = f"{v/1000:.1f}k"
                elif abs(v) >= 10:
                    lbl = f"{v:.0f}"
                else:
                    lbl = f"{v:.1f}"
                self.create_text(ax_x-6, y, text=lbl,
                                 fill=color, font=FT2, anchor="e")

            self.create_text(ax_x, PT-8, text=name,
                             fill=color, font=("Consolas", 7, "bold"),
                             anchor="s")

            if len(data) < 2:
                continue
            pts = []
            for i, v in enumerate(data):
                x = PL + i * gw / (self.n - 1)
                y = H - PB - (v - vmin) / span * gh
                y = max(PT, min(H-PB, y))
                pts.extend([x, y])
            if len(pts) >= 4:
                self.create_line(*pts, fill=color, width=1, smooth=True)

            last = data[-1]
            y_l  = H - PB - (last - vmin) / span * gh
            y_l  = max(PT+8, min(H-PB-8, y_l))
            if abs(last) >= 1000:
                lbl = f"{last/1000:.2f}k"
            else:
                lbl = f"{last:.1f}"
            self.create_text(W-PR+2, y_l, text=lbl,
                             fill=color, font=FT2, anchor="w")

        self.create_text(W-PR, H-PB+14, text="◄ son dakika",
                         fill=DIM, font=FT2, anchor="e")

    def refresh(self):
        self._redraw()


# ─────────────────────────────────────────────────────────────────────────────
# Ana Uygulama
# ─────────────────────────────────────────────────────────────────────────────
class SDPPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("ÜmitTez — SIAM SDP Kompresör Kontrol Paneli  v2.1")
        self.root.configure(bg=BG)
        self.root.geometry("1280x1020")
        self.root.minsize(1050, 860)

        self.v_port   = tk.StringVar(value="COM3")
        self.v_baud   = tk.IntVar(value=9600)
        self.v_slave  = tk.IntVar(value=1)
        self.v_parity = tk.StringVar(value="N")
        self.v_stop   = tk.IntVar(value=1)
        self.v_cosphi = tk.DoubleVar(value=0.85)

        self.client    = None
        self.connected = False
        self._stop_ev  = threading.Event()
        self._slider_after = None

        self.is_logging = False
        self.log_path   = None
        self.log_file   = None
        self.csv_writer = None
        self.log_rows   = 0

        self.alarms = {
            "discharge_t": {"limit": tk.DoubleVar(value=100.0), "fired": False},
            "heatsink_t":  {"limit": tk.DoubleVar(value=80.0),  "fired": False},
            "output_curr": {"limit": tk.DoubleVar(value=30.0),  "fired": False},
            "dc_bus":      {"limit": tk.DoubleVar(value=750.0), "fired": False},
        }

        self.live = {
            "freq_out": 0.0,    "speed_est": 0.0,   "dc_bus": 0.0,
            "output_curr": 0.0, "output_volt": 0.0, "heatsink_t": 0.0,
            "discharge_t": 0.0, "freq_ro": 0.0,     "power_w": 0.0,
            "power_kw": 0.0,    "duty_pct": 0.0,
            "op_raw": 0, "prot_raw": 0, "trip_raw": 0,
            "op_mode": 0, "comp_model": 0, "hold_freq": 0.0,
            "hold_time": 0, "max_freq": 0.0, "min_freq": 0.0,
            "stop_freq": 0.0, "ctrl_temp": 0, "prot_temp": 0,
            "trip_temp": 0, "stall": 0,
            "heat_en": 0, "heat_temp": 0.0, "heat_pow": 0,
            "baud_code": 0, "fmt_code": 0, "timeout_en": 0,
            "slave_addr": 0, "sw_ver": 0, "param_tbl": 0,
        }

        # Grafik kanalları — Vout eklendi
        self.ch = {
            "Hz":   {"color": CH_COLOR["Hz"],   "data": deque(maxlen=GRAPH_N),
                     "visible": tk.BooleanVar(value=True)},
            "V":    {"color": CH_COLOR["V"],    "data": deque(maxlen=GRAPH_N),
                     "visible": tk.BooleanVar(value=True)},
            "Vout": {"color": CH_COLOR["Vout"], "data": deque(maxlen=GRAPH_N),
                     "visible": tk.BooleanVar(value=True)},
            "A":    {"color": CH_COLOR["A"],    "data": deque(maxlen=GRAPH_N),
                     "visible": tk.BooleanVar(value=True)},
            "TC":   {"color": CH_COLOR["TC"],   "data": deque(maxlen=GRAPH_N),
                     "visible": tk.BooleanVar(value=True)},
            "TH":   {"color": CH_COLOR["TH"],   "data": deque(maxlen=GRAPH_N),
                     "visible": tk.BooleanVar(value=False)},
            "W":    {"color": CH_COLOR["W"],    "data": deque(maxlen=GRAPH_N),
                     "visible": tk.BooleanVar(value=True)},
        }

        self.stat_hist = {
            k: deque(maxlen=120)
            for k in ["freq_out", "dc_bus", "output_curr", "discharge_t", "power_w", "output_volt"]
        }

        self._build_ui()
        self._chart_tick()
        self._clock_tick()

    # ══════════════════════════════════════════════════════════════════════════
    # UI inşası
    # ══════════════════════════════════════════════════════════════════════════
    def _build_ui(self):
        self._build_header()
        body = tk.Frame(self.root, bg=BG)
        body.pack(fill="both", expand=True, padx=6, pady=4)
        left = tk.Frame(body, bg=BG, width=372)
        left.pack(side="left", fill="y")
        left.pack_propagate(False)
        right = tk.Frame(body, bg=BG)
        right.pack(side="left", fill="both", expand=True, padx=(6, 0))
        self._build_left(left)
        self._build_right(right)
        self._build_statusbar()

    def _build_header(self):
        hdr = tk.Frame(self.root, bg=BLACK, height=46)
        hdr.pack(fill="x"); hdr.pack_propagate(False)
        tk.Label(hdr, text="  ÜmitTez", font=("Consolas", 15, "bold"),
                 bg=BLACK, fg=ACCENT).pack(side="left", padx=(10, 0))
        tk.Label(hdr, text="  │  SIAM SDP POWER DRIVER  —  MODBUS RTU  v2.1",
                 font=("Consolas", 9), bg=BLACK, fg=DIM).pack(side="left")
        self.lbl_conn = tk.Label(hdr, text="⬤ BAĞLI DEĞİL",
                                 font=FBT, bg=BLACK, fg=DANGER)
        self.lbl_conn.pack(side="right", padx=14)
        self.lbl_clock = tk.Label(hdr, text="", font=FS, bg=BLACK, fg=DIM)
        self.lbl_clock.pack(side="right", padx=6)

    def _build_left(self, p):
        self._section_conn(p)
        self._section_ctrl(p)
        self._section_speed(p)
        self._section_alarms(p)
        self._section_log(p)

    def _section_conn(self, p):
        f = self._card(p, "▸ BAĞLANTI AYARLARI", ACCENT)
        r1 = tk.Frame(f, bg=PANEL); r1.pack(fill="x", pady=2)
        self._lbl(r1, "Port:").pack(side="left")
        tk.Entry(r1, textvariable=self.v_port, width=7,
                 **self._ekw()).pack(side="left", padx=4)
        self._lbl(r1, "Baud:").pack(side="left")
        ttk.Combobox(r1, textvariable=self.v_baud, width=8,
                     values=[1200,2400,9600,19200,38400,57600,115200],
                     state="readonly", font=FM).pack(side="left", padx=4)
        r2 = tk.Frame(f, bg=PANEL); r2.pack(fill="x", pady=2)
        self._lbl(r2, "Slave:").pack(side="left")
        tk.Entry(r2, textvariable=self.v_slave, width=4,
                 **self._ekw()).pack(side="left", padx=4)
        self._lbl(r2, "Parity:").pack(side="left")
        ttk.Combobox(r2, textvariable=self.v_parity, width=3,
                     values=["N","E","O"], state="readonly",
                     font=FM).pack(side="left", padx=4)
        self._lbl(r2, "Stop:").pack(side="left")
        ttk.Combobox(r2, textvariable=self.v_stop, width=2,
                     values=[1,2], state="readonly",
                     font=FM).pack(side="left", padx=4)
        self.btn_conn = self._btn(f, "⬡  BAĞLAN", ACCENT,
                                  self._toggle_connect, fg=BLACK)
        self.btn_conn.pack(fill="x", pady=(6, 2))

    def _section_ctrl(self, p):
        f = self._card(p, "▸ ÇALIŞMA KONTROL", ACCENT2)
        row = tk.Frame(f, bg=PANEL); row.pack(fill="x")
        self._btn(row, "▶ BAŞLAT",  GREEN,  self._cmd_start).pack(side="left", padx=(0,3), pady=2)
        self._btn(row, "■ DURDUR",  DANGER, self._cmd_stop).pack(side="left",  padx=(0,3), pady=2)
        self._btn(row, "⚡ ACİL",   WARN,   self._cmd_emerg, fg=BLACK).pack(side="left", padx=(0,3), pady=2)
        self._btn(row, "↺ RESET",   DIM,    self._cmd_reset, fg=TEXT).pack(side="left", pady=2)
        self.lbl_op   = tk.Label(f, text="Durum : —", font=FLB,
                                 bg=PANEL, fg=DIM, anchor="w")
        self.lbl_op.pack(fill="x", pady=(4, 1))
        self.lbl_trip = tk.Label(f, text="Trip   : —", font=FLB,
                                  bg=PANEL, fg=DIM, anchor="w")
        self.lbl_trip.pack(fill="x")

    def _section_speed(self, p):
        f = self._card(p, "▸ HIZ AYARI  [45 – 390 Hz]", BLUE)
        self.v_speed = tk.DoubleVar(value=45.0)
        top = tk.Frame(f, bg=PANEL); top.pack(fill="x")
        self.lbl_hz = tk.Label(top, text="45.00",
                               font=FB, bg=PANEL, fg=ACCENT, width=7)
        self.lbl_hz.pack(side="left")
        tk.Label(top, text=" Hz", font=("Consolas",13), bg=PANEL, fg=DIM).pack(side="left")
        self.slider = tk.Scale(f, from_=45.0, to=390.0, resolution=0.5,
                               orient="horizontal", variable=self.v_speed,
                               bg=PANEL, fg=TEXT, troughcolor=BORDER,
                               highlightthickness=0, sliderlength=16,
                               activebackground=ACCENT, showvalue=False,
                               command=self._on_slider)
        self.slider.pack(fill="x", pady=(2, 4))
        tk.Label(f, text="  ↑ Slider hareket ettirince otomatik Hz komutu gönderir",
                 font=("Consolas", 7), bg=PANEL, fg=DIM).pack(anchor="w")
        mr = tk.Frame(f, bg=PANEL); mr.pack(fill="x", pady=2)
        self._lbl(mr, "Manuel Hz:").pack(side="left")
        self.entry_hz = tk.Entry(mr, width=8, **self._ekw())
        self.entry_hz.pack(side="left", padx=4)
        self.entry_hz.bind("<Return>", lambda e: self._apply_manual())
        self._btn(mr, "UYGULA", ACCENT, self._apply_manual,
                  padx=6, pady=2, fg=BLACK).pack(side="left")
        cr = tk.Frame(f, bg=PANEL); cr.pack(fill="x", pady=2)
        self._lbl(cr, "cos φ :").pack(side="left")
        tk.Scale(cr, from_=0.5, to=1.0, resolution=0.01,
                 orient="horizontal", variable=self.v_cosphi,
                 bg=PANEL, fg=TEXT, troughcolor=BORDER,
                 highlightthickness=0, sliderlength=14,
                 activebackground=GREEN, showvalue=True,
                 length=180, font=FT2).pack(side="left", padx=4)
        self._btn(f, "💾  PARAMETRELERİ KALICI KAYDET  (reg 0313)",
                  WARN, self._save_params, padx=6, pady=5,
                  fg=BLACK).pack(fill="x", pady=(6, 0))

    def _section_alarms(self, p):
        f = self._card(p, "▸ ALARM EŞİKLERİ", DANGER)
        specs = [
            ("Basma Sıcaklığı °C", "discharge_t",  0, 130),
            ("Soğutucu Sıcak. °C", "heatsink_t",   0, 130),
            ("Akım Limiti  A",      "output_curr",  0, 100),
            ("DC Bus Maks  V",      "dc_bus",        0, 800),
        ]
        for label, key, lo, hi in specs:
            row = tk.Frame(f, bg=PANEL); row.pack(fill="x", pady=1)
            tk.Label(row, text=label, font=FT2, bg=PANEL, fg=DIM,
                     width=22, anchor="w").pack(side="left")
            tk.Scale(row, from_=lo, to=hi, resolution=1,
                     orient="horizontal", variable=self.alarms[key]["limit"],
                     bg=PANEL, fg=TEXT, troughcolor=BORDER,
                     highlightthickness=0, sliderlength=12,
                     activebackground=DANGER, showvalue=True,
                     length=110, font=FT2).pack(side="left")

    def _section_log(self, p):
        f = self._card(p, "▸ VERİ KAYIT  (CSV)", DIM)
        self.btn_log = self._btn(f, "⏺  KAYDI BAŞLAT", GREEN,
                                 self._toggle_log, pady=4)
        self.btn_log.pack(fill="x", pady=(0, 4))
        row = tk.Frame(f, bg=PANEL); row.pack(fill="x", pady=2)
        self._btn(row, "📂  Farklı Kaydet", BLUE,
                  self._save_as, padx=6, pady=3, fg=BLACK).pack(side="left", padx=(0, 4))
        self._btn(row, "🗂  Dosyayı Göster", PANEL2,
                  self._open_folder, padx=6, pady=3, fg=TEXT).pack(side="left")
        self.lbl_log = tk.Label(f, text="Henüz kayıt başlatılmadı.",
                                font=FT2, bg=PANEL, fg=DIM,
                                anchor="w", wraplength=340)
        self.lbl_log.pack(fill="x", pady=(4, 0))

    # ── Sağ panel ─────────────────────────────────────────────────────────────
    def _build_right(self, parent):
        gauge_row = tk.Frame(parent, bg=BG)
        gauge_row.pack(fill="x", pady=(0, 4))
        self._build_gauges(gauge_row)

        nb = ttk.Notebook(parent)
        nb.pack(fill="both", expand=True)

        tab_main = tk.Frame(nb, bg=BG)
        tab_all  = tk.Frame(nb, bg=BG)
        nb.add(tab_main, text="  📊 Ana Panel  ")
        nb.add(tab_all,  text="  🗂 Tüm Modbus Verileri  ")

        self._build_main_tab(tab_main)
        self._build_all_data_tab(tab_all)

    def _build_gauges(self, parent):
        specs = [
            ("Çıkış Hz",     "freq_out",    "Hz",   0,   400, 350, 390,  ACCENT),
            ("DC Bus",        "dc_bus",      "V",    0,   800, 700, 760,  BLUE),
            ("Akım",          "output_curr", "A",    0,   100,  40,  80,  WARN),
            ("Basma Sıcak.",  "discharge_t", "°C", -30,  130,  90, 110,  ACCENT2),
            ("Soğutucu Sıc.", "heatsink_t",  "°C", -30,  130,  70,  90,  PURPLE),
            ("Güç",           "power_w",     "W",    0, 25000,18000,22000,GREEN),
        ]
        self.gauges = {}
        for label, key, unit, vmin, vmax, warn, danger, color in specs:
            g = GaugeMeter(parent, label, unit, vmin, vmax,
                           warn, danger, size=118, color=color)
            g.pack(side="left", padx=3, pady=2)
            self.gauges[key] = g

        # Output Voltage + Duty — özel gauge (DutyGauge)
        self.duty_gauge = DutyGauge(parent, size=118)
        self.duty_gauge.pack(side="left", padx=3, pady=2)

    def _build_main_tab(self, parent):
        # Duty cycle açıklama bandı
        duty_info = tk.Frame(parent, bg=PANEL2,
                             highlightbackground=BORDER, highlightthickness=1)
        duty_info.pack(fill="x", pady=(4, 2))
        self.lbl_duty_info = tk.Label(
            duty_info,
            text="Çıkış Gerilimi: — V   |   DC Bus: — V   |   Modülasyon İndeksi (mₐ): —   |   Kestirilen Duty: —%",
            font=("Consolas", 8, "bold"), bg=PANEL2, fg=CYAN, anchor="w", padx=8, pady=3)
        self.lbl_duty_info.pack(fill="x")
        tk.Label(duty_info,
                 text="  mₐ = Vout_LL_rms × 2√2 / (Vdc × √3)   │   mₐ≤1: Lineer SPWM   mₐ>1: Aşırı Modülasyon (6-basamak yaklaşımı)",
                 font=("Consolas", 7), bg=PANEL2, fg=DIM, anchor="w", padx=8).pack(fill="x")

        stat_frame = tk.Frame(parent, bg=PANEL,
                              highlightbackground=BORDER, highlightthickness=1)
        stat_frame.pack(fill="x", pady=(2, 4))
        self._build_stats(stat_frame)

        chart_wrap = tk.Frame(parent, bg=PANEL,
                              highlightbackground=BORDER, highlightthickness=1)
        chart_wrap.pack(fill="both", expand=True)

        toolbar = tk.Frame(chart_wrap, bg=PANEL)
        toolbar.pack(fill="x", padx=6, pady=(4, 2))
        tk.Label(toolbar, text="GRAFİK KANALLARI :", font=FT2, bg=PANEL, fg=DIM).pack(side="left")
        for name, ch in self.ch.items():
            tk.Checkbutton(toolbar,
                           text=f" {CH_LABEL[name]}",
                           variable=ch["visible"],
                           bg=PANEL, fg=ch["color"],
                           selectcolor=BLACK,
                           activebackground=PANEL,
                           font=FT2).pack(side="left", padx=3)

        self.chart = MultiAxisChart(chart_wrap, self.ch, n=GRAPH_N)
        self.chart.pack(fill="both", expand=True, padx=4, pady=(0, 4))

    def _build_stats(self, parent):
        cols       = ["PARAMETRE", "ANLK", "MİN", "MAKS", "ORT", "BİRİM"]
        col_colors = [DIM, TEXT, BLUE, ACCENT2, GREEN, DIM]
        for ci, (h, c) in enumerate(zip(cols, col_colors)):
            tk.Label(parent, text=h, font=FT2, bg=PANEL, fg=c,
                     width=12, anchor="center").grid(row=0, column=ci, padx=2, pady=2)
        rows = [
            ("Çıkış Frekansı", "freq_out",    "Hz",  2),
            ("DC Bus Voltajı", "dc_bus",       "V",   0),
            ("Çıkış Akımı",    "output_curr",  "A",   1),
            ("Basma Sıcaklık", "discharge_t",  "°C",  1),
            ("Güç (hesaplı)",  "power_w",      "W",   1),
            ("Çıkış Gerilimi", "output_volt",  "V",   0),
        ]
        self.stat_sv = {}
        for ri, (label, key, unit, dec) in enumerate(rows, 1):
            tk.Label(parent, text=label, font=FT2, bg=PANEL, fg=TEXT,
                     width=15, anchor="w").grid(row=ri, column=0, padx=4, pady=1)
            svs = {}
            for ci, col in enumerate(["live", "min", "max", "avg"], 1):
                sv = tk.StringVar(value="—")
                tk.Label(parent, textvariable=sv,
                         font=("Consolas", 9, "bold"), bg=PANEL,
                         fg=[TEXT, BLUE, ACCENT2, GREEN][ci-1],
                         width=9, anchor="e").grid(row=ri, column=ci, padx=2)
                svs[col] = sv
            tk.Label(parent, text=unit, font=FT2, bg=PANEL, fg=DIM,
                     anchor="w").grid(row=ri, column=5, padx=2)
            self.stat_sv[key] = (svs, dec)

    # ── Tüm Modbus Verileri sekmesi (YAZMA DESTEKLİ) ─────────────────────────
    def _build_all_data_tab(self, parent):
        """
        Her register için:
          - Salt okunur olanlar: sadece değer gösterir
          - Yazılabilir olanlar: Entry + YAZ butonu (inline)
        """
        canvas = tk.Canvas(parent, bg=BG, highlightthickness=0)
        vsb    = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)

        inner = tk.Frame(canvas, bg=BG)
        win   = canvas.create_window((0, 0), window=inner, anchor="nw")

        def _on_resize(e):
            canvas.itemconfig(win, width=e.width)
        canvas.bind("<Configure>", _on_resize)
        inner.bind("<Configure>", lambda e: canvas.configure(
            scrollregion=canvas.bbox("all")))

        def _scroll(e):
            canvas.yview_scroll(int(-1*(e.delta/120)), "units")
        canvas.bind_all("<MouseWheel>", _scroll)

        # Başlık
        hdr_cols = ["Adres", "Ad", "Anlık Değer", "Birim", "Yaz (Enter/Buton)", "Açıklama"]
        hdr_w    = [6,        22,   12,             6,       20,                  30]
        for ci, (h, w) in enumerate(zip(hdr_cols, hdr_w)):
            tk.Label(inner, text=h, font=FBT, bg=PANEL2, fg=ACCENT,
                     width=w, anchor="w", padx=4, pady=3).grid(
                         row=0, column=ci, sticky="ew", padx=1, pady=1)

        # ── Register tanımları ──────────────────────────────────────────────
        # (adres_str, isim, live_key, birim, açıklama, fmt_fn,
        #  yazılabilir?, modbus_addr, raw_scale_wr)
        # raw_scale_wr: kayıt değeri = float * raw_scale_wr
        # None → integer gönder
        self._all_rows_def = [
            # ── Grup 01: Komut (Yazılabilir) ──
            ("─── GRUP 01: Komut Yazmaçları (Yazılabilir) ──────────────────────────────────────────────────────────", None, None, None, None, None, False, None, None),
            ("0101", "Frekans Komutu",  "freq_ro",   "Hz×100","Hedef frekans (değer = Hz × 100)",
             lambda v: f"{v:.2f}",
             True, A_FREQ_CMD, 100.0),
            ("0102", "Kontrol Komutu", "op_raw",    "kod",   "0=Dur 1=Çalış 2=AcilDur 4=Reset",
             lambda v: f"0x{int(v):04X}",
             True, A_CTRL_CMD, None),

            # ── Grup 02: Çalışma Verileri (Salt Okunur) ──
            ("─── GRUP 02: Çalışma Verileri (Salt Okunur) ──────────────────────────────────────────────────────────", None, None, None, None, None, False, None, None),
            ("0201", "İşletme Durumu", "op_raw",      "bits",  "Bit alanı — 0:Çalış 1:Hata 2:SabitHz …",
             lambda v: f"0x{int(v):04X}", False, None, None),
            ("0202", "Koruma Durumu",  "prot_raw",    "kod",   "0=Başlatma 1=Normal 2=Kontrol …",
             lambda v: PROT_NAMES.get(int(v)&0xFF, str(int(v)&0xFF)), False, None, None),
            ("0203", "Trip Tipi",      "trip_raw",    "kod",   "0=Normal, diğerleri hata kodları",
             lambda v: TRIP_NAMES.get(int(v), f"Kod:{int(v)}"), False, None, None),
            ("0204", "Çıkış Frekansı", "freq_out",    "Hz",    "Kompresöre giden çıkış frekansı",
             lambda v: f"{v:.2f}", False, None, None),
            ("0205", "Hız Tahmini",    "speed_est",   "Hz",    "Kompresör rotor hızı tahmini",
             lambda v: f"{v:.2f}", False, None, None),
            ("0206", "DC Bus Gerilim", "dc_bus",      "Vdc",   "Ana devre DC bara gerilimi",
             lambda v: f"{v:.0f}", False, None, None),
            ("0207", "Çıkış Akımı",    "output_curr", "Arms",  "Kompresör gerçek akım",
             lambda v: f"{v:.1f}", False, None, None),
            ("0208", "Çıkış Gerilimi", "output_volt", "Vrms",  "Çıkış line-to-line gerilimi (PWM'den türetilmiş)",
             lambda v: f"{v:.0f}", False, None, None),
            ("0209", "Soğutucu Sıcak.","heatsink_t",  "°C",    "Soğutucu gerçek sıcaklık",
             lambda v: f"{v:.1f}", False, None, None),
            ("0210", "Basma Sıcaklığı","discharge_t", "°C",    "Kompresör basma sıcaklığı",
             lambda v: f"{v:.1f}", False, None, None),
            ("0211", "Hz Komutu (RO)", "freq_ro",     "Hz",    "Sürücüye gönderilen frekans komutu",
             lambda v: f"{v:.2f}", False, None, None),

            # ── Hesaplanan ──
            ("─── HESAPLANAN DEĞERLER ───────────────────────────────────────────────────────────────────────────────", None, None, None, None, None, False, None, None),
            ("—",    "Güç (W)",         "power_w",     "W",     "P = √3 × Vout × I × cosφ",
             lambda v: f"{v:.1f}", False, None, None),
            ("—",    "Güç (kW)",        "power_kw",    "kW",    "P = √3 × Vout × I × cosφ / 1000",
             lambda v: f"{v:.3f}", False, None, None),
            ("—",    "Mod. İndeksi mₐ", "duty_pct",    "%×0.01","mₐ = Vout×2√2/(Vdc×√3) × 100",
             lambda v: f"{v:.1f}", False, None, None),

            # ── Grup 03: Sürücü Ayarları (Yazılabilir) ──
            ("─── GRUP 03: Sürücü Ayarları (Yazılabilir ✎) ────────────────────────────────────────────────────────", None, None, None, None, None, False, None, None),
            ("0301", "Çalışma Modu",   "op_mode",     "kod",   "1=MODBUS 3=Ext+MODBUS 5=ExtTerminal",
             lambda v: str(int(v)), True, A_OP_MODE, None),
            ("0302", "Kompresör Model","comp_model",  "kod",   "Motor tablo numarası",
             lambda v: str(int(v)), True, A_COMP_MODEL, None),
            ("0303", "Tutma Frekansı", "hold_freq",   "Hz",    "Başlangıç tutma frekansı (×100)",
             lambda v: f"{v:.2f}", True, A_HOLD_FREQ, 100.0),
            ("0304", "Tutma Süresi",   "hold_time",   "sn",    "Tutma frekansı bekleme süresi",
             lambda v: str(int(v)), True, A_HOLD_TIME, None),
            ("0305", "Maks Hz Limiti", "max_freq",    "Hz",    "Maksimum çıkış frekansı sınırı (×100)",
             lambda v: f"{v:.2f}", True, A_MAX_FREQ, 100.0),
            ("0306", "Min Hz Limiti",  "min_freq",    "Hz",    "Minimum çıkış frekansı sınırı (×100)",
             lambda v: f"{v:.2f}", True, A_MIN_FREQ, 100.0),
            ("0309", "Durdurma Hz",    "stop_freq",   "Hz",    "Sürücünün durduğu frekans (×100)",
             lambda v: f"{v:.2f}", True, A_STOP_FREQ, 100.0),
            ("0310", "Kontrol Sıcak.", "ctrl_temp",   "°C",    "Sıcaklık kontrol bölgesi limiti",
             lambda v: str(int(v)), True, A_CTRL_TEMP, None),
            ("0311", "Koruma Sıcak.",  "prot_temp",   "°C",    "Sıcaklık koruma bölgesi limiti",
             lambda v: str(int(v)), True, A_PROT_TEMP, None),
            ("0312", "Trip Sıcaklığı", "trip_temp",   "°C",    "Sıcaklık trip limiti",
             lambda v: str(int(v)), True, A_TRIP_TEMP, None),
            ("0313", "Parametre Kayıt","op_raw",      "—",     "1 yaz → EEPROM'a kaydet",
             lambda v: "—", True, A_SAVE, None),
            ("0315", "Stall Koruması", "stall",       "0/1",   "0=Pasif 1=Etkin",
             lambda v: "Etkin" if int(v) else "Pasif", True, A_STALL, None),
            ("0325", "Motor Isıtma",   "heat_en",     "kod",   "0=Kapalı 1=Ortam 2=Manuel",
             lambda v: ["Kapalı","Ortam","Manuel"][int(v)] if int(v) in [0,1,2] else str(int(v)),
             True, A_HEAT_EN, None),
            ("0326", "Isıtma AktSıc.", "heat_temp",   "°C",    "Ortam modu aktif sıcaklık (×10)",
             lambda v: f"{v:.1f}", True, A_HEAT_TEMP, 10.0),
            ("0327", "Isıtma Güç",     "heat_pow",    "W",     "Isıtma güç kontrolü (0-60W)",
             lambda v: str(int(v)), True, A_HEAT_POW, None),

            # ── Grup 05: Protokol ──
            ("─── GRUP 05: Protokol Ayarları (Yazılabilir ✎) ──────────────────────────────────────────────────────", None, None, None, None, None, False, None, None),
            ("0501", "Baud Rate",      "baud_code",   "kod",   "0=1200 1=2400 2=9600 3=12800 4=19200 5=38400 6=57600 7=115200",
             lambda v: ["1200","2400","9600","12800","19200","38400","57600","115200"][min(int(v),7)],
             True, A_BAUD, None),
            ("0502", "Veri Formatı",   "fmt_code",    "kod",   "0=8N1 1=8E1 2=8O1 3=8N2 4=8E2 5=8O2",
             lambda v: ["8N1","8E1","8O1","8N2","8E2","8O2"][min(int(v),5)],
             True, A_FORMAT, None),
            ("0503", "Zaman Aşımı",    "timeout_en",  "0/1",   "0=Pasif 1=Etkin",
             lambda v: "Etkin" if int(v) else "Pasif", True, A_TIMEOUT, None),
            ("0504", "Slave Adresi",   "slave_addr",  "—",     "RS-485 düğüm adresi (1-247)",
             lambda v: str(int(v)), True, A_SLAVE, None),

            # ── Grup 99: Bilgi ──
            ("─── GRUP 99: Bilgi (Salt Okunur) ─────────────────────────────────────────────────────────────────────", None, None, None, None, None, False, None, None),
            ("9901", "Yazılım Versiy.", "sw_ver",      "—",     "Sürücü yazılım versiyonu",
             lambda v: f"{int(v)>>8}.{int(v)&0xFF}", False, None, None),
            ("9902", "Param. Tablosu", "param_tbl",   "—",     "Parametre tablo versiyonu",
             lambda v: f"{int(v)>>8} / {int(v)&0xFF}", False, None, None),
        ]

        self._all_sv      = {}   # live_key → (StringVar, fmt_fn)
        self._write_vars  = {}   # modbus_addr → (Entry widget, scale)
        row_idx = 1

        for item in self._all_rows_def:
            addr, name, key, unit, desc, fmt, writable, mb_addr, scale = item

            if key is None:
                # Grup başlığı
                tk.Label(inner, text=addr, font=("Consolas", 8, "bold"),
                         bg=PANEL2, fg=ACCENT, anchor="w", padx=4).grid(
                             row=row_idx, column=0, columnspan=6,
                             sticky="ew", pady=(6, 2))
                row_idx += 1
                continue

            sv = tk.StringVar(value="—")
            self._all_sv[key] = (sv, fmt)
            bg = BLACK if row_idx % 2 == 0 else PANEL

            # Adres
            tk.Label(inner, text=addr, font=FT2, bg=bg, fg=DIM,
                     width=6, anchor="w", padx=4).grid(
                         row=row_idx, column=0, sticky="ew", padx=1, pady=1)
            # Ad
            clr = GREEN if writable else TEXT
            tk.Label(inner, text=name + (" ✎" if writable else ""),
                     font=FT2, bg=bg, fg=clr,
                     width=22, anchor="w", padx=4).grid(
                         row=row_idx, column=1, sticky="ew", padx=1, pady=1)
            # Anlık değer
            tk.Label(inner, textvariable=sv,
                     font=("Consolas", 9, "bold"), bg=bg,
                     fg=GREEN, width=12, anchor="e", padx=4).grid(
                         row=row_idx, column=2, sticky="ew", padx=1, pady=1)
            # Birim
            tk.Label(inner, text=unit, font=FT2, bg=bg, fg=DIM,
                     width=6, anchor="w", padx=4).grid(
                         row=row_idx, column=3, sticky="ew", padx=1, pady=1)

            # Yazma sütunu
            wr_frame = tk.Frame(inner, bg=bg)
            wr_frame.grid(row=row_idx, column=4, sticky="ew", padx=2, pady=1)
            if writable and mb_addr is not None:
                wr_entry = tk.Entry(wr_frame, width=8, **self._ekw())
                wr_entry.pack(side="left", padx=(0, 2))
                wr_btn = tk.Button(
                    wr_frame, text="YAZ", font=("Consolas", 7, "bold"),
                    bg=ACCENT2, fg=BLACK, relief="flat", padx=4, pady=1,
                    cursor="hand2",
                    command=lambda e=wr_entry, a=mb_addr, sc=scale: self._write_register(a, e, sc))
                wr_btn.pack(side="left")
                wr_entry.bind("<Return>",
                    lambda ev, e=wr_entry, a=mb_addr, sc=scale: self._write_register(a, e, sc))
                self._write_vars[mb_addr] = (wr_entry, scale)
            else:
                tk.Label(wr_frame, text="—", font=FT2, bg=bg, fg=DIM).pack(side="left")

            # Açıklama
            tk.Label(inner, text=desc, font=FT2, bg=bg, fg=DIM,
                     width=30, anchor="w", padx=4).grid(
                         row=row_idx, column=5, sticky="ew", padx=1, pady=1)

            row_idx += 1

    # ── Register yazma ────────────────────────────────────────────────────────
    def _write_register(self, mb_addr, entry_widget, scale):
        if not self._chk():
            return
        raw_text = entry_widget.get().strip()
        if not raw_text:
            messagebox.showwarning("Boş Değer", "Yazılacak değeri giriniz.")
            return
        try:
            fval = float(raw_text)
            if scale is not None:
                reg_val = int(round(fval * scale))
            else:
                reg_val = int(round(fval))
            if reg_val < 0 or reg_val > 65535:
                messagebox.showwarning("Aralık Hatası",
                    f"Register değeri 0-65535 arasında olmalı.\nHesaplanan: {reg_val}")
                return
            ok = self._wr(mb_addr, reg_val)
            if ok:
                self.sv_status.set(
                    f"✓ Yazıldı → Adres {mb_addr:04d}  değer={reg_val}  "
                    f"(girilen={fval}, ölçek={scale})")
                entry_widget.config(bg="#0d2a1a")  # yeşilimsi flash
                self.root.after(600, lambda: entry_widget.config(bg=BLACK))
            else:
                messagebox.showerror("Yazma Hatası", f"Adres {mb_addr:04d} yazılamadı.")
        except ValueError:
            messagebox.showerror("Geçersiz Değer",
                f"'{raw_text}' sayısal değil.\nOndalık için nokta kullanın.")

    def _build_statusbar(self):
        bar = tk.Frame(self.root, bg=BLACK, height=20)
        bar.pack(fill="x", side="bottom"); bar.pack_propagate(False)
        self.sv_status = tk.StringVar(value="Hazır")
        tk.Label(bar, textvariable=self.sv_status,
                 font=FT2, bg=BLACK, fg=DIM, anchor="w").pack(side="left", padx=8)
        tk.Label(bar, text="SDP-Series MODBUS  |  TDID21A008 RevD  |  ÜmitTez  v2.1",
                 font=FT2, bg=BLACK, fg=DIM).pack(side="right", padx=8)

    # ══════════════════════════════════════════════════════════════════════════
    # MODBUS
    # ══════════════════════════════════════════════════════════════════════════
    def _toggle_connect(self):
        if self.connected: self._disconnect()
        else:              self._do_connect()

    def _do_connect(self):
        if not MODBUS_OK:
            messagebox.showerror("Hata",
                "pymodbus kurulu değil!\n\n  pip install pymodbus")
            return
        try:
            self.client = ModbusClient(
                port=self.v_port.get(), baudrate=self.v_baud.get(),
                parity=self.v_parity.get(), stopbits=self.v_stop.get(),
                bytesize=8, timeout=2, retries=3)
            if self.client.connect():
                self.connected = True
                self.btn_conn.config(text="⬡  BAĞLANTIYI KES", bg=DANGER, fg=TEXT)
                self.lbl_conn.config(text="⬤ BAĞLI", fg=GREEN)
                self.sv_status.set(f"Bağlandı: {self.v_port.get()} "
                                   f"@ {self.v_baud.get()} baud  "
                                   f"slave={self.v_slave.get()}")
                self._stop_ev.clear()
                threading.Thread(target=self._data_loop, daemon=True).start()
            else:
                messagebox.showerror("Bağlantı Hatası",
                    f"Port '{self.v_port.get()}' açılamadı.")
        except Exception as e:
            messagebox.showerror("Bağlantı Hatası", str(e))

    def _disconnect(self):
        self._stop_ev.set()
        time.sleep(0.4)
        try:
            if self.client: self.client.close()
        except Exception:
            pass
        self.connected = False
        self.btn_conn.config(text="⬡  BAĞLAN", bg=ACCENT, fg=BLACK)
        self.lbl_conn.config(text="⬤ BAĞLI DEĞİL", fg=DANGER)
        self.sv_status.set("Bağlantı kesildi.")

    def _rd(self, addr):
        try:
            r = self.client.read_holding_registers(
                address=addr, count=1, device_id=self.v_slave.get())
            if r and not r.isError():
                return r.registers[0]
        except Exception:
            pass
        return None

    def _wr(self, addr, val):
        try:
            self.client.write_register(
                address=addr, value=int(val), device_id=self.v_slave.get())
            return True
        except Exception:
            return False

    # ── Kontrol ───────────────────────────────────────────────────────────────
    def _cmd_start(self):
        if self._chk(): self._wr(A_CTRL_CMD, 0x0001); self.sv_status.set("→ BAŞLAT (0102←1)")
    def _cmd_stop(self):
        if self._chk(): self._wr(A_CTRL_CMD, 0x0000); self.sv_status.set("→ DURDUR (0102←0)")
    def _cmd_emerg(self):
        if self._chk(): self._wr(A_CTRL_CMD, 0x0002); self.sv_status.set("→ ACİL DURDURMA (0102←2)")
    def _cmd_reset(self):
        if self._chk(): self._wr(A_CTRL_CMD, 0x0004); self.sv_status.set("→ HATA SIFIRLA (0102←4)")

    def _on_slider(self, val):
        hz = float(val)
        self.lbl_hz.config(text=f"{hz:.2f}")
        if self._slider_after:
            self.root.after_cancel(self._slider_after)
        self._slider_after = self.root.after(400, self._slider_send, hz)

    def _slider_send(self, hz):
        self._slider_after = None
        if self.connected:
            self._send_hz(hz)

    def _apply_manual(self):
        if not self._chk(): return
        try:
            hz = float(self.entry_hz.get())
            if 45.0 <= hz <= 390.0:
                self.v_speed.set(hz)
                self.slider.set(hz)
                self.lbl_hz.config(text=f"{hz:.2f}")
                self._send_hz(hz)
            else:
                messagebox.showwarning("Geçersiz Değer", "45.0 – 390.0 Hz arası!")
        except ValueError:
            messagebox.showwarning("Geçersiz Giriş", "Sayısal değer giriniz.")

    def _send_hz(self, hz):
        reg = int(round(hz * 100))
        self._wr(A_FREQ_CMD, reg)
        self.sv_status.set(f"Hz komutu: {hz:.2f} Hz  (0101←{reg})")

    def _save_params(self):
        if not self._chk(): return
        self._wr(A_SAVE, 1)
        messagebox.showinfo("Parametre Kayıt", "Parametreler kalıcı hafızaya kaydedildi. (0313←1)")
        self.sv_status.set("Parametreler kalıcı kaydedildi  (0313←1)")

    def _chk(self):
        if not self.connected:
            messagebox.showwarning("Bağlantı Yok", "Önce MODBUS bağlantısı kurun.")
            return False
        return True

    # ══════════════════════════════════════════════════════════════════════════
    # Veri döngüsü
    # ══════════════════════════════════════════════════════════════════════════
    def _data_loop(self):
        while not self._stop_ev.is_set():
            try:
                rd = self._rd

                def sc(v, div=1.0, signed=False):
                    if v is None: return 0.0
                    if signed and v > 32767: v -= 65536
                    return v / div

                raw_fo = rd(A_FREQ_OUT);   raw_se = rd(A_SPEED_EST)
                raw_dc = rd(A_DC_BUS);     raw_ic = rd(A_OUT_CURR)
                raw_ov = rd(A_OUT_VOLT);   raw_hs = rd(A_HEATSINK)
                raw_di = rd(A_DISCHARGE);  raw_fr = rd(A_FREQ_RO)
                raw_op = rd(A_OP_STAT);    raw_pr = rd(A_PROT_STAT)
                raw_tr = rd(A_TRIP)

                freq_out    = sc(raw_fo, 100.0)
                speed_est   = sc(raw_se, 100.0)
                dc_bus      = sc(raw_dc, 1.0)
                out_curr    = sc(raw_ic, 10.0)
                out_volt    = sc(raw_ov, 1.0)   # Vrms LL
                heatsink_t  = sc(raw_hs, 10.0, True)
                discharge_t = sc(raw_di, 10.0, True)
                freq_ro     = sc(raw_fr, 100.0)

                cp       = self.v_cosphi.get()
                power_w  = math.sqrt(3) * out_volt * out_curr * cp
                power_kw = power_w / 1000.0

                # Modülasyon indeksi (mₐ)
                if dc_bus > 10:
                    ma = (out_volt * 2.0 * math.sqrt(2)) / (dc_bus * math.sqrt(3))
                else:
                    ma = 0.0
                duty_pct = ma * 100.0  # % olarak

                raw_om  = rd(A_OP_MODE);   raw_cm = rd(A_COMP_MODEL)
                raw_hf  = rd(A_HOLD_FREQ); raw_ht = rd(A_HOLD_TIME)
                raw_mxf = rd(A_MAX_FREQ);  raw_mnf = rd(A_MIN_FREQ)
                raw_sf  = rd(A_STOP_FREQ); raw_ct = rd(A_CTRL_TEMP)
                raw_pt  = rd(A_PROT_TEMP); raw_tt = rd(A_TRIP_TEMP)
                raw_st  = rd(A_STALL);     raw_he = rd(A_HEAT_EN)
                raw_htp = rd(A_HEAT_TEMP); raw_hp = rd(A_HEAT_POW)
                raw_bd  = rd(A_BAUD);      raw_fm = rd(A_FORMAT)
                raw_to  = rd(A_TIMEOUT);   raw_sa = rd(A_SLAVE)
                raw_sv  = rd(A_SW_VER);    raw_pt2 = rd(A_PARAM_TBL)

                self.live.update({
                    "freq_out": freq_out,   "speed_est": speed_est,
                    "dc_bus": dc_bus,       "output_curr": out_curr,
                    "output_volt": out_volt, "heatsink_t": heatsink_t,
                    "discharge_t": discharge_t, "freq_ro": freq_ro,
                    "power_w": power_w,     "power_kw": power_kw,
                    "duty_pct": duty_pct,
                    "op_raw": raw_op or 0,  "prot_raw": raw_pr or 0,
                    "trip_raw": raw_tr or 0,
                    "op_mode":    raw_om  or 0,
                    "comp_model": raw_cm  or 0,
                    "hold_freq":  sc(raw_hf,  100.0),
                    "hold_time":  raw_ht  or 0,
                    "max_freq":   sc(raw_mxf, 100.0),
                    "min_freq":   sc(raw_mnf, 100.0),
                    "stop_freq":  sc(raw_sf,  100.0),
                    "ctrl_temp":  raw_ct  or 0,
                    "prot_temp":  raw_pt  or 0,
                    "trip_temp":  raw_tt  or 0,
                    "stall":      raw_st  or 0,
                    "heat_en":    raw_he  or 0,
                    "heat_temp":  sc(raw_htp, 10.0),
                    "heat_pow":   raw_hp  or 0,
                    "baud_code":  raw_bd  or 0,
                    "fmt_code":   raw_fm  or 0,
                    "timeout_en": raw_to  or 0,
                    "slave_addr": raw_sa  or 0,
                    "sw_ver":     raw_sv  or 0,
                    "param_tbl":  raw_pt2 or 0,
                })

                for k in self.stat_hist:
                    self.stat_hist[k].append(self.live[k])

                self.ch["Hz"]["data"].append(freq_out)
                self.ch["V"]["data"].append(dc_bus)
                self.ch["Vout"]["data"].append(out_volt)
                self.ch["A"]["data"].append(out_curr)
                self.ch["TC"]["data"].append(discharge_t)
                self.ch["TH"]["data"].append(heatsink_t)
                self.ch["W"]["data"].append(power_w)

                if self.is_logging and self.csv_writer:
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    self.csv_writer.writerow([
                        ts,
                        f"{freq_out:.2f}", f"{speed_est:.2f}",
                        f"{dc_bus:.0f}", f"{out_curr:.1f}",
                        f"{out_volt:.0f}", f"{heatsink_t:.1f}",
                        f"{discharge_t:.1f}", f"{freq_ro:.2f}",
                        f"{power_w:.1f}", f"{power_kw:.3f}",
                        f"{duty_pct:.2f}",
                        raw_op or 0, raw_pr or 0, raw_tr or 0,
                    ])
                    self.log_file.flush()
                    self.log_rows += 1

                self.root.after(0, self._update_ui)
                self._check_alarms()
            except Exception as e:
                self.root.after(0, self.sv_status.set, f"Veri hatası: {str(e)[:80]}")
            time.sleep(1.0)

    def _update_ui(self):
        d = self.live

        for key, g in self.gauges.items():
            g.set(d[key])

        # Duty gauge
        self.duty_gauge.set(d["output_volt"], d["dc_bus"])

        # Duty bilgi bandı
        vout = d["output_volt"]; vdc = d["dc_bus"]
        ma   = d["duty_pct"] / 100.0
        mode = "Lineer SPWM" if ma <= 1.0 else "⚠ Aşırı Modülasyon"
        self.lbl_duty_info.config(
            text=(f"Çıkış Gerilimi: {vout:.0f} V   |   DC Bus: {vdc:.0f} V   "
                  f"|   mₐ = {ma:.3f}   |   Duty ≈ {d['duty_pct']:.1f}%   [{mode}]"),
            fg=ACCENT2 if ma > 1.0 else (WARN if ma > 0.9 else CYAN))

        for key, (svs, dec) in self.stat_sv.items():
            hist = list(self.stat_hist[key])
            svs["live"].set(f"{d[key]:.{dec}f}")
            if hist:
                svs["min"].set(f"{min(hist):.{dec}f}")
                svs["max"].set(f"{max(hist):.{dec}f}")
                svs["avg"].set(f"{sum(hist)/len(hist):.{dec}f}")

        op = d["op_raw"]; bits = []
        if op & 0x001: bits.append("Çalışıyor")
        if op & 0x002: bits.append("⚠HATA")
        if op & 0x004: bits.append("SabitHz")
        if op & 0x008: bits.append("Hızlanıyor")
        if op & 0x010: bits.append("Yavaşlıyor")
        if op & 0x020: bits.append("Koruma")
        if op & 0x040: bits.append("Kontrol")
        if op & 0x080: bits.append("Normal")
        if op & 0x100: bits.append("Isıtma")
        if op & 0x200: bits.append("PFC Arıza")
        op_str = "  ".join(bits) if bits else "Boşta"
        self.lbl_op.config(
            text=f"Durum : {op_str}",
            fg=DANGER if (op & 0x02) else (GREEN if (op & 0x01) else DIM))

        trip = d["trip_raw"]
        self.lbl_trip.config(
            text=f"Trip   : {TRIP_NAMES.get(trip, f'Kod:{trip}')}",
            fg=DANGER if trip != 0 else GREEN)

        # Tüm Veriler sekmesi
        for key, (sv, fmt) in self._all_sv.items():
            v = self.live.get(key, 0)
            try:
                sv.set(fmt(v))
            except Exception:
                sv.set(str(v))

        if self.is_logging:
            self.lbl_log.config(
                text=f"⏺  {self.log_rows} satır kaydediliyor…\n{self.log_path}",
                fg=DANGER)

    def _check_alarms(self):
        vals = {
            "discharge_t": self.live["discharge_t"],
            "heatsink_t":  self.live["heatsink_t"],
            "output_curr": self.live["output_curr"],
            "dc_bus":      self.live["dc_bus"],
        }
        for key, val in vals.items():
            limit = self.alarms[key]["limit"].get()
            was   = self.alarms[key]["fired"]
            active = val >= limit
            self.alarms[key]["fired"] = active
            if active and not was:
                self.root.after(0, self._alarm_flash, key, val, limit)

    def _alarm_flash(self, key, val, limit):
        names = {
            "discharge_t": "Basma Sıcaklığı",
            "heatsink_t":  "Soğutucu Sıcaklığı",
            "output_curr": "Çıkış Akımı",
            "dc_bus":      "DC Bus Voltajı",
        }
        msg = f"⚠  ALARM: {names.get(key, key)}  Ölçülen: {val:.1f}  Eşik: {limit:.1f}"
        self.sv_status.set(msg)
        self.root.configure(bg=DANGER)
        self.root.after(350, lambda: self.root.configure(bg=BG))

    # ══════════════════════════════════════════════════════════════════════════
    # CSV
    # ══════════════════════════════════════════════════════════════════════════
    CSV_HDR = [
        "Zaman",
        "FreqCikis_Hz", "HizTahmini_Hz",
        "DCBus_V",      "CikisAkimi_A",    "CikisVolt_Vrms",
        "SogutucuSicak_C", "BasmaSicak_C", "FreqKomut_Hz",
        "Guc_W", "Guc_kW", "ModulasyonIndeksi_pct",
        "CalismaStatus_raw", "KorumaStatus_raw", "TripTipi_raw",
    ]

    def _toggle_log(self):
        if not self.is_logging:
            ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
            self._start_log(f"sdp_log_{ts}.csv")
        else:
            self._stop_log()

    def _start_log(self, path):
        try:
            self.log_file   = open(path, "w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(self.CSV_HDR)
            self.log_path   = os.path.abspath(path)
            self.log_rows   = 0
            self.is_logging = True
            self.btn_log.config(text="⏹  KAYDI DURDUR", bg=DANGER, fg=TEXT)
            self.lbl_log.config(text=f"→ {self.log_path}", fg=GREEN)
            self.sv_status.set(f"CSV kaydı başladı: {self.log_path}")
        except Exception as e:
            messagebox.showerror("CSV Hatası", str(e))

    def _stop_log(self):
        self.is_logging = False
        if self.log_file:
            self.log_file.close(); self.log_file = None; self.csv_writer = None
        self.btn_log.config(text="⏺  KAYDI BAŞLAT", bg=GREEN, fg=BLACK)
        self.lbl_log.config(
            text=f"Kaydedildi: {self.log_rows} satır\n{self.log_path or '—'}", fg=DIM)
        if self.log_rows > 0:
            messagebox.showinfo("Kayıt Tamam",
                f"{self.log_rows} satır kaydedildi.\n\n{self.log_path}")

    def _save_as(self):
        if self.is_logging:
            messagebox.showwarning("Uyarı", "Aktif kaydı önce durdurun.")
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = filedialog.asksaveasfilename(
            title="Farklı Kaydet", defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("Tüm", "*.*")],
            initialfile=f"sdp_log_{ts}.csv")
        if path: self._start_log(path)

    def _open_folder(self):
        target = None
        if self.log_path and os.path.exists(self.log_path):
            target = os.path.dirname(self.log_path)
        else:
            path = filedialog.askopenfilename(
                title="CSV Dosyası Seç",
                filetypes=[("CSV", "*.csv"), ("Tüm", "*.*")])
            if path: target = os.path.dirname(path)
        if target:
            try:
                if sys.platform == "win32":   os.startfile(target)
                elif sys.platform == "darwin": subprocess.Popen(["open", target])
                else:                          subprocess.Popen(["xdg-open", target])
            except Exception as e:
                messagebox.showerror("Hata", str(e))

    # ══════════════════════════════════════════════════════════════════════════
    # Tick'ler
    # ══════════════════════════════════════════════════════════════════════════
    def _chart_tick(self):
        self.chart.refresh()
        self.root.after(500, self._chart_tick)

    def _clock_tick(self):
        self.lbl_clock.config(text=datetime.now().strftime("%d.%m.%Y  %H:%M:%S"))
        self.root.after(1000, self._clock_tick)

    # ══════════════════════════════════════════════════════════════════════════
    # Yardımcılar
    # ══════════════════════════════════════════════════════════════════════════
    def _card(self, parent, title, color=ACCENT):
        outer = tk.Frame(parent, bg=PANEL,
                         highlightbackground=BORDER, highlightthickness=1)
        outer.pack(fill="x", pady=3, padx=2)
        tk.Label(outer, text=f" {title}", font=("Consolas", 8, "bold"),
                 bg=PANEL, fg=color, anchor="w").pack(fill="x", padx=6, pady=(4, 1))
        inner = tk.Frame(outer, bg=PANEL)
        inner.pack(fill="x", padx=8, pady=(0, 6))
        return inner

    def _lbl(self, parent, text):
        return tk.Label(parent, text=text, font=FLB, bg=PANEL, fg=DIM)

    def _ekw(self):
        return dict(bg=BLACK, fg=TEXT, insertbackground=ACCENT,
                    font=FM, relief="flat",
                    highlightbackground=BORDER, highlightthickness=1)

    def _btn(self, parent, text, color, cmd, padx=8, pady=3, fg=BLACK):
        return tk.Button(parent, text=text, font=FBT,
                         bg=color, fg=fg,
                         activebackground=color,
                         relief="flat", padx=padx, pady=pady,
                         cursor="hand2", command=cmd)


# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    style = ttk.Style()
    style.theme_use("clam")
    style.configure("TCombobox",
                    fieldbackground=BLACK, background=BLACK,
                    foreground=TEXT, selectbackground=BORDER,
                    arrowcolor=ACCENT)
    style.configure("TNotebook",        background=BG, borderwidth=0)
    style.configure("TNotebook.Tab",    background=PANEL, foreground=DIM,
                    padding=[12, 4], font=("Consolas", 9))
    style.map("TNotebook.Tab",
              background=[("selected", PANEL2)],
              foreground=[("selected", ACCENT)])
    app = SDPPanel(root)
    root.mainloop()
