"""
Визуализатор вибромоторов для Raspberry Pi Pico.

Подключается к плате по USB (COM-порт находится сам), включает на плате
машинный вывод командой "plot 1" и рисует:
  - 10 моторов по окружности (мотор 1 слева, дальше против часовой стрелки),
    яркость оранжевого = сила мотора;
  - дугу колокола Гаусса (зона ведущего мотора + захват в обе стороны);
  - входящее значение, основную позицию, ширину захвата, силу ведущего, скорость.

Запуск:   py python/visualizer.py
Перед запуском закройте монитор PlatformIO — порт может занимать только одна программа.
"""

import math
import queue
import random
import threading
import time
import tkinter as tk

import serial
import serial.tools.list_ports

# ---- Параметры прошивки (должны совпадать с src/main.cpp) ----
N_MOTORS = 10
VALUE_MAX = 1000
HALF_WIDTH_MIN = 20       # HALF_WIDTH_SLOW
HALF_WIDTH_MAX = 100      # HALF_WIDTH_FAST
CUSTOM_SPEED_MIN = 5
CUSTOM_SPEED_MAX = 800    # не быстрее 4 моторов за 0.5 с
UNITS_PER_MOTOR = 100

BAUD = 115200
PICO_VID = 0x2E8A

# ---- Оформление ----
BG = "#2b2a28"            # тёмно-серый «пыльный» фон
BG_GRAIN = ("#2f2d2a", "#272624", "#32302c", "#292724")
PANEL = "#33312d"
RING = "#3e3a35"
MOTOR_OFF = "#3d3935"
ORANGE = "#ff8a1f"
ARC_TAIL = "#6e4520"
ARC_ZONE = "#c0661c"
TEXT = "#ddd6ca"
MUTED = "#8c857a"
BAR_BG = "#3d3935"
BAR_FG = "#c9b9a2"
OK_COLOR = "#8fb573"
ERR_COLOR = "#d9765f"
FONT = "Segoe UI"


def blend(c1, c2, t):
    """Смешать два цвета '#rrggbb', t = 0..1."""
    t = max(0.0, min(1.0, t))
    a = [int(c1[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(c2[i:i + 2], 16) for i in (1, 3, 5)]
    return "#%02x%02x%02x" % tuple(round(x + (y - x) * t) for x, y in zip(a, b))


def value_to_angle(v):
    """Значение 0..1000 -> угол на окружности (градусы, против часовой от 3 часов).
    Мотор 1 (центр зоны = 50) — слева, 180°; каждый следующий на 36° дальше."""
    return 180.0 + 360.0 / N_MOTORS * (v / UNITS_PER_MOTOR - 0.5)


def zone(i):
    """Зона мотора i (0..9) в у.е., как в прошивке."""
    return (30 if i == 0 else i * 100), (i + 1) * 100


def add_grain(canvas, w, h, count):
    rnd = random.Random(7)
    for _ in range(count):
        x, y = rnd.randrange(w), rnd.randrange(h)
        s = rnd.choice((1, 1, 1, 2))
        canvas.create_rectangle(x, y, x + s, y + s, fill=rnd.choice(BG_GRAIN), width=0)


class Slider(tk.Canvas):
    """Ползунок в стиле окна: серая полоса, оранжевая заполненная часть, круглая ручка."""
    W, H, PAD = 290, 26, 10

    def __init__(self, parent, var, lo, hi, step, bg):
        super().__init__(parent, width=self.W + 80, height=self.H, bg=bg,
                         highlightthickness=0, cursor="hand2")
        self.var, self.lo, self.hi, self.step = var, lo, hi, step
        y = self.H / 2
        self.create_line(self.PAD, y, self.W, y, fill=BAR_BG, width=6, capstyle="round")
        self.fill = self.create_line(self.PAD, y, self.PAD, y, fill=ORANGE, width=6,
                                     capstyle="round")
        self.knob = self.create_oval(0, 0, 0, 0, fill="#f3e6d4", outline=ORANGE, width=2)
        self.text = self.create_text(self.W + 76, y, anchor="e", fill=TEXT,
                                     font=(FONT, 12, "bold"))
        self.bind("<Button-1>", self._drag)
        self.bind("<B1-Motion>", self._drag)
        var.trace_add("write", lambda *_: self._draw())
        self._draw()

    def _drag(self, ev):
        f = (ev.x - self.PAD) / (self.W - self.PAD)
        v = self.lo + max(0.0, min(1.0, f)) * (self.hi - self.lo)
        self.var.set(int(round(v / self.step) * self.step))

    def _draw(self):
        f = (self.var.get() - self.lo) / (self.hi - self.lo)
        x = self.PAD + f * (self.W - self.PAD)
        y = self.H / 2
        self.coords(self.fill, self.PAD, y, x, y)
        self.coords(self.knob, x - 8, y - 8, x + 8, y + 8)
        self.itemconfig(self.text, text=str(self.var.get()))


# =====================================================================
#  Связь с платой (в отдельном потоке)
# =====================================================================
class PicoLink:
    def __init__(self, events):
        self.events = events
        self.ser = None
        self.lock = threading.Lock()
        self.running = True
        threading.Thread(target=self._run, daemon=True).start()

    @staticmethod
    def find_port():
        for p in serial.tools.list_ports.comports():
            if p.vid == PICO_VID:
                return p.device
        return None

    def send(self, cmd):
        with self.lock:
            if self.ser:
                try:
                    self.ser.write((cmd + "\n").encode())
                except serial.SerialException:
                    pass

    def close(self):
        self.running = False
        self.send("plot 0")
        with self.lock:
            if self.ser:
                self.ser.close()
                self.ser = None

    def _run(self):
        while self.running:
            if self.ser is None:
                port = self.find_port()
                if not port:
                    self.events.put(("status", False, "Плата не найдена — подключите USB"))
                    time.sleep(1)
                    continue
                try:
                    ser = serial.Serial(port, BAUD, timeout=0.2)
                except serial.SerialException:
                    self.events.put(("status", False,
                                     f"{port} занят — закройте монитор PlatformIO"))
                    time.sleep(1.5)
                    continue
                with self.lock:
                    self.ser = ser
                self.events.put(("status", True, f"Подключено: {port}"))
                self.send("plot 1")
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError, AttributeError):
                with self.lock:
                    if self.ser:
                        try:
                            self.ser.close()
                        except Exception:
                            pass
                    self.ser = None
                self.events.put(("status", False, "Связь потеряна, переподключаюсь..."))
                time.sleep(1)
                continue
            if raw:
                text = raw.decode("utf-8", "replace").strip()
                if text:
                    self.events.put(("line", text))


class _NoLink:
    """Заглушка вместо платы для режима --preview."""
    def send(self, cmd):
        print("->", cmd)

    def close(self):
        pass


# =====================================================================
#  Окно
# =====================================================================
class App:
    CIRCLE_SIZE = 600
    R = 200               # радиус окружности моторов
    ARC_R = 132           # радиус внутреннего кольца с дугой колокола
    MOTOR_R = 34          # радиус круга мотора

    def __init__(self, root, connect=True):
        self.root = root
        root.title("Вибромоторы — визуализация")
        root.configure(bg=BG)
        root.resizable(False, False)

        self.events = queue.Queue()
        self.link = PicoLink(self.events) if connect else _NoLink()

        self._build_circle()
        self._build_side()
        self._apply_state(None)

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        root.after(30, self._poll)

    # ---------- окружность с моторами ----------
    def _build_circle(self):
        S = self.CIRCLE_SIZE
        c = tk.Canvas(self.root, width=S, height=S, bg=BG, highlightthickness=0)
        c.grid(row=0, column=0, padx=(16, 0), pady=16)
        add_grain(c, S, S, 3500)
        self.cv = c
        cx = cy = S / 2
        R = self.R
        self.cx, self.cy = cx, cy
        box = (cx - R, cy - R, cx + R, cy + R)
        A = self.ARC_R
        arc_box = (cx - A, cy - A, cx + A, cy + A)

        c.create_oval(*box, outline=RING, width=2)
        c.create_oval(*arc_box, outline=RING, width=1)
        # дуги колокола на внутреннем кольце:
        # широкая бледная — захват, яркая — зона ведущего мотора
        self.arc_tail = c.create_arc(*arc_box, style="arc", width=14, outline=ARC_TAIL,
                                     start=0, extent=0)
        self.arc_zone = c.create_arc(*arc_box, style="arc", width=14, outline=ARC_ZONE,
                                     start=0, extent=0)

        self.glows, self.motors, self.labels, self.pcts = [], [], [], []
        for i in range(N_MOTORS):
            a = math.radians(180 + 360 / N_MOTORS * i)
            x, y = cx + R * math.cos(a), cy - R * math.sin(a)
            layers = []
            for extra in (22, 14, 7):
                rr = self.MOTOR_R + extra
                layers.append(c.create_oval(x - rr, y - rr, x + rr, y + rr, fill=BG, width=0))
            self.glows.append(layers)
            r = self.MOTOR_R
            self.motors.append(c.create_oval(x - r, y - r, x + r, y + r,
                                             fill=MOTOR_OFF, outline="#55504a", width=2))
            self.labels.append(c.create_text(x, y, text=str(i + 1), fill=TEXT,
                                             font=(FONT, 15, "bold")))
            px, py = cx + (R + 72) * math.cos(a), cy - (R + 72) * math.sin(a)
            self.pcts.append(c.create_text(px, py, text="0%", fill=MUTED, font=(FONT, 11)))

        # метки на окружности: входящее значение (кольцо снаружи) и позиция (точка)
        self.mark_input = c.create_oval(0, 0, 0, 0, outline=TEXT, width=2)
        self.mark_pos = c.create_oval(0, 0, 0, 0, fill="#fff3e0", width=0)

        # центр
        self.center_big = c.create_text(cx, cy - 14, text="—", fill=TEXT,
                                        font=(FONT, 36, "bold"))
        self.center_small = c.create_text(cx, cy + 24, text="моторы выключены",
                                          fill=MUTED, font=(FONT, 12))
        c.create_text(cx, S - 14, text="○ входящее значение    ● основная позиция    "
                                       "дуга — колокол Гаусса",
                      fill=MUTED, font=(FONT, 9))

    # ---------- правая панель ----------
    def _build_side(self):
        side = tk.Frame(self.root, bg=BG)
        side.grid(row=0, column=1, sticky="n", padx=16, pady=16)

        W, H = 400, 320
        s = tk.Canvas(side, width=W, height=H, bg=BG, highlightthickness=0)
        s.pack()
        add_grain(s, W, H, 1800)
        self.sv = s
        self.stats = {}
        rows = [
            ("value", "Входящее значение"),
            ("pos", "Основная позиция"),
            ("width", "Ширина захвата"),
            ("amp", "Сила ведущего"),
            ("speed", "Скорость"),
        ]
        for k, (key, title) in enumerate(rows):
            y = 4 + k * 63
            s.create_text(0, y, text=title, anchor="nw", fill=MUTED, font=(FONT, 10))
            big = s.create_text(0, y + 16, text="—", anchor="nw", fill=TEXT,
                                font=(FONT, 20, "bold"))
            sub = s.create_text(W, y + 24, text="", anchor="ne", fill=MUTED, font=(FONT, 10))
            s.create_rectangle(0, y + 54, W, y + 60, fill=BAR_BG, width=0)
            bar = s.create_rectangle(0, y + 54, 0, y + 60,
                                     fill=ORANGE if key == "amp" else BAR_FG, width=0)
            self.stats[key] = (big, sub, bar, y)
        self.bar_w = W

        # статус и сообщения платы
        self.status = tk.Label(side, text="", bg=BG, fg=MUTED, font=(FONT, 10),
                               anchor="w")
        self.status.pack(fill="x", pady=(6, 0))
        self.message = tk.Label(side, text="", bg=BG, fg=TEXT, font=(FONT, 10),
                                anchor="w", justify="left", wraplength=400)
        self.message.pack(fill="x", pady=(2, 10))

        # кнопки
        self._button(side, "Полный тест всех режимов",
                     lambda: self.link.send("fulltest")).pack(fill="x")

        box = tk.Frame(side, bg=PANEL, padx=12, pady=10)
        box.pack(fill="x", pady=10)
        tk.Label(box, text="Свой тест", bg=PANEL, fg=TEXT,
                 font=(FONT, 11, "bold")).pack(anchor="w")
        tk.Label(box, text="Плавно к цели → стоим 5 с → мгновенно в 0",
                 bg=PANEL, fg=MUTED, font=(FONT, 9)).pack(anchor="w", pady=(0, 6))

        self.target = tk.IntVar(value=700)
        self.speed = tk.IntVar(value=150)
        self._scale(box, "Цель, у.е.", self.target, 0, VALUE_MAX, 10)
        self._scale(box, "Плавность (скорость), у.е./с", self.speed,
                    CUSTOM_SPEED_MIN, CUSTOM_SPEED_MAX, 5)
        self.speed_hint = tk.Label(box, text="", bg=PANEL, fg=MUTED, font=(FONT, 9),
                                   justify="left", anchor="w")
        self.speed_hint.pack(fill="x")
        self.speed.trace_add("write", lambda *_: self._update_speed_hint())
        self._update_speed_hint()
        self._button(box, "Запустить свой тест", self._run_custom).pack(fill="x", pady=(8, 0))

        self._button(side, "Стоп", lambda: self.link.send("stop"),
                     bg="#5a3a30", active="#72483a").pack(fill="x")

    def _button(self, parent, text, cmd, bg="#46413b", active="#5a534b"):
        return tk.Button(parent, text=text, command=cmd, bg=bg, fg=TEXT,
                         activebackground=active, activeforeground=TEXT,
                         relief="flat", bd=0, font=(FONT, 11), pady=7, cursor="hand2")

    def _scale(self, parent, title, var, lo, hi, step):
        tk.Label(parent, text=title, bg=PANEL, fg=MUTED, font=(FONT, 9)).pack(anchor="w")
        Slider(parent, var, lo, hi, step, PANEL).pack(anchor="w", pady=(0, 6))

    def _update_speed_hint(self):
        v = self.speed.get()
        motors_per_s = v / UNITS_PER_MOTOR
        self.speed_hint.config(
            text=f"≈ {motors_per_s:.1f} мотора/с  ·  {motors_per_s / 2:.1f} мотора за 0.5 с\n"
                 f"максимум {CUSTOM_SPEED_MAX} у.е./с = "
                 f"{CUSTOM_SPEED_MAX / UNITS_PER_MOTOR / 2:.0f} мотора за 0.5 с")

    def _run_custom(self):
        self.link.send(f"go {self.target.get()} {self.speed.get()}")

    # ---------- обновление ----------
    def _poll(self):
        latest = None
        try:
            while True:
                ev = self.events.get_nowait()
                if ev[0] == "status":
                    _, ok, text = ev
                    self.status.config(text=("● " if ok else "○ ") + text,
                                       fg=OK_COLOR if ok else ERR_COLOR)
                    if not ok:
                        latest = "off"
                elif ev[0] == "line":
                    line = ev[1]
                    if line.startswith("P,"):
                        parts = line[2:].split(",")
                        if len(parts) == 6 + N_MOTORS:
                            try:
                                latest = [int(p) for p in parts]
                            except ValueError:
                                pass
                    elif line != "Pico started":
                        self.message.config(text=line)
        except queue.Empty:
            pass
        if latest == "off":
            self._apply_state(None)
        elif latest is not None:
            self._apply_state(latest)
        self.root.after(30, self._poll)

    def _apply_state(self, st):
        c, s = self.cv, self.sv
        active = bool(st and st[0])
        duties = st[6:] if st else [0] * N_MOTORS

        for i, d in enumerate(duties):
            f = d / 100.0
            c.itemconfig(self.motors[i], fill=blend(MOTOR_OFF, ORANGE, f ** 0.8),
                         outline=blend("#55504a", "#ffb366", f))
            for layer, k in zip(self.glows[i], (0.18, 0.32, 0.5)):
                c.itemconfig(layer, fill=blend(BG, ORANGE, f * k))
            c.itemconfig(self.labels[i], fill="#2b1a0c" if f > 0.55 else TEXT)
            c.itemconfig(self.pcts[i], text=f"{d}%", fill=blend(MUTED, ORANGE, f))

        if not active:
            for item in (self.arc_tail, self.arc_zone, self.mark_input, self.mark_pos):
                c.itemconfig(item, state="hidden")
            c.itemconfig(self.center_big, text="—")
            c.itemconfig(self.center_small, text="моторы выключены")
            for key in self.stats:
                self._stat(key, "—", "", 0)
            return

        _, value, pos, speed, hw, amp = st[:6]
        lead = max(range(N_MOTORS), key=lambda i: duties[i]) if max(duties) > 0 else None

        if lead is None:
            for item in (self.arc_tail, self.arc_zone):
                c.itemconfig(item, state="hidden")
        else:
            z0, z1 = zone(lead)
            t0, t1 = max(0, z0 - hw), min(VALUE_MAX, z1 + hw)
            a0, a1 = value_to_angle(t0), value_to_angle(t1)
            c.itemconfig(self.arc_tail, state="normal", start=a0 % 360, extent=a1 - a0)
            b0, b1 = value_to_angle(z0), value_to_angle(z1)
            c.itemconfig(self.arc_zone, state="normal", start=b0 % 360, extent=b1 - b0)

        self._marker(self.mark_input, value, self.ARC_R - 22, 7)
        self._marker(self.mark_pos, pos, self.ARC_R, 6)

        if lead is None:
            c.itemconfig(self.center_big, text="—")
            c.itemconfig(self.center_small, text="ниже порога 30 — выключено")
        else:
            c.itemconfig(self.center_big, text=str(lead + 1))
            c.itemconfig(self.center_small, text="ведущий мотор")

        width_pct = (hw - HALF_WIDTH_MIN) / (HALF_WIDTH_MAX - HALF_WIDTH_MIN) * 100
        width_pct = max(0, min(100, width_pct))
        mps = speed / UNITS_PER_MOTOR
        self._stat("value", f"{value} у.е.", "из 1000", value / VALUE_MAX)
        self._stat("pos", f"{pos} у.е.",
                   f"мотор {lead + 1}" if lead is not None else "выключено", pos / VALUE_MAX)
        self._stat("width", f"{width_pct:.0f} %", f"±{hw} у.е.", width_pct / 100)
        self._stat("amp", f"{amp} %",
                   f"мотор {lead + 1}" if lead is not None else "", amp / 100)
        self._stat("speed", f"{speed} у.е./с", f"≈ {mps:.1f} мотора/с",
                   speed / CUSTOM_SPEED_MAX)

    def _marker(self, item, v, radius, size):
        a = math.radians(value_to_angle(v))
        x, y = self.cx + radius * math.cos(a), self.cy - radius * math.sin(a)
        self.cv.coords(item, x - size, y - size, x + size, y + size)
        self.cv.itemconfig(item, state="normal")

    def _stat(self, key, big, sub, frac):
        b, sb, bar, y = self.stats[key]
        self.sv.itemconfig(b, text=big)
        self.sv.itemconfig(sb, text=sub)
        self.sv.coords(bar, 0, y + 54, self.bar_w * max(0.0, min(1.0, frac)), y + 60)

    def _on_close(self):
        self.link.close()
        self.root.destroy()


if __name__ == "__main__":
    import sys
    preview = "--preview" in sys.argv
    root = tk.Tk()
    app = App(root, connect=not preview)
    if preview:
        # Картинка без платы: пример состояния «быстро едем около мотора 6»
        sample = [1, 640, 612, 190, 94, 68, 0, 0, 0, 1, 20, 68, 51, 12, 0, 0]
        app.message.config(text="[Полный тест 7/19] Разгон 500 -> 1000 (20 -> 350): "
                                "колокол расширяется  (предпросмотр)")
        app.status.config(text="○ Предпросмотр без платы", fg=MUTED)
        root.after(300, lambda: app._apply_state(sample))
    root.mainloop()
