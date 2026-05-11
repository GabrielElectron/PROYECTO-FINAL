import sys
import struct
import serial
import numpy as np
import time
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

# =========================
# CONFIG
# =========================
PORT = "COM9"
BAUD = 2000000

SYNC = b"\xA5\x5A\xC3\x3C"
HDR_LEN = 10                  # 4 sync + 4 seq + 2 frames
CH_PHY = 8           # canales físicos reales
CH_RMS = 6           # V1..V3 + I1..I3
CH = CH_PHY + CH_RMS # canales que maneja la UI
BYTES_PER_FRAME = 16          # 8ch * int16 = 16 bytes/frame
MAX_FRAMES = 4096

V_RANGE = 10.0
ADC_SCALE = V_RANGE / 32768.0

FREQ_MAX_HZ = 50000.0
FREQ_MIN_VPP = 0.05
FREQ_HYST_FRAC = 0.02
FREQ_HOLD_SEC = 1.0
FREQ_NEED_CYCLES_LOW = 2.0
FREQ_NEED_CYCLES_HIGH = 3.0
FREQ_SPLIT_HZ = 200.0

RMS_WIN_MS = 200.0        # ventana de cálculo RMS
RMS_HIST_SEC = 10.0       # cuánto histórico mostrar en la gráfica RMS
RMS_DECIM_HZ = 20.0       # cuántos puntos por segundo agrego al trend (20Hz queda bien)

TIME_DIV_LIST = [
    1e-6, 2e-6, 5e-6,
    10e-6, 20e-6, 50e-6,
    100e-6, 200e-6, 500e-6,
    1e-3, 2e-3, 5e-3,
    10e-3, 20e-3, 50e-3,
    100e-3, 200e-3, 500e-3,
    1.0
]

CH_COLORS = [
    (255, 255, 0),
    (0, 255, 255),
    (255, 0, 255),
    (0, 255, 0),
    (255, 128, 0),
    (0, 128, 255),
    (255, 0, 0),
    (200, 200, 200)
]

RMS_COLORS = [
    (255, 255, 255),
    (220, 220, 220),
    (200, 200, 200),
    (255, 255, 255),
    (220, 220, 220),
    (200, 200, 200),
]

# ... (pegá acá tus FREQ_*, TIME_DIV_LIST, CH_COLORS)
# ==========================================================
# PEGÁ ACÁ TU Reader y tu Scope tal cual los tenés
# (No los repito para no hacer esto eterno)
# ==========================================================
# from tu_archivo import Scope  # si lo tenés separado
# class Reader(QtCore.QThread): ...
# class Scope(QtWidgets.QMainWindow): ...


# =========================
# READER THREAD
# =========================
class Reader(QtCore.QThread):
    packet = QtCore.pyqtSignal(object, int)  # data(np.ndarray), seq(int)

    def __init__(self, port, baud):
        super().__init__()
        self.ser = serial.Serial(port, baud, timeout=0.2)
        self.buf = bytearray()
        self.running = True

        self.sync_hits = 0
        self.bad_frames = 0

    def stop(self):
        self.running = False

    def run(self):
        while self.running:
            chunk = self.ser.read(65536)
            if chunk:
                self.buf += chunk

            while True:
                idx = self.buf.find(SYNC)
                if idx < 0:
                    if len(self.buf) > 262144:
                        self.buf = self.buf[-64:]
                    break

                if idx > 0:
                    del self.buf[:idx]

                if len(self.buf) < HDR_LEN:
                    break

                try:
                    seq = struct.unpack_from("<I", self.buf, 4)[0]
                    frames = struct.unpack_from("<H", self.buf, 8)[0]
                except struct.error:
                    break

                if frames == 0 or frames > MAX_FRAMES:
                    self.bad_frames += 1
                    del self.buf[:1]
                    continue

                pkt_len = HDR_LEN + frames * BYTES_PER_FRAME
                if len(self.buf) < pkt_len:
                    break

                self.sync_hits += 1
                payload = bytes(self.buf[HDR_LEN:pkt_len])
                del self.buf[:pkt_len]

                data = np.frombuffer(payload, dtype=">i2")
                if data.size != frames * CH_PHY:
                    continue
                data = data.reshape(frames, CH_PHY)

                self.packet.emit(data, seq)

        try:
            self.ser.close()
        except:
            pass




# =========================
# SCOPE UI
# =========================
class Scope(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Scope - Rigol style (canales seleccionables + freeze)")
        self.resize(1450, 780)

        pg.setConfigOptions(useOpenGL=True)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        h = QtWidgets.QHBoxLayout(central)
        h.setContentsMargins(6, 6, 6, 6)
        h.setSpacing(8)

        # -------- plot --------
        self.plot = pg.PlotWidget()
        self.plot.setBackground('k')  # <<< FONDO NEGRO
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        h.addWidget(self.plot, 1)

        # <<< CLAVE: no autorango nunca
        self.plot.enableAutoRange(x=False, y=False)

        # Y fijo
        self.plot.setYRange(-V_RANGE, V_RANGE)

        # Fijar escala Y según rango bipolar
        self.plot.enableAutoRange(axis='y', enable=False)
        self.plot.setYRange(-V_RANGE, V_RANGE)

        self.curves = []

        # curvas físicas (8)
        for i in range(CH_PHY):
            pen = pg.mkPen(color=CH_COLORS[i], width=2)
            self.curves.append(self.plot.plot(pen=pen))

        # curvas RMS (6) - más finitas y punteadas
        for k in range(CH_RMS):
            pen = pg.mkPen(color=RMS_COLORS[k], width=1, style=QtCore.Qt.DashLine)
            self.curves.append(self.plot.plot(pen=pen))

        # ==========================================================
        # >>> NUEVO: TEXTOS DE CANALES DENTRO DEL PLOT (ARRIBA)
        # ==========================================================
        from PyQt5 import QtGui

        self.ch_readouts = []
        font = QtGui.QFont("Consolas", 10)
        font.setBold(True)

        for i in range(CH_PHY):
            color = pg.mkColor(CH_COLORS[i])
            ti = pg.TextItem(color=color, anchor=(0, 0))
            ti.setFont(font)
            ti.setZValue(50)
            self.plot.addItem(ti)
            self.ch_readouts.append(ti)

        # =====================
        # Cursores (manuales)
        # =====================
        self.cursors_enabled = True

        self.vline1 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen((180, 180, 180), width=2))
        self.vline2 = pg.InfiniteLine(angle=90, movable=True, pen=pg.mkPen((180, 180, 180), width=2))
        self.vline1.setZValue(10)
        self.vline2.setZValue(10)

        self.hline1 = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen((180, 180, 180), width=2))
        self.hline2 = pg.InfiniteLine(angle=0, movable=True, pen=pg.mkPen((180, 180, 180), width=2))
        self.hline1.setZValue(10)
        self.hline2.setZValue(10)

        self.plot.addItem(self.vline1)
        self.plot.addItem(self.vline2)
        self.plot.addItem(self.hline1)
        self.plot.addItem(self.hline2)

        self.vline1.setPos(0.002)
        self.vline2.setPos(0.006)
        self.hline1.setPos(0.0)
        self.hline2.setPos(1.0)

        self.vline1.sigPositionChanged.connect(self.update_cursors)
        self.vline2.sigPositionChanged.connect(self.update_cursors)
        self.hline1.sigPositionChanged.connect(self.update_cursors)
        self.hline2.sigPositionChanged.connect(self.update_cursors)

        self.lbl_cursors = QtWidgets.QLabel("Cursores: --")
        self.lbl_cursors.setWordWrap(True)
        self.lbl_cursors.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
        self.lbl_cursors.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignTop)
        self.lbl_cursors.setFixedHeight(130)

        # -------- panel derecho --------
        self.panel = QtWidgets.QFrame()
        self.panel.setFrameShape(QtWidgets.QFrame.StyledPanel)

        self.scroll = QtWidgets.QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.scroll.setWidget(self.panel)

        self.scroll.setMinimumWidth(330)
        self.scroll.setMaximumWidth(420)

        h.addWidget(self.scroll, 0)

        p = QtWidgets.QVBoxLayout(self.panel)
        p.setContentsMargins(10, 10, 10, 10)
        p.setSpacing(10)

        title = QtWidgets.QLabel("CONTROLES")
        title.setStyleSheet("font-weight: 700; font-size: 16px;")
        p.addWidget(title)

        # =====================
        # Adquisición
        # =====================
        g_acq = QtWidgets.QGroupBox("Adquisición")
        gl = QtWidgets.QGridLayout(g_acq)

        self.btn_run = QtWidgets.QPushButton("RUN")
        self.btn_run.setCheckable(True)
        self.btn_run.setChecked(True)
        self.btn_run.clicked.connect(self.on_toggle_run)
        gl.addWidget(self.btn_run, 0, 0, 1, 2)

        gl.addWidget(QtWidgets.QLabel("UI FPS:"), 1, 0)
        self.spin_fps = QtWidgets.QSpinBox()
        self.spin_fps.setRange(5, 120)
        self.spin_fps.setValue(30)
        self.spin_fps.valueChanged.connect(self.on_fps_changed)
        gl.addWidget(self.spin_fps, 1, 1)

        gl.addWidget(QtWidgets.QLabel("Puntos display:"), 2, 0)
        self.spin_pts = QtWidgets.QSpinBox()
        self.spin_pts.setRange(200, 20000)
        self.spin_pts.setSingleStep(200)
        self.spin_pts.setValue(2000)
        gl.addWidget(self.spin_pts, 2, 1)

        p.addWidget(g_acq)

        # =====================
        # Canales visibles
        # =====================
        g_ch = QtWidgets.QGroupBox("Canales (visibles)")
        vch = QtWidgets.QVBoxLayout(g_ch)

        self.ch_checks = []
        self.ch_visible = np.ones(CH, dtype=bool)

        channel_names = [
            "V1(t)", "V2(t)", "V3(t)",
            "I1(t)", "I2(t)", "I3(t)",
            "Aux1", "Aux2",
            "V1rms", "V2rms", "V3rms",
            "I1rms", "I2rms", "I3rms",
        ]

        for i in range(CH):
            cb = QtWidgets.QCheckBox(channel_names[i])
            cb.setChecked(True)

            if i < CH_PHY:
                r, g, b = CH_COLORS[i]
                cb.setStyleSheet(f"color: rgb({r},{g},{b}); font-weight: 600;")
            else:
                r, g, b = RMS_COLORS[i - CH_PHY]
                cb.setStyleSheet(f"color: rgb({r},{g},{b}); font-weight: 700;")

            cb.stateChanged.connect(self.on_channels_changed)
            vch.addWidget(cb)
            self.ch_checks.append(cb)

        p.addWidget(g_ch)

        # =====================
        # Timebase
        # =====================
        g_time = QtWidgets.QGroupBox("Timebase")
        glt = QtWidgets.QGridLayout(g_time)

        glt.addWidget(QtWidgets.QLabel("Time/div:"), 0, 0)
        self.cmb_time = QtWidgets.QComboBox()
        for td in TIME_DIV_LIST:
            self.cmb_time.addItem(self.format_time_div(td), userData=td)
        self.cmb_time.setCurrentText(self.format_time_div(1e-3))
        self.cmb_time.currentIndexChanged.connect(self.on_time_div_changed)
        glt.addWidget(self.cmb_time, 0, 1)

        glt.addWidget(QtWidgets.QLabel("Divisiones:"), 1, 0)
        self.spin_div = QtWidgets.QSpinBox()
        self.spin_div.setRange(5, 20)
        self.spin_div.setValue(10)
        glt.addWidget(self.spin_div, 1, 1)

        p.addWidget(g_time)

        # =====================
        # Trigger
        # =====================
        g_tr = QtWidgets.QGroupBox("Trigger")
        glr = QtWidgets.QGridLayout(g_tr)

        glr.addWidget(QtWidgets.QLabel("Canal:"), 0, 0)
        self.cmb_tr_ch = QtWidgets.QComboBox()
        for i in range(CH_PHY):
            self.cmb_tr_ch.addItem(f"CH{i+1}", userData=i)
        self.cmb_tr_ch.currentIndexChanged.connect(self.on_trig_changed)
        glr.addWidget(self.cmb_tr_ch, 0, 1)

        glr.addWidget(QtWidgets.QLabel("Flanco:"), 1, 0)
        self.cmb_edge = QtWidgets.QComboBox()
        self.cmb_edge.addItems(["Rising", "Falling"])
        self.cmb_edge.currentIndexChanged.connect(self.on_trig_changed)
        glr.addWidget(self.cmb_edge, 1, 1)

        glr.addWidget(QtWidgets.QLabel("Nivel (raw):"), 2, 0)
        self.spin_level = QtWidgets.QSpinBox()
        self.spin_level.setRange(-32768, 32767)
        self.spin_level.setSingleStep(200)
        self.spin_level.setValue(0)
        self.spin_level.valueChanged.connect(self.on_trig_changed)
        glr.addWidget(self.spin_level, 2, 1)

        self.btn_fix_y = QtWidgets.QPushButton("Fix Y = ±Range")
        self.btn_fix_y.clicked.connect(self.fix_y_0_5)
        glr.addWidget(self.btn_fix_y, 3, 0, 1, 2)

        p.addWidget(g_tr)

        # =====================
        # Info
        # =====================
        g_info = QtWidgets.QGroupBox("Info")
        gli = QtWidgets.QVBoxLayout(g_info)
        self.lbl_info = QtWidgets.QLabel("Esperando datos...")
        self.lbl_info.setWordWrap(True)
        gli.addWidget(self.lbl_info)
        p.addWidget(g_info)

        # =====================
        # Cursores UI
        # =====================
        g_cur = QtWidgets.QGroupBox("Cursores")
        glc = QtWidgets.QGridLayout(g_cur)

        self.btn_cursors = QtWidgets.QPushButton("CURSORS")
        self.btn_cursors.setCheckable(True)
        self.btn_cursors.setChecked(False)
        self.btn_cursors.clicked.connect(self.on_toggle_cursors)
        glc.addWidget(self.btn_cursors, 0, 0, 1, 2)

        btn_reset_cur = QtWidgets.QPushButton("Reset cursores")
        btn_reset_cur.clicked.connect(self.reset_cursors)
        glc.addWidget(btn_reset_cur, 1, 0, 1, 2)

        glc.addWidget(self.lbl_cursors, 2, 0, 1, 2)

        p.addWidget(g_cur)
        g_cur.setFixedHeight(180)
        self.on_toggle_cursors()

        p.addStretch(1)

        # =====================
        # RMS Tensiones
        # =====================
        g_rms = QtWidgets.QGroupBox("Tensiones eficaces")
        glrms = QtWidgets.QGridLayout(g_rms)

        self.lbl_v1rms = QtWidgets.QLabel("-- V")
        self.lbl_v2rms = QtWidgets.QLabel("-- V")
        self.lbl_v3rms = QtWidgets.QLabel("-- V")

        for lbl in (self.lbl_v1rms, self.lbl_v2rms, self.lbl_v3rms):
            lbl.setStyleSheet("font-size: 16px; font-weight: 700; font-family: Consolas;")

        glrms.addWidget(QtWidgets.QLabel("V1rms:"), 0, 0)
        glrms.addWidget(self.lbl_v1rms, 0, 1)

        glrms.addWidget(QtWidgets.QLabel("V2rms:"), 1, 0)
        glrms.addWidget(self.lbl_v2rms, 1, 1)

        glrms.addWidget(QtWidgets.QLabel("V3rms:"), 2, 0)
        glrms.addWidget(self.lbl_v3rms, 2, 1)

        p.addWidget(g_rms)

        # =====================
        # Estado interno
        # =====================
        self.running = True

        self.buffer_len = 200000
        self.rb = np.zeros((self.buffer_len, CH_PHY), dtype=np.int16)
        self.wr = 0

        self.frozen = False
        self.frozen_rb = None
        self.frozen_wr = 0
        self.frozen_view = None
        self.frozen_x = None
        self._last_x = None
        self.frozen_fs = 0.0
        self._last_y = [None] * CH
        self._freq_last = [0.0] * CH
        self._freq_last_t = [0.0] * CH

        self.time_div = 1e-3
        self.divisions = 10

        self.rms_t = np.array([], dtype=np.float32)          # eje tiempo del trend
        self.rms_y = [np.array([], dtype=np.float32) for _ in range(CH_RMS)]  # 6 señales
        self._rms_last_push = 0.0

        # X fijo inicial (ya existen time_div y spin_div)
        total_time0 = self.time_div * int(self.spin_div.value())
        self.plot.setXRange(0.0, total_time0, padding=0.0)

        self.trig_ch = 0
        self.trig_level = 0
        self.trig_rising = True

        self.fs_real = 0.0
        self.frames_total = 0
        self.bytes_total = 0
        self.packets = 0
        self.t0 = time.time()

        self.last_seq = None
        self.lost_packets = 0

        # Timer UI
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh)
        self.set_ui_fps(30)

        self.statusBar().showMessage("Iniciando...")

    def fix_y_0_5(self):
        self.plot.enableAutoRange(axis='y', enable=False)
        self.plot.setYRange(-V_RANGE, V_RANGE)

    # -------- formatting --------
    def format_time_div(self, td: float) -> str:
        if td >= 1:
            return f"{td:.3f} s/div"
        if td >= 1e-3:
            return f"{td*1e3:.3f} ms/div"
        if td >= 1e-6:
            return f"{td*1e6:.3f} us/div"
        return f"{td:.3e} s/div"

    # -------- callbacks --------
    def on_toggle_run(self):
        self.running = self.btn_run.isChecked()
        self.btn_run.setText("RUN" if self.running else "STOP")

        if not self.running:
            self.frozen = True
            self.frozen_rb = self.rb.copy()
            self.frozen_wr = int(self.wr)
            self.frozen_fs = float(self.fs_real) if self.fs_real > 0 else 1.0
            self.frozen_view = None
            self.frozen_x = None
        else:
            self.frozen = False
            self.frozen_rb = None
            self.frozen_view = None
            self.frozen_x = None
            self.frozen_fs = 0.0

    def on_channels_changed(self):
        for i, cb in enumerate(self.ch_checks):
            self.ch_visible[i] = cb.isChecked()
            self.curves[i].setVisible(bool(self.ch_visible[i]))

        # readouts solo para físicos
        for i in range(CH_PHY):
            self.ch_readouts[i].setVisible(bool(self.ch_visible[i]))

    def set_ui_fps(self, fps: int):
        fps = max(5, int(fps))
        self.timer.start(int(1000 / fps))

    def on_fps_changed(self, v):
        self.set_ui_fps(v)

    def on_time_div_changed(self):
        td = self.cmb_time.currentData()
        if td is not None:
            self.time_div = float(td)

    def on_trig_changed(self):
        self.trig_ch = int(self.cmb_tr_ch.currentData())
        self.trig_level = int(self.spin_level.value())
        self.trig_rising = (self.cmb_edge.currentText() == "Rising")

    # -------- ingestion --------
    def on_packet(self, data, seq):
        n = data.shape[0]

        self.packets += 1
        self.frames_total += n
        self.bytes_total += n * BYTES_PER_FRAME

        if self.last_seq is not None and seq != self.last_seq + 1:
            self.lost_packets += max(0, (seq - self.last_seq - 1))
        self.last_seq = seq

        if self.frozen:
            return

        if n >= self.buffer_len:
            self.rb[:] = data[-self.buffer_len:]
            self.wr = 0
        else:
            end = self.wr + n
            if end <= self.buffer_len:
                self.rb[self.wr:end] = data
            else:
                k = self.buffer_len - self.wr
                self.rb[self.wr:] = data[:k]
                self.rb[:end - self.buffer_len] = data[k:]
            self.wr = (self.wr + n) % self.buffer_len

    # -------- helpers --------
    def find_trigger_index(self, y: np.ndarray):
        lvl = self.trig_level
        if y.size < 4:
            return None

        if self.trig_rising:
            idx = np.where((y[:-1] < lvl) & (y[1:] >= lvl))[0]
        else:
            idx = np.where((y[:-1] > lvl) & (y[1:] <= lvl))[0]

        if idx.size == 0:
            return None
        return int(idx[-1])

    def get_window(self, rb_src: np.ndarray, wr_src: int, samples_visible: int) -> np.ndarray:
        if wr_src >= samples_visible:
            return rb_src[wr_src - samples_visible:wr_src]
        part1 = rb_src[self.buffer_len - (samples_visible - wr_src):]
        part2 = rb_src[:wr_src]
        return np.vstack((part1, part2))

    def measure_freq_auto(self, v: np.ndarray, fs: float):
        """return (f_hz, ncycles_est)"""
        if v is None or v.size < 80 or fs <= 0:
            return 0.0, 0.0

        v = v.astype(np.float32)
        v = v - float(np.mean(v))  # quitar DC

        vmax = float(np.max(v))
        vmin = float(np.min(v))
        vpp = vmax - vmin
        if vpp < FREQ_MIN_VPP:
            return 0.0, 0.0

        # zona muerta alrededor de 0 para ignorar ruido
        hyst = max(FREQ_HYST_FRAC * vpp, 0.001)  # mínimo 1 mV

        a = v.copy()
        a[np.abs(a) <= hyst] = 0.0

        signa = np.sign(a)
        # rellenar ceros con el último signo válido (para no inventar cruces)
        nz = np.nonzero(signa)[0]
        if nz.size < 10:
            return 0.0, 0.0
        last = signa[nz[0]]
        for k in range(signa.size):
            if signa[k] == 0:
                signa[k] = last
            else:
                last = signa[k]

        # cruces NEG -> POS
        c = np.where((signa[:-1] < 0) & (signa[1:] > 0))[0]
        if c.size < 2:
            return 0.0, 0.0

        dp = np.diff(c).astype(np.float32)
        if dp.size < 1:
            return 0.0, 0.0

        p_s = float(np.median(dp)) / fs
        if p_s <= 0:
            return 0.0, 0.0

        f = 1.0 / p_s
        if f <= 0:
            return 0.0, 0.0
        
        if f > FREQ_MAX_HZ:
            return 0.0, 0.0

        # ciclos observados en la ventana
        win_s = v.size / fs
        ncycles = win_s * f

        return float(f), float(ncycles)
    


        # ==========================================================
        # >>> NUEVO: actualizar textos arriba del plot
        # ==========================================================
    def update_channel_readouts(self, view_i16: np.ndarray, fs_use: float):
        if view_i16 is None or view_i16.shape[0] < 2:
            for i in range(CH_PHY):
                self.ch_readouts[i].setText("")
            return

        vb = self.plot.getViewBox()
        (xmin, xmax), (ymin, ymax) = vb.viewRange()

        # layout tipo Rigol: bloques en columnas arriba
        visible_idx = [i for i in range(CH_PHY) if self.ch_visible[i]]
        nvis = len(visible_idx)
        if nvis == 0:
            return

        cols = min(4, nvis)                # 3 o 4 columnas según cuantos canales muestres
        col_w = (xmax - xmin) / cols

        x_pad = 0.015 * (xmax - xmin)
        y_top = ymax - 0.03 * (ymax - ymin)

        for k, i in enumerate(visible_idx):
            v = view_i16[:, i].astype(np.float32) * ADC_SCALE

            vpp  = float(np.max(v) - np.min(v))
            vrms = float(np.sqrt(np.mean(v * v)))
            vmax = float(np.max(v))
            vmin = float(np.min(v))

            f_meas, ncy = self.measure_freq_auto(v, fs_use)

            # requerir ciclos según banda
            need = FREQ_NEED_CYCLES_LOW if f_meas < FREQ_SPLIT_HZ else FREQ_NEED_CYCLES_HIGH
            ok = (f_meas > 0) and (ncy >= need)

            now = time.time()
            if ok:
                self._freq_last[i] = f_meas
                self._freq_last_t[i] = now
                f = f_meas
            else:
                # HOLD para evitar parpadeo
                if (now - self._freq_last_t[i]) <= FREQ_HOLD_SEC and self._freq_last[i] > 0:
                    f = self._freq_last[i]
                else:
                    f = 0.0

            T_ms = (1000.0 / f) if f > 0 else 0.0

            T_ms = (1000.0 / f) if f > 0 else 0.0

            freq_str = f"{f:7.1f} Hz" if f > 0 else "   --   "
            t_str    = f"{T_ms:7.2f} ms" if f > 0 else "   --   "
            # texto estilo captura
            txt = (
                f"CH{i+1}\n"
                f"Vpp   {vpp:7.3f} V\n"
                f"Vrms  {vrms:7.3f} V\n"
                f"Vmax  {vmax:7.3f} V\n"
                f"Vmin  {vmin:7.3f} V\n"
                f"Freq  {freq_str}\n"
                f"T     {t_str}"
            )

            self.ch_readouts[i].setText(txt)

            col = k % cols
            row = k // cols   # si hay muchos, baja a otra fila (opcional)
            x = xmin + col * col_w + x_pad
            y = y_top - row * (0.30 * (ymax - ymin))  # separación entre filas
            self.ch_readouts[i].setPos(x, y)

        # ocultar los que no estén visibles
        for i in range(CH_PHY):
            if i not in visible_idx:
                self.ch_readouts[i].setText("")

    # -------- refresh --------
    def refresh(self):
        view = None
        samples_visible = 0
        x_full = None

        f_trig = 0.0
        raw_min = None
        raw_max = None

        # ====== INFO por segundo ======
        dt = time.time() - self.t0
        if dt >= 1.0:
            self.fs_real = self.frames_total / dt if dt > 0 else 0.0
            mbps = (self.bytes_total * 8) / dt / 1e6 if dt > 0 else 0.0
            pps = self.packets / dt if dt > 0 else 0.0

            self.frames_total = 0
            self.bytes_total = 0
            self.packets = 0
            self.t0 = time.time()

            mode = "STOP (frozen)" if self.frozen else "RUN"
            self.lbl_info.setText(
                f"Modo: {mode}\n"
                f"Fs real: {self.fs_real:,.0f} S/s\n"
                f"Throughput: {mbps:.2f} Mbit/s\n"
                f"Packets/s: {pps:.1f}\n"
                f"Lost packets: {self.lost_packets}\n"
                f"SYNC hits: {getattr(self, 'reader', None).sync_hits if hasattr(self,'reader') else '-'}\n"
                f"Time/div: {self.format_time_div(self.time_div)}"
            )

        if self.fs_real <= 0 and not (self.frozen and self.frozen_rb is not None):
            return

        # ====== fuente RUN/STOP ======
        if self.frozen and self.frozen_rb is not None:
            fs_use = float(self.frozen_fs) if self.frozen_fs > 0 else 1.0
            rb_src = self.frozen_rb
            wr_src = int(self.frozen_wr)
        else:
            fs_use = float(self.fs_real)
            rb_src = self.rb
            wr_src = int(self.wr)

        divs = int(self.spin_div.value())
        total_time = self.time_div * divs
        self.plot.setXRange(0.0, total_time, padding=0.0)

        samples_visible = int(fs_use * total_time)
        samples_visible = max(50, min(samples_visible, self.buffer_len - 1))

        view = self.get_window(rb_src, wr_src, samples_visible)
        if view is None or view.shape[0] < 2:
            return

        # ====== RMS V1/V2/V3 (siempre) ======
        vv = view.astype(np.float32) * ADC_SCALE
        v1rms = float(np.sqrt(np.mean(vv[:, 0] * vv[:, 0])))
        v2rms = float(np.sqrt(np.mean(vv[:, 1] * vv[:, 1])))
        v3rms = float(np.sqrt(np.mean(vv[:, 2] * vv[:, 2])))

        self.lbl_v1rms.setText(f"{v1rms:7.3f} V")
        self.lbl_v2rms.setText(f"{v2rms:7.3f} V")
        self.lbl_v3rms.setText(f"{v3rms:7.3f} V")

        # ====== Trigger (recorte) ======
        tidx = self.find_trigger_index(view[:, self.trig_ch])
        if tidx is not None and tidx + samples_visible <= view.shape[0]:
            view = view[tidx:tidx + samples_visible]

        samples_visible = view.shape[0]
        dt_s = 1.0 / fs_use if fs_use > 0 else 1.0
        x_full = np.arange(samples_visible, dtype=np.float32) * dt_s

        # ====== Frecuencia canal trigger ======
        v_tr = view[:, self.trig_ch].astype(np.float32) * ADC_SCALE
        f_trig, _ = self.measure_freq_auto(v_tr, fs_use)

        # ====== raw min/max CH1 ======
        raw_min = int(view[:, 0].min())
        raw_max = int(view[:, 0].max())

        self.statusBar().showMessage(
            f"Fs: {fs_use:,.0f} S/s | f(trig): {f_trig:,.2f} Hz | RAW CH1 min/max: {raw_min}/{raw_max} | Lost: {self.lost_packets}"
        )


        # ====== RMS trending (V1..V3, I1..I3) ======
        now = time.time()
        if (now - self._rms_last_push) >= (1.0 / RMS_DECIM_HZ):
            self._rms_last_push = now

            nwin = int(fs_use * (RMS_WIN_MS / 1000.0))
            nwin = max(64, min(nwin, view.shape[0]))
            w = view[-nwin:, :].astype(np.float32) * ADC_SCALE  # volts/amps (según tu escala)

            v1 = float(np.sqrt(np.mean(w[:, 0] * w[:, 0])))
            v2 = float(np.sqrt(np.mean(w[:, 1] * w[:, 1])))
            v3 = float(np.sqrt(np.mean(w[:, 2] * w[:, 2])))
            i1 = float(np.sqrt(np.mean(w[:, 3] * w[:, 3])))
            i2 = float(np.sqrt(np.mean(w[:, 4] * w[:, 4])))
            i3 = float(np.sqrt(np.mean(w[:, 5] * w[:, 5])))

            vals = [v1, v2, v3, i1, i2, i3]

            self.rms_t = np.append(self.rms_t, now)
            for k in range(CH_RMS):
                self.rms_y[k] = np.append(self.rms_y[k], vals[k])

            # recortar historia a RMS_HIST_SEC
            tmin = now - RMS_HIST_SEC
            keep = self.rms_t >= tmin
            self.rms_t = self.rms_t[keep]
            for k in range(CH_RMS):
                self.rms_y[k] = self.rms_y[k][keep]

        # ====== downsample y dibujar ======
        display_pts = int(self.spin_pts.value())
        step = max(1, samples_visible // max(200, display_pts))

        self._last_x = x_full[::step]

        # ---- dibujar físicos (0..7) ----
        for i in range(CH_PHY):
            if self.ch_visible[i]:
                y = view[::step, i].astype(np.float32) * ADC_SCALE
                self.curves[i].setData(self._last_x, y, _copy=False)
            else:
                self.curves[i].clear()

            # ---- dibujar RMS (8..13) ----
        if self.rms_t.size > 2:
            # escalar RMS al eje visible del osciloscopio
            hist_dur = self.rms_t[-1] - self.rms_t[0]
            if hist_dur > 0:
                xt = (self.rms_t - self.rms_t[0]) / hist_dur * total_time
            else:
                xt = np.zeros_like(self.rms_t, dtype=np.float32)
            for k in range(CH_RMS):
                i = CH_PHY + k
                if self.ch_visible[i]:
                    self.curves[i].setData(xt, self.rms_y[k], _copy=False)
                else:
                    self.curves[i].clear()
        else:
            # si no hay historia aún, limpialas
            for k in range(CH_RMS):
                self.curves[CH_PHY + k].clear()

        self.update_cursors()
        self.update_channel_readouts(view, fs_use)

        
    def closeEvent(self, event):
        event.accept()

    def on_toggle_cursors(self):
        self.cursors_enabled = self.btn_cursors.isChecked()

        for it in (self.vline1, self.vline2, self.hline1, self.hline2):
            it.setVisible(self.cursors_enabled)

        if self.cursors_enabled:
            self.btn_cursors.setStyleSheet("background-color: rgb(60,120,60); font-weight: bold;")
        else:
            self.btn_cursors.setStyleSheet("")

        self.update_cursors()

    def reset_cursors(self):
        vb = self.plot.getViewBox()
        xr, yr = vb.viewRange()
        xmin, xmax = xr
        ymin, ymax = yr

        if xmax <= xmin:
            xmin, xmax = 0.0, 0.01
        if ymax <= ymin:
            ymin, ymax = -V_RANGE, V_RANGE

        self.vline1.setPos(xmin + 0.25 * (xmax - xmin))
        self.vline2.setPos(xmin + 0.75 * (xmax - xmin))
        self.hline1.setPos(ymin + 0.25 * (ymax - ymin))
        self.hline2.setPos(ymin + 0.75 * (ymax - ymin))
        self.update_cursors()

    def update_cursors(self):
        if not getattr(self, "cursors_enabled", False):
            self.lbl_cursors.setText("Cursores: OFF")
            return

        t1 = float(self.vline1.value())
        t2 = float(self.vline2.value())
        v1 = float(self.hline1.value())
        v2 = float(self.hline2.value())

        dt = abs(t2 - t1)
        dv = abs(v2 - v1)

        finv = (1.0 / dt) if dt > 0 else 0.0

        def fmt_s(x):
            ax = abs(x)
            if ax >= 1:
                return f"{x:.6f} s"
            if ax >= 1e-3:
                return f"{x*1e3:.3f} ms"
            if ax >= 1e-6:
                return f"{x*1e6:.3f} us"
            return f"{x:.3e} s"

        self.lbl_cursors.setText(
            "Tiempo:\n"
            f"  t1 = {fmt_s(t1)}\n"
            f"  t2 = {fmt_s(t2)}\n"
            f"  Δt = {fmt_s(dt)}\n"
            f"  1/Δt = {finv:,.3f} Hz\n"
            "Voltaje:\n"
            f"  V1 = {v1:,.4f} V\n"
            f"  V2 = {v2:,.4f} V\n"
            f"  ΔV = {dv:,.4f} V"
        )

    def measure_freq(self, y: np.ndarray, fs: float, level: float) -> float:
        """Frecuencia para trigger: cruces por nivel."""
        if y is None or y.size < 10 or fs <= 0:
             return 0.0
        y = y.astype(np.float32)
        s = y - float(level)
        c = np.where((s[:-1] < 0) & (s[1:] > 0))[0]
        if c.size < 2:
            return 0.0
        periods = np.diff(c) / fs
        if periods.size == 0:
            return 0.0
        return float(1.0 / np.mean(periods))



# =========================
# FSM States
# =========================
class AppState:
    MENU1 = 1
    MENU2 = 2
    MENU5 = 5


# =========================
# MENU 1
# =========================

class Menu1Widget(QtWidgets.QWidget):
    open_menu = QtCore.pyqtSignal(int)  # emite 0..5 según el cuadro

    def __init__(self):
        super().__init__()

        # ===== root layout (centra todo) =====
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        root.addStretch(1)  # centra vertical

        # Contenedor para centrar horizontalmente
        container = QtWidgets.QWidget()
        container.setMaximumWidth(1300)
        c = QtWidgets.QVBoxLayout(container)
        c.setContentsMargins(30, 20, 30, 20)
        c.setSpacing(18)

        # ===== título centrado =====
        title = QtWidgets.QLabel("MENÚ PRINCIPAL")
        title.setAlignment(QtCore.Qt.AlignHCenter)
        title.setStyleSheet("font-size: 34px; font-weight: 900;")
        c.addWidget(title)

        subtitle = QtWidgets.QLabel("Elegí una opción:")
        subtitle.setAlignment(QtCore.Qt.AlignHCenter)
        subtitle.setStyleSheet("font-size: 14px; opacity: 0.85;")
        c.addWidget(subtitle)

        # ===== grilla 2x3 =====
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(20, 10, 20, 10)
        grid.setHorizontalSpacing(40)
        grid.setVerticalSpacing(35)

        items = [
            ("📈", "Adquisición de Datos",  "Abrir adquisición / visualización", 0),
            ("⚙️", "Config Motor",  "Parámetros del drive / setpoints", 1),
            ("🧪", "Calibración",   "Offsets, escalas y pruebas", 2),
            ("💾", "Registro",      "Capturas, .npy, exportaciones", 3),
            ("🩺", "Diagnóstico",   "Estado link, pérdida, throughput", 4),
            ("🔧", "Ajustes",       "Preferencias generales", 5),
        ]

        self.cards = []
        for k, (ico, name, desc, idx) in enumerate(items):
            card = QtWidgets.QFrame()
            card.setMinimumSize(320, 170)
            card.setCursor(QtCore.Qt.PointingHandCursor)

            card.setStyleSheet("""
               QFrame{
                    border-radius: 18px;
                    border: 1px solid rgba(0,0,0,25);
                    background: rgba(0,0,0,6);
                }
                QFrame:hover{
                    background: rgba(0,0,0,12);
                    border: 1px solid rgba(0,0,0,45);
                }
            """)

            layout_card = QtWidgets.QVBoxLayout(card)
            layout_card.setContentsMargins(24, 20, 24, 20)
            layout_card.setSpacing(8)

            # ICONO
            lbl_icon = QtWidgets.QLabel(ico)
            lbl_icon.setAlignment(QtCore.Qt.AlignHCenter)
            lbl_icon.setStyleSheet("""
                QLabel {
                    background: transparent;
                    border: none;
                    font-size: 54px;
                }
            """)

            # TITULO
            lbl_title = QtWidgets.QLabel(name)
            lbl_title.setAlignment(QtCore.Qt.AlignHCenter)
            lbl_title.setStyleSheet("""
                QLabel {
                    background: transparent;
                    border: none;
                    font-size: 18px;
                    font-weight: 800;
                }
            """)

            # DESCRIPCIÓN
            lbl_desc = QtWidgets.QLabel(desc)
            lbl_desc.setAlignment(QtCore.Qt.AlignHCenter)
            lbl_desc.setWordWrap(True)
            lbl_desc.setStyleSheet("""
                QLabel {
                    background: transparent;
                    border: none;
                    font-size: 14px;
                    color: rgba(0,0,0,200);
                }
            """)

            layout_card.addWidget(lbl_icon)
            layout_card.addWidget(lbl_title)
            layout_card.addWidget(lbl_desc)
            lbl_icon.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
            lbl_title.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)
            lbl_desc.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents, True)

            # Click handler
            def make_click(i):
                return lambda e: self.open_menu.emit(i)

            card.mousePressEvent = make_click(idx)

            self.cards.append(card)

            r = k // 3
            ccol = k % 3
            grid.addWidget(card, r, ccol)

        c.addLayout(grid)

        # centrar el container
        root.addWidget(container, 0, QtCore.Qt.AlignHCenter)

        root.addStretch(1)  # centra vertical

# =========================
# MENU 2 (dashboard)
# =========================
class Menu2Widget(QtWidgets.QWidget):
    goto_menu1 = QtCore.pyqtSignal()
    open_scope = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()

        self.last_data = None   # np.ndarray frames x CH (int16)
        self.last_seq  = None

        # === escalas reales (AJUSTAR) ===
        # si ADC_SCALE ya te da volts reales, dejá gain=1
        self.gain_v = np.ones(8, dtype=np.float32)  # multiplica a volts ADC -> volts reales
        self.gain_i = np.ones(8, dtype=np.float32)  # multiplica a volts ADC -> amps reales (si corrientes vienen como V de shunt/CT)

        # asignación de canales (AJUSTAR)
        self.V_CH = [0, 1, 2]   # CH1..3
        self.I_CH = [3, 4, 5]   # CH4..6

        # ventana de cálculo por defecto
        self.win_ms = 200.0
        self.avg_n = 5
        self._hist = []  # historial de dicts para promedio

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(24, 24, 24, 24)
        lay.setSpacing(14)

        # header
        top = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("MEDICIONES ELÉCTRICAS")
        title.setStyleSheet("font-size: 26px; font-weight: 900;")
        top.addWidget(title)
        top.addStretch(1)

        btn_scope = QtWidgets.QPushButton("Abrir Osciloscopio (formas de onda)")
        btn_scope.setMinimumHeight(40)
        btn_scope.setStyleSheet("font-size: 14px; font-weight: 800;")
        btn_scope.clicked.connect(self.open_scope.emit)
        top.addWidget(btn_scope)

        btn_back = QtWidgets.QPushButton("Volver")
        btn_back.setMinimumHeight(40)
        btn_back.clicked.connect(self.goto_menu1.emit)
        top.addWidget(btn_back)

        lay.addLayout(top)

        # controles de ventana
        g = QtWidgets.QGroupBox("Cálculo")
        gl = QtWidgets.QGridLayout(g)
        gl.addWidget(QtWidgets.QLabel("Ventana:"), 0, 0)
        self.cmb_win = QtWidgets.QComboBox()
        for ms in [20, 50, 100, 200, 500, 1000]:
            self.cmb_win.addItem(f"{ms} ms", userData=float(ms))
        self.cmb_win.setCurrentText("200 ms")
        self.cmb_win.currentIndexChanged.connect(self._on_win_changed)
        gl.addWidget(self.cmb_win, 0, 1)

        gl.addWidget(QtWidgets.QLabel("Promediar N:"), 0, 2)
        self.spin_avg = QtWidgets.QSpinBox()
        self.spin_avg.setRange(1, 50)
        self.spin_avg.setValue(self.avg_n)
        self.spin_avg.valueChanged.connect(self._on_avg_changed)
        gl.addWidget(self.spin_avg, 0, 3)

        self.lbl_status = QtWidgets.QLabel("Esperando datos…")
        self.lbl_status.setStyleSheet("font-family: Consolas, monospace;")
        gl.addWidget(self.lbl_status, 1, 0, 1, 4)

        lay.addWidget(g)

        # cards
        cards = QtWidgets.QGridLayout()
        cards.setHorizontalSpacing(12)
        cards.setVerticalSpacing(12)

        def card(title):
            w = QtWidgets.QFrame()
            w.setStyleSheet("""
                QFrame{
                    border-radius: 16px;
                    background-color: rgba(0,0,0,6);
                    border: 1px solid rgba(0,0,0,30);
                }
                QFrame QLabel{
                    background: transparent;
                    border: none;
                }
            """)
            v = QtWidgets.QVBoxLayout(w)
            v.setContentsMargins(14, 12, 14, 12)
            v.setSpacing(4)
            t = QtWidgets.QLabel(title)
            t.setStyleSheet("font-size: 13px; font-weight: 800; color: rgba(0,0,0,170);")
            val = QtWidgets.QLabel("--")
            val.setStyleSheet("font-size: 26px; font-weight: 900; font-family: Consolas, monospace;")
            v.addWidget(t)
            v.addWidget(val)
            v.addStretch(1)
            return w, val

        self.lbl_va, self.v_va = card("Vrms A")
        self.lbl_vb, self.v_vb = card("Vrms B")
        self.lbl_vc, self.v_vc = card("Vrms C")

        self.lbl_ia, self.v_ia = card("Irms A")
        self.lbl_ib, self.v_ib = card("Irms B")
        self.lbl_ic, self.v_ic = card("Irms C")

        self.lbl_p,  self.v_p  = card("P total (W)")
        self.lbl_s,  self.v_s  = card("S total (VA)")
        self.lbl_pf, self.v_pf = card("FP")

        cards.addWidget(self.lbl_va, 0, 0)
        cards.addWidget(self.lbl_vb, 0, 1)
        cards.addWidget(self.lbl_vc, 0, 2)
        cards.addWidget(self.lbl_ia, 1, 0)
        cards.addWidget(self.lbl_ib, 1, 1)
        cards.addWidget(self.lbl_ic, 1, 2)
        cards.addWidget(self.lbl_p,  0, 3)
        cards.addWidget(self.lbl_s,  1, 3)
        cards.addWidget(self.lbl_pf, 2, 3)

        lay.addLayout(cards)

        # tabla (opcional pero útil para “entregable”)
        self.table = QtWidgets.QTableWidget(3, 5)
        self.table.setHorizontalHeaderLabels(["Fase", "Vrms (V)", "Irms (A)", "P (W)", "S (VA)"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setMinimumHeight(150)
        lay.addWidget(self.table)

        lay.addStretch(1)

    def _on_win_changed(self):
        self.win_ms = float(self.cmb_win.currentData())
        self._hist.clear()

    def _on_avg_changed(self, v):
        self.avg_n = int(v)
        self._hist.clear()

    # === recibe data desde Reader global ===
    def on_packet(self, data_i16: np.ndarray, seq: int, fs_real: float):
        self.last_data = data_i16
        self.last_seq = seq

        if fs_real <= 0:
            return

        # recortar última ventana
        nwin = int(fs_real * (self.win_ms / 1000.0))
        nwin = max(64, min(nwin, data_i16.shape[0]))
        w = data_i16[-nwin:, :].astype(np.float32) * ADC_SCALE  # volts ADC

        # aplicar escalas reales
        vabc = w[:, self.V_CH] * self.gain_v[self.V_CH]
        iabc = w[:, self.I_CH] * self.gain_i[self.I_CH]

        # RMS por fase
        vrms = np.sqrt(np.mean(vabc * vabc, axis=0))
        irms = np.sqrt(np.mean(iabc * iabc, axis=0))

        # potencias instantáneas por fase
        p_phase = np.mean(vabc * iabc, axis=0)  # W si unidades reales
        s_phase = vrms * irms                    # VA

        P = float(np.sum(p_phase))
        S = float(np.sum(s_phase))
        PF = (P / S) if S > 1e-9 else 0.0

        cur = {
            "vrms": vrms, "irms": irms,
            "p_phase": p_phase, "s_phase": s_phase,
            "P": P, "S": S, "PF": PF
        }
        self._hist.append(cur)
        if len(self._hist) > self.avg_n:
            self._hist = self._hist[-self.avg_n:]

        # promediar N ventanas para que no “baile”
        vrms_m = np.mean([h["vrms"] for h in self._hist], axis=0)
        irms_m = np.mean([h["irms"] for h in self._hist], axis=0)
        pph_m  = np.mean([h["p_phase"] for h in self._hist], axis=0)
        sph_m  = np.mean([h["s_phase"] for h in self._hist], axis=0)
        Pm     = float(np.mean([h["P"] for h in self._hist]))
        Sm     = float(np.mean([h["S"] for h in self._hist]))
        PFm    = (Pm / Sm) if Sm > 1e-9 else 0.0

        # actualizar UI
        self.v_va.setText(f"{vrms_m[0]:.2f}")
        self.v_vb.setText(f"{vrms_m[1]:.2f}")
        self.v_vc.setText(f"{vrms_m[2]:.2f}")

        self.v_ia.setText(f"{irms_m[0]:.2f}")
        self.v_ib.setText(f"{irms_m[1]:.2f}")
        self.v_ic.setText(f"{irms_m[2]:.2f}")

        self.v_p.setText(f"{Pm:,.1f}")
        self.v_s.setText(f"{Sm:,.1f}")
        self.v_pf.setText(f"{PFm:.3f}")

        fases = ["A", "B", "C"]
        for k in range(3):
            self.table.setItem(k, 0, QtWidgets.QTableWidgetItem(fases[k]))
            self.table.setItem(k, 1, QtWidgets.QTableWidgetItem(f"{vrms_m[k]:.3f}"))
            self.table.setItem(k, 2, QtWidgets.QTableWidgetItem(f"{irms_m[k]:.3f}"))
            self.table.setItem(k, 3, QtWidgets.QTableWidgetItem(f"{pph_m[k]:.3f}"))
            self.table.setItem(k, 4, QtWidgets.QTableWidgetItem(f"{sph_m[k]:.3f}"))

        self.lbl_status.setText(f"Fs: {fs_real:,.0f} S/s | Ventana: {self.win_ms:.0f} ms | Avg: {len(self._hist)}/{self.avg_n} | seq: {seq}")

# =========================
# MENU 5 (dashboard)
# =========================
class Menu5Widget(QtWidgets.QWidget):
    goto_menu1 = QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(24, 24, 24, 24)
        lay.setSpacing(12)

        title = QtWidgets.QLabel("DIAGNÓSTICO")
        title.setStyleSheet("font-size: 26px; font-weight: 900;")
        lay.addWidget(title)

        self.lbl = QtWidgets.QLabel("Esperando datos…")
        self.lbl.setStyleSheet("font-family: Consolas, monospace; font-size: 14px;")
        self.lbl.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        lay.addWidget(self.lbl)

        lay.addStretch(1)

        btn_back = QtWidgets.QPushButton("Volver a MENÚ 1")
        btn_back.setMinimumHeight(44)
        btn_back.clicked.connect(self.goto_menu1.emit)
        lay.addWidget(btn_back)

        # stats
        self._bytes_acc = 0
        self._frames_acc = 0
        self._pkts_acc = 0
        self._t0 = time.time()

    def on_packet(self, data_i16: np.ndarray, seq: int, reader: "Reader"):
        # acumular para medir por segundo
        self._pkts_acc += 1
        self._frames_acc += int(data_i16.shape[0])
        self._bytes_acc += int(data_i16.shape[0]) * BYTES_PER_FRAME

        now = time.time()
        dt = now - self._t0
        if dt < 1.0:
            return

        fs = self._frames_acc / dt if dt > 0 else 0.0
        pps = self._pkts_acc / dt if dt > 0 else 0.0
        mbps = (self._bytes_acc * 8) / dt / 1e6 if dt > 0 else 0.0

        self._t0 = now
        self._bytes_acc = 0
        self._frames_acc = 0
        self._pkts_acc = 0

        self.lbl.setText(
            f"seq: {seq}\n"
            f"Fs: {fs:,.0f} S/s\n"
            f"pps: {pps:,.1f} packets/s\n"
            f"throughput: {mbps:.2f} Mbit/s\n"
            f"SYNC hits: {reader.sync_hits}\n"
            f"bad frames: {reader.bad_frames}\n"
        )



# =========================
# MAIN WINDOW = FSM
# =========================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Aplicación - Máquina de Estados")
        self.resize(900, 550)

        self.state = AppState.MENU1
        self.scope_win = None  # guardamos referencia para que no se destruya

        # Stack para estados (pantallas)
        self.stack = QtWidgets.QStackedWidget()
        self.setCentralWidget(self.stack)

        self.menu1 = Menu1Widget()
        self.menu2 = Menu2Widget()
        self.menu5 = Menu5Widget()

        self.stack.addWidget(self.menu1)  # index 0
        self.stack.addWidget(self.menu2)  # index 1
        self.stack.addWidget(self.menu5)  # index 2

        # Wiring de eventos
        self.menu1.open_menu.connect(self.on_menu1_choice)
        self.menu2.goto_menu1.connect(lambda: self.set_state(AppState.MENU1))
        self.menu2.open_scope.connect(self.on_open_scope)
        self.menu5.goto_menu1.connect(lambda: self.set_state(AppState.MENU1))

        self.set_state(AppState.MENU1)

        # ===== Reader global (UNO SOLO) =====
        self.reader = Reader(PORT, BAUD)
        self.reader.packet.connect(self._on_packet_global)
        self.reader.start()

        self._frames_total = 0
        self._t0 = time.time()
        self.fs_real = 0.0

        self.statusBar().showMessage("Listo.")

    def set_state(self, new_state: int):
        self.state = new_state

        if self.state == AppState.MENU1:
            self.stack.setCurrentWidget(self.menu1)
            self.statusBar().showMessage("Estado: MENÚ 1")

        elif self.state == AppState.MENU2:
            self.stack.setCurrentWidget(self.menu2)
            self.statusBar().showMessage("Estado: MENÚ 2")

        elif self.state == AppState.MENU5:
            self.stack.setCurrentWidget(self.menu5)
            self.statusBar().showMessage("Estado: DIAGNÓSTICO")

    def on_menu1_choice(self, idx: int):
        if idx == 0:
            self.set_state(AppState.MENU2)

        elif idx == 1:
            self.on_open_scope()

        elif idx == 4:  # 🩺 Diagnóstico
            self.set_state(AppState.MENU5)

        else:
            self.statusBar().showMessage(f"Menú {idx+1} aún no implementado")
    
    def on_open_scope(self):
        # Abrimos el Scope como ventana aparte.
        # Si ya existe, la traemos al frente.
        if self.scope_win is None:
            self.scope_win = Scope()   # <-- usa tu clase Scope real
            self.scope_win.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
            self.scope_win.destroyed.connect(self._on_scope_destroyed)
            self.scope_win.showMaximized()
        else:
            self.scope_win.showMaximized()
            self.scope_win.raise_()
            self.scope_win.activateWindow()

        self.statusBar().showMessage("Osciloscopio abierto.")

    def _on_scope_destroyed(self, *_):
        self.scope_win = None
        self.set_state(AppState.MENU2)
        self.statusBar().showMessage("Osciloscopio cerrado. Volviste a MENÚ 2")

    def _on_packet_global(self, data, seq):

        # cálculo Fs global
        self._frames_total += data.shape[0]
        dt = time.time() - self._t0
        if dt >= 1.0:
            self.fs_real = self._frames_total / dt if dt > 0 else 0.0
            self._frames_total = 0
            self._t0 = time.time()

        # alimentar menú 2
        self.menu2.on_packet(data, seq, self.fs_real)

        # alimentar menú 5 (diagnóstico)
        self.menu5.on_packet(data, seq, self.reader)

        # alimentar scope si está abierto
        if self.scope_win is not None:
            self.scope_win.on_packet(data, seq)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow()
    w.showMaximized()
    sys.exit(app.exec_())