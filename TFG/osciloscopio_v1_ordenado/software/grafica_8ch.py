from pydoc import text
import sys
import struct
import time

import serial
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from mediciones import calcular_mediciones


# =====================================================
# CONFIGURACIÓN
# =====================================================

PORT = "COM9"
BAUD = 2000000

SYNC = b"\xA5\x5A\xC3\x3C"
HDR_LEN = 10

CH_PHY = 8
BYTES_PER_FRAME = 16
MAX_FRAMES = 4096

V_RANGE = 10.0
ADC_SCALE = V_RANGE / 32768.0

FS_HZ = 32000
MAX_POINTS = FS_HZ * 2  # 2 segundos de historial para poder ver señales lentas

# Separación vertical entre canales para que no se encimen.
CHANNEL_OFFSET = 3.0

# Cantidad inicial de paquetes que no cuentan como pérdida.
WARMUP_PACKETS = 50


# =====================================================
# READER SERIAL EN HILO SEPARADO
# =====================================================

class Reader(QtCore.QThread):
    packet = QtCore.Signal(object, int)

    def __init__(self, port, baud):
        super().__init__()

        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(1.0)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.buf = bytearray()
        self.running = True

    def stop(self):
        self.running = False

        try:
            self.ser.cancel_read()
        except Exception:
            pass

    def run(self):
        while self.running:
            chunk = self.ser.read(65536)

            if chunk:
                self.buf += chunk

                # Evita que el buffer interno crezca demasiado.
                if len(self.buf) > 262144:
                    self.buf = self.buf[-4096:]

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
                    del self.buf[:1]
                    continue

                pkt_len = HDR_LEN + frames * BYTES_PER_FRAME

                if len(self.buf) < pkt_len:
                    break

                payload = bytes(self.buf[HDR_LEN:pkt_len])
                del self.buf[:pkt_len]

                data = np.frombuffer(payload, dtype=">i2")

                if data.size != frames * CH_PHY:
                    continue

                data = data.reshape(frames, CH_PHY)

                self.packet.emit(data, seq)

        try:
            self.ser.reset_input_buffer()
            self.ser.close()
        except Exception:
            pass


# =====================================================
# VENTANA PRINCIPAL
# =====================================================

VDIV_OPTIONS = [
    "10 mV/div",
    "20 mV/div",
    "50 mV/div",
    "100 mV/div",
    "200 mV/div",
    "500 mV/div",
    "1 V/div",
    "2 V/div",
    "5 V/div",
    "10 V/div",
]

TDIV_OPTIONS = [
    "50 us/div",
    "100 us/div",
    "200 us/div",
    "500 us/div",
    "1 ms/div",
    "2 ms/div",
    "5 ms/div",
    "10 ms/div",
    "20 ms/div",
    "50 ms/div",
    "100 ms/div",
    "200 ms/div",
]

CHANNEL_COLORS = [
    (255, 255, 0),    # CH1 amarillo
    (0, 255, 255),    # CH2 cyan
    (255, 0, 255),    # CH3 magenta
    (0, 255, 0),      # CH4 verde
    (255, 128, 0),    # CH5 naranja
    (0, 128, 255),    # CH6 azul
    (255, 0, 0),      # CH7 rojo
    (180, 180, 180),  # CH8 gris
]


class Scope8CH(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Osciloscopio v1 - 8 canales")
        self.resize(1300, 750)

        pg.setConfigOptions(useOpenGL=True)

        self.last_seq = None
        self.lost_packets = 0
        self.packets = 0
        self.running = True

        # Buffer circular: MAX_POINTS muestras por 8 canales.
        self.buffer = np.zeros((MAX_POINTS, CH_PHY), dtype=np.float32)
        self.wr = 0
        self.filled = False

        # Estado visual de cada canal.
        self.channel_coupling = ["DC"] * CH_PHY
        self.channel_volts_per_div = [1.0] * CH_PHY
        self.channel_names = [f"CH{i + 1}" for i in range(CH_PHY)]

        # Base de tiempo: 10 divisiones horizontales como un osciloscopio clásico.
        self.time_divisions = 10
        self.time_per_div = 0.005  # 5 ms/div inicial

        self.main_widget = QtWidgets.QWidget()
        self.main_layout = QtWidgets.QHBoxLayout(self.main_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # Gráfica principal.
        self.plot = pg.PlotWidget()
        self.plot.getAxis("left").setStyle(showValues=False)
        self.main_layout.addWidget(self.plot, stretch=1)

        # Panel derecho + botón lateral de abrir/cerrar.
        self.right_container = self.create_right_container()
        self.main_layout.addWidget(self.right_container)

        self.setCentralWidget(self.main_widget)

        self.plot.setBackground("k")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setLabel("bottom", "Tiempo", units="ms")
        self.plot.setTitle("CH1 a CH8 - Señales en tiempo real")

        self.apply_time_scale(update_plot=False)
        self.plot.setYRange(-4, 4, padding=0)
        self.plot.enableAutoRange(x=False, y=False)

        self.curves = []
        for ch in range(CH_PHY):
            curve = self.plot.plot(pen=pg.mkPen(CHANNEL_COLORS[ch], width=1))
            self.curves.append(curve)

        self.text = pg.TextItem(color=(255, 255, 0), anchor=(0, 0))
        self.plot.addItem(self.text)

        # Por ahora dejamos visible la escala de CH1 abajo a la izquierda.
        self.ch1_vdiv_text = pg.TextItem(color=(255, 255, 0), anchor=(0, 1))
        self.plot.addItem(self.ch1_vdiv_text)
        self.apply_ch1_vertical_scale()

        # Cada vez que cambia el zoom o el desplazamiento,
        # actualizamos la posición de los textos.
        self.plot.getViewBox().sigRangeChanged.connect(self.update_ch1_text_position)
        self.update_ch1_text_position()

        print("Abriendo puerto:", PORT)
        print("Baudrate:", BAUD)

        self.reader = Reader(PORT, BAUD)
        self.reader.packet.connect(self.on_packet)
        self.reader.start()

    def create_side_panel(self):
        panel = QtWidgets.QFrame()
        panel.setFixedWidth(430)
        panel.setStyleSheet("""
            QFrame {
                background-color: #1e1e1e;
                border-left: 2px solid #505050;
            }
            QFrame#separator {
                background-color: #505050;
                min-height: 1px;
                max-height: 1px;
            }
            QFrame#channel_box, QFrame#time_box {
                background-color: #2a2a2a;
                border: 1px solid #606060;
                border-radius: 8px;
            }
            QLabel#channel_title, QLabel#time_title {
                font-size: 15px;
                font-weight: bold;
                padding: 1px;
            }
            QLabel#time_title {
                color: #ffffff;
            }
            QRadioButton {
                color: white;
                font-size: 12px;
                padding: 1px;
                spacing: 4px;
            }
            QRadioButton::indicator {
                width: 12px;
                height: 12px;
                border-radius: 6px;
                border: 2px solid #d0d0d0;
                background-color: #202020;
            }
            QRadioButton::indicator:checked {
                border: 2px solid #ffffff;
                background-color: #ffffff;
            }
            QRadioButton::indicator:hover {
                border: 2px solid #ffffff;
            }
            QComboBox#vdiv_combo, QComboBox#tdiv_combo {
                background-color: #202020;
                color: white;
                border: 1px solid #707070;
                border-radius: 5px;
                padding: 3px;
                font-size: 12px;
            }
            QComboBox#vdiv_combo:hover, QComboBox#tdiv_combo:hover {
                border: 1px solid #ffffff;
            }
            QComboBox#vdiv_combo QAbstractItemView, QComboBox#tdiv_combo QAbstractItemView {
                background-color: #202020;
                color: white;
                selection-background-color: #505050;
            }
        """)

        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(6)

        separator = QtWidgets.QFrame()
        separator.setObjectName("separator")
        separator.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        layout.addWidget(separator)

        self.btn_run_stop = QtWidgets.QPushButton("RUN")
        self.btn_run_stop.setObjectName("run_stop_button")
        self.btn_run_stop.setFixedHeight(40)
        self.btn_run_stop.clicked.connect(self.toggle_run_stop)
        layout.addWidget(self.btn_run_stop)

        self.rb_ac = []
        self.rb_dc = []
        self.rb_off = []
        self.combo_vdiv = []

        channels_layout = QtWidgets.QHBoxLayout()
        channels_layout.setContentsMargins(0, 0, 0, 0)
        channels_layout.setSpacing(8)

        col1 = QtWidgets.QVBoxLayout()
        col1.setContentsMargins(0, 0, 0, 0)
        col1.setSpacing(6)

        col2 = QtWidgets.QVBoxLayout()
        col2.setContentsMargins(0, 0, 0, 0)
        col2.setSpacing(6)

        # Primera columna: CH1 a CH6.
        for ch in range(6):
            col1.addWidget(self.create_channel_box(ch))

        # Segunda columna: CH7 y CH8 arriba.
        for ch in range(6, CH_PHY):
            col2.addWidget(self.create_channel_box(ch))

        # Debajo de CH7 y CH8 queda la escala horizontal de tiempo.
        col2.addWidget(self.create_time_box())
        col2.addStretch()

        channels_layout.addLayout(col1, stretch=1)
        channels_layout.addLayout(col2, stretch=1)
        layout.addLayout(channels_layout)

        layout.addStretch()
        return panel

    def create_channel_box(self, ch):
        box = QtWidgets.QFrame()
        box.setObjectName("channel_box")

        box_layout = QtWidgets.QVBoxLayout(box)
        box_layout.setContentsMargins(8, 4, 8, 4)
        box_layout.setSpacing(2)

        r, g, b = CHANNEL_COLORS[ch]
        title = QtWidgets.QLabel(self.channel_names[ch])
        title.setObjectName("channel_title")
        title.setStyleSheet(f"color: rgb({r}, {g}, {b});")
        box_layout.addWidget(title)

        rb_ac = QtWidgets.QRadioButton("AC")
        rb_dc = QtWidgets.QRadioButton("DC")
        rb_off = QtWidgets.QRadioButton("OFF")
        rb_dc.setChecked(True)

        rb_ac.toggled.connect(lambda checked, canal=ch: self.update_channel_coupling(canal))
        rb_dc.toggled.connect(lambda checked, canal=ch: self.update_channel_coupling(canal))
        rb_off.toggled.connect(lambda checked, canal=ch: self.update_channel_coupling(canal))

        radio_layout = QtWidgets.QHBoxLayout()
        radio_layout.setContentsMargins(0, 0, 0, 0)
        radio_layout.setSpacing(8)
        radio_layout.addWidget(rb_ac)
        radio_layout.addWidget(rb_dc)
        radio_layout.addWidget(rb_off)
        box_layout.addLayout(radio_layout)

        combo = QtWidgets.QComboBox()
        combo.setObjectName("vdiv_combo")
        combo.addItems(VDIV_OPTIONS)
        combo.setCurrentText("1 V/div")
        combo.currentTextChanged.connect(lambda text, canal=ch: self.update_channel_volts_per_div(canal, text))
        box_layout.addWidget(combo)

        self.rb_ac.append(rb_ac)
        self.rb_dc.append(rb_dc)
        self.rb_off.append(rb_off)
        self.combo_vdiv.append(combo)

        return box

    def create_time_box(self):
        box = QtWidgets.QFrame()
        box.setObjectName("time_box")

        box_layout = QtWidgets.QVBoxLayout(box)
        box_layout.setContentsMargins(8, 6, 8, 6)
        box_layout.setSpacing(4)

        title = QtWidgets.QLabel("TIME")
        title.setObjectName("time_title")
        box_layout.addWidget(title)

        self.combo_tdiv = QtWidgets.QComboBox()
        self.combo_tdiv.setObjectName("tdiv_combo")
        self.combo_tdiv.addItems(TDIV_OPTIONS)
        self.combo_tdiv.setCurrentText("5 ms/div")
        self.combo_tdiv.currentTextChanged.connect(self.update_time_per_div)
        box_layout.addWidget(self.combo_tdiv)

        self.lbl_time_window = QtWidgets.QLabel()
        self.lbl_time_window.setStyleSheet("color: #cfcfcf; font-size: 12px;")
        box_layout.addWidget(self.lbl_time_window)
        self.update_time_window_label()

        return box

    def parse_time_per_div(self, text):
        if "us/div" in text:
            value_text = text.replace(" us/div", "")
            return float(value_text) / 1_000_000.0

        if "ms/div" in text:
            value_text = text.replace(" ms/div", "")
            return float(value_text) / 1000.0

        value_text = text.replace(" s/div", "")
        return float(value_text)

    def format_time_window(self, seconds):
        if seconds < 0.001:
            return f"{seconds * 1_000_000:g} us"
        if seconds < 1.0:
            return f"{seconds * 1000:g} ms"
        return f"{seconds:g} s"

    def update_time_window_label(self):
        total_time = self.time_per_div * self.time_divisions
        self.lbl_time_window.setText(
            f"Ventana: {self.format_time_window(total_time)}"
        )

    def update_time_per_div(self, text):
        self.time_per_div = self.parse_time_per_div(text)
        print(f"T/div: {text}")
        self.apply_time_scale(update_plot=True)

    def apply_time_scale(self, update_plot=True):
        total_time_ms = self.time_per_div * self.time_divisions * 1000.0
        self.plot.setXRange(0, total_time_ms, padding=0)

        if hasattr(self, "lbl_time_window"):
            self.update_time_window_label()

        if update_plot:
            self.refresh_plot()

    def toggle_side_panel(self):
        if self.side_panel.isVisible():
            self.side_panel.hide()
            self.btn_toggle_panel.setText("<<")
            self.btn_toggle_panel.setToolTip("Mostrar panel")
        else:
            self.side_panel.show()
            self.btn_toggle_panel.setText(">>")
            self.btn_toggle_panel.setToolTip("Ocultar panel")

    def toggle_run_stop(self):
        self.running = not self.running
        self.update_run_stop_button_style()

    def update_run_stop_button_style(self):
        if self.running:
            self.btn_run_stop.setText("RUN")
            self.btn_run_stop.setStyleSheet("""
            QPushButton#run_stop_button {
                background-color: #145a32;
                color: white;
                border: 1px solid #2ecc71;
                border-radius: 6px;
                font-size: 18px;
                font-weight: bold;
                padding: 6px;
            }
            QPushButton#run_stop_button:hover {
                background-color: #196f3d;
            }
            QPushButton#run_stop_button:pressed {
                background-color: #0b3d22;
            }
        """)
        else:
            self.btn_run_stop.setText("STOP")
            self.btn_run_stop.setStyleSheet("""
                QPushButton#run_stop_button {
                    background-color: #7b241c;
                    color: white;
                    border: 1px solid #e74c3c;
                    border-radius: 6px;
                    font-size: 18px;
                    font-weight: bold;
                    padding: 6px;
                }
                QPushButton#run_stop_button:hover {
                    background-color: #922b21;
                }
                QPushButton#run_stop_button:pressed {
                    background-color: #641e16;
                }
            """)

    def update_channel_coupling(self, ch):
        if self.rb_ac[ch].isChecked():
            self.channel_coupling[ch] = "AC"
        elif self.rb_dc[ch].isChecked():
            self.channel_coupling[ch] = "DC"
        elif self.rb_off[ch].isChecked():
            self.channel_coupling[ch] = "OFF"

        print(f"{self.channel_names[ch]} coupling: {self.channel_coupling[ch]}")
        self.refresh_plot()

    def update_channel_volts_per_div(self, ch, text):
        self.channel_volts_per_div[ch] = self.parse_volts_per_div(text)

        print(f"{self.channel_names[ch]} V/div: {self.channel_volts_per_div[ch]}")

        if ch == 0:
            self.apply_ch1_vertical_scale()

        self.refresh_plot()

    def parse_volts_per_div(self, text):
        if "mV/div" in text:
            value_text = text.replace(" mV/div", "")
            return float(value_text) / 1000.0

        value_text = text.replace(" V/div", "")
        return float(value_text)

    def format_volts_per_div(self, value):
        if value < 1.0:
            return f"{value * 1000:g} mV/div"
        return f"{value:g} V/div"

    def apply_ch1_vertical_scale(self):
        self.ch1_vdiv_text.setText(
            f"CH1  {self.format_volts_per_div(self.channel_volts_per_div[0])}"
        )
        self.update_ch1_text_position()

    def create_right_container(self):
        container = QtWidgets.QFrame()

        layout = QtWidgets.QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Panel derecho principal.
        self.side_panel = self.create_side_panel()
        self.update_run_stop_button_style()
        layout.addWidget(self.side_panel)

        # Barra angosta donde queda siempre visible el botón.
        self.panel_toggle_bar = QtWidgets.QFrame()
        self.panel_toggle_bar.setFixedWidth(34)
        self.panel_toggle_bar.setStyleSheet("""
            QFrame {
                background-color: #181818;
                border-left: 1px solid #404040;
            }
            QPushButton {
                background-color: #2c2c2c;
                color: white;
                border: 1px solid #606060;
                border-radius: 4px;
                font-size: 16px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #3a3a3a;
            }
            QPushButton:pressed {
                background-color: #505050;
            }
        """)

        toggle_layout = QtWidgets.QVBoxLayout(self.panel_toggle_bar)
        toggle_layout.setContentsMargins(4, 6, 4, 0)
        toggle_layout.setSpacing(0)

        self.btn_toggle_panel = QtWidgets.QPushButton(">>")
        self.btn_toggle_panel.setFixedSize(26, 26)
        self.btn_toggle_panel.setToolTip("Ocultar panel")
        self.btn_toggle_panel.clicked.connect(self.toggle_side_panel)

        toggle_layout.addWidget(self.btn_toggle_panel)
        toggle_layout.addStretch()

        layout.addWidget(self.panel_toggle_bar)
        return container

    def update_ch1_text_position(self):
        x_range, y_range = self.plot.getViewBox().viewRange()

        x_min, x_max = x_range
        y_min, y_max = y_range

        margen_x = (x_max - x_min) * 0.015
        margen_y_top = (y_max - y_min) * 0.04
        margen_y_bottom = (y_max - y_min) * 0.015

        # Mediciones CH1 arriba a la izquierda.
        self.text.setPos(x_min + margen_x, y_max - margen_y_top)

        # Escala CH1 abajo a la izquierda.
        self.ch1_vdiv_text.setPos(x_min + margen_x, y_min + margen_y_bottom)

    def on_packet(self, data_i16, seq):
        self.packets += 1

        # Ignora pérdidas falsas durante el arranque.
        if self.packets <= WARMUP_PACKETS:
            self.last_seq = seq
            self.lost_packets = 0

            if not self.running:
                return

        else:
            if self.last_seq is not None and seq != self.last_seq + 1:
                self.lost_packets += max(0, seq - self.last_seq - 1)

            self.last_seq = seq

            # Si está en STOP, congelamos la pantalla.
            # No guardamos muestras nuevas en el buffer.
            if not self.running:
                return

        # Conversión de cuentas ADC a voltios.
        data_volts = data_i16.astype(np.float32) * ADC_SCALE

        n = data_volts.shape[0]

        # Guardamos las muestras en el buffer circular.
        if n >= MAX_POINTS:
            self.buffer[:, :] = data_volts[-MAX_POINTS:, :]
            self.wr = 0
            self.filled = True

        else:
            end = self.wr + n

            if end <= MAX_POINTS:
                self.buffer[self.wr:end, :] = data_volts

                # Si justo llenamos el buffer, marcamos filled.
                if end == MAX_POINTS:
                    self.filled = True

            else:
                first = MAX_POINTS - self.wr
                self.buffer[self.wr:, :] = data_volts[:first, :]
                self.buffer[:end % MAX_POINTS, :] = data_volts[first:]
                self.filled = True

            self.wr = end % MAX_POINTS

        # Dibuja usando el contenido actual del buffer.
        self.refresh_plot()

    def get_y_all_from_buffer(self):
        if self.filled:
            y_all = np.vstack((self.buffer[self.wr:, :], self.buffer[:self.wr, :]))
        else:
            y_all = self.buffer[:self.wr, :]

        return y_all

    def refresh_plot(self):
        y_all = self.get_y_all_from_buffer()

        if y_all.shape[0] == 0:
            return

        # Mostramos solamente la ventana definida por T/div.
        visible_samples = int(self.time_per_div * self.time_divisions * FS_HZ)
        visible_samples = max(2, min(visible_samples, y_all.shape[0]))
        y_visible = y_all[-visible_samples:, :]

        # Eje X en milisegundos, empezando en 0 para la ventana visible.
        x = (np.arange(y_visible.shape[0]) / FS_HZ) * 1000.0

        # Graficamos los 8 canales todos centrados en Y = 0.
        for ch in range(CH_PHY):
            y = y_visible[:, ch]
            coupling = self.channel_coupling[ch]

            if coupling == "OFF":
                self.curves[ch].setData([], [])
                continue

            if coupling == "AC":
                y = y - np.mean(y)

            y_display = y / self.channel_volts_per_div[ch]
            self.curves[ch].setData(x, y_display)

        # =========================
        # Mediciones CH1
        # OFF solo oculta la curva, pero NO deja de medir.
        # =========================

        y_ch1_med = y_visible[:, 0].copy()

        if self.channel_coupling[0] == "AC":
            y_ch1_med = y_ch1_med - np.mean(y_ch1_med)

        med = calcular_mediciones(y_ch1_med, FS_HZ)

        if self.channel_coupling[0] == "OFF":
            estado_ch1 = "OFF visual"
        else:
            estado_ch1 = self.channel_coupling[0]

        self.text.setText(
            f"CH1  {estado_ch1}\n"
            f"Vmax: {med['vmax']:.3f} V\n"
            f"Vmin: {med['vmin']:.3f} V\n"
            f"Vpp: {med['vpp']:.3f} V\n"
            f"Vrms: {med['vrms_ac']:.3f} V\n"
            f"Freq: {med['frecuencia']:.2f} Hz\n"
        )

    def closeEvent(self, event):
        self.reader.stop()
        self.reader.wait(1000)
        event.accept()


def main():
    app = pg.mkQApp("Osciloscopio v1 - 8 canales")

    win = Scope8CH()
    win.showMaximized()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
