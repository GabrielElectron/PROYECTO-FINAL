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
MAX_POINTS = 2000

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

class Scope8CH(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Osciloscopio v1 - 8 canales")
        self.resize(1300, 750)

        pg.setConfigOptions(useOpenGL=True)

        self.last_seq = None
        self.lost_packets = 0
        self.packets = 0

        # Buffer circular: MAX_POINTS muestras por 8 canales.
        self.buffer = np.zeros((MAX_POINTS, CH_PHY), dtype=np.float32)
        self.wr = 0
        self.filled = False

        # =========================

        self.main_widget = QtWidgets.QWidget()
        self.main_layout = QtWidgets.QHBoxLayout(self.main_widget)
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        self.main_layout.setSpacing(0)

        # Gráfica principal
        self.plot = pg.PlotWidget()
        self.plot.getAxis("left").setStyle(showValues=False)
        self.main_layout.addWidget(self.plot, stretch=1)
        self.running = True
        self.ch1_coupling = "DC"
        self.ch1_volts_per_div = 1.0
        self.vertical_divisions = 8

        # Panel derecho + botón lateral de abrir/cerrar
        self.right_container = self.create_right_container()
        self.main_layout.addWidget(self.right_container)

        self.setCentralWidget(self.main_widget)

        self.plot.setBackground("k")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setTitle("Osciloscopio 8CH - Señales centradas en 0 V")
        self.plot.setLabel("bottom", "Tiempo", units="ms")
        self.plot.setTitle("CH1 a CH8 - Señales en tiempo real")

        self.plot.setXRange(0, (MAX_POINTS / FS_HZ) * 1000)
        self.plot.setYRange(-4, 4, padding=0)
        self.plot.enableAutoRange(x=False, y=False)

        colors = [
            (255, 255, 0),    # CH1 amarillo
            (0, 255, 255),    # CH2 cyan
            (255, 0, 255),    # CH3 magenta
            (0, 255, 0),      # CH4 verde
            (255, 128, 0),    # CH5 naranja
            (0, 128, 255),    # CH6 azul
            (255, 0, 0),      # CH7 rojo
            (180, 180, 180),  # CH8 gris
        ]

        self.curves = []

        for ch in range(CH_PHY):
            curve = self.plot.plot(
                pen=pg.mkPen(colors[ch], width=1)
            )
            self.curves.append(curve)

        self.text = pg.TextItem(color=(255, 255, 0), anchor=(0, 0))
        self.plot.addItem(self.text)

        self.ch1_vdiv_text = pg.TextItem(color=(255, 255, 0), anchor=(0, 1))
        self.ch1_vdiv_text.setText("CH1  1 V/div")
        self.plot.addItem(self.ch1_vdiv_text)
        self.apply_ch1_vertical_scale()

        # Cada vez que cambia el zoom o el desplazamiento,
        # actualizamos la posición del texto de CH1.
        self.plot.getViewBox().sigRangeChanged.connect(self.update_ch1_text_position)
        self.update_ch1_text_position()

        print("Abriendo puerto:", PORT)
        print("Baudrate:", BAUD)

        self.reader = Reader(PORT, BAUD)
        self.reader.packet.connect(self.on_packet)
        self.reader.start()

    def create_side_panel(self):
        panel = QtWidgets.QFrame()
        panel.setFixedWidth(260)
        panel.setStyleSheet("""
            QFrame {
                background-color: #1e1e1e;
                border-left: 2px solid #505050;
            }
            QLabel#panel_title {
                color: #ffffff;
                font-size: 20px;
                font-weight: bold;
                padding: 8px;
            }
            QLabel#panel_subtitle {
                color: #bbbbbb;
                font-size: 13px;
                padding-left: 8px;
                padding-bottom: 12px;
            }
            QFrame#separator {
                background-color: #505050;
                min-height: 1px;
                max-height: 1px;
            }
                            
            QFrame#channel_box {
                background-color: #2a2a2a;
                border: 1px solid #606060;
                border-radius: 8px;
            }

            QLabel#ch1_title {
                color: rgb(255, 255, 0);
                font-size: 16px;
                font-weight: bold;
                padding: 2px;
            }

            QRadioButton {
                color: white;
                font-size: 14px;
                padding: 3px;
                spacing: 8px;
            }

            QRadioButton::indicator {
                width: 14px;
                height: 14px;
                border-radius: 7px;
                border: 2px solid #d0d0d0;
                background-color: #202020;
            }

            QRadioButton::indicator:checked {
                border: 2px solid rgb(255, 255, 0);
                background-color: rgb(255, 255, 0);
            }

            QRadioButton::indicator:unchecked {
                border: 2px solid #d0d0d0;
                background-color: #202020;
            }

            QRadioButton::indicator:hover {
                border: 2px solid #ffffff;
            }
            
            QLabel#small_label {
                color: #bbbbbb;
                font-size: 13px;
                padding-top: 8px;
            }

            QComboBox#vdiv_combo {
                background-color: #202020;
                color: white;
                border: 1px solid #707070;
                border-radius: 5px;
                padding: 5px;
                font-size: 14px;
            }

            QComboBox#vdiv_combo:hover {
                border: 1px solid rgb(255, 255, 0);
            }

            QComboBox#vdiv_combo QAbstractItemView {
                background-color: #202020;
                color: white;
                selection-background-color: #505050;
            }
                
        """)

        layout = QtWidgets.QVBoxLayout(panel)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        title = QtWidgets.QLabel("OSC 8CH")
        title.setObjectName("panel_title")
        layout.addWidget(title)

        subtitle = QtWidgets.QLabel("Panel auxiliar")
        subtitle.setObjectName("panel_subtitle")
        layout.addWidget(subtitle)

        separator = QtWidgets.QFrame()
        separator.setObjectName("separator")
        separator.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        layout.addWidget(separator)

        self.btn_run_stop = QtWidgets.QPushButton("RUN")
        self.btn_run_stop.setObjectName("run_stop_button")
        self.btn_run_stop.setFixedHeight(42)
        self.btn_run_stop.clicked.connect(self.toggle_run_stop)
        layout.addWidget(self.btn_run_stop)

        # =========================
        # Recuadro CH1
        # =========================

        self.ch1_box = QtWidgets.QFrame()
        self.ch1_box.setObjectName("channel_box")

        ch1_layout = QtWidgets.QVBoxLayout(self.ch1_box)
        ch1_layout.setContentsMargins(10, 8, 10, 8)
        ch1_layout.setSpacing(6)

        ch1_title = QtWidgets.QLabel("CH1")
        ch1_title.setObjectName("ch1_title")
        ch1_layout.addWidget(ch1_title)

        self.rb_ch1_ac = QtWidgets.QRadioButton("AC")
        self.rb_ch1_dc = QtWidgets.QRadioButton("DC")
        self.rb_ch1_off = QtWidgets.QRadioButton("OFF")

        self.rb_ch1_dc.setChecked(True)

        self.rb_ch1_ac.toggled.connect(self.update_ch1_coupling)
        self.rb_ch1_dc.toggled.connect(self.update_ch1_coupling)
        self.rb_ch1_off.toggled.connect(self.update_ch1_coupling)

        ch1_layout.addWidget(self.rb_ch1_ac)
        ch1_layout.addWidget(self.rb_ch1_dc)
        ch1_layout.addWidget(self.rb_ch1_off)

        layout.addWidget(self.ch1_box)

        vdiv_label = QtWidgets.QLabel("V/div")
        vdiv_label.setObjectName("small_label")
        ch1_layout.addWidget(vdiv_label)

        self.combo_ch1_vdiv = QtWidgets.QComboBox()
        self.combo_ch1_vdiv.setObjectName("vdiv_combo")

        self.combo_ch1_vdiv.addItems([
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
        ])

        self.combo_ch1_vdiv.setCurrentText("1 V/div")
        self.combo_ch1_vdiv.currentTextChanged.connect(self.update_ch1_volts_per_div)

        ch1_layout.addWidget(self.combo_ch1_vdiv)

        layout.addStretch()

        return panel

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
            
    def update_ch1_coupling(self):
        if self.rb_ch1_ac.isChecked():
            self.ch1_coupling = "AC"
        elif self.rb_ch1_dc.isChecked():
            self.ch1_coupling = "DC"
        elif self.rb_ch1_off.isChecked():
            self.ch1_coupling = "OFF"

        print(f"CH1 coupling: {self.ch1_coupling}")

    def update_ch1_volts_per_div(self, text):
        if "mV/div" in text:
            value_text = text.replace(" mV/div", "")
            self.ch1_volts_per_div = float(value_text) / 1000.0

        elif "V/div" in text:
            value_text = text.replace(" V/div", "")
            self.ch1_volts_per_div = float(value_text)

        self.apply_ch1_vertical_scale()

        self.ch1_vdiv_text.setText(
            f"CH1  {self.format_volts_per_div(self.ch1_volts_per_div)}"
        )

        self.update_ch1_text_position()

    def format_volts_per_div(self, value):
        if value < 1.0:
            return f"{value * 1000:g} mV/div"
        else:
            return f"{value:g} V/div"

    def apply_ch1_vertical_scale(self):
        self.ch1_vdiv_text.setText(
            f"CH1  {self.format_volts_per_div(self.ch1_volts_per_div)}"
        )

        self.update_ch1_text_position()

    def create_right_container(self):
        container = QtWidgets.QFrame()

        layout = QtWidgets.QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # Panel derecho principal
        self.side_panel = self.create_side_panel()
        self.update_run_stop_button_style()
        layout.addWidget(self.side_panel)

        # Barra angosta donde queda siempre visible el botón
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

        # Mediciones CH1 arriba a la izquierda
        self.text.setPos(
            x_min + margen_x,
            y_max - margen_y_top
        )

        # Escala CH1 abajo a la izquierda
        self.ch1_vdiv_text.setPos(
            x_min + margen_x,
            y_min + margen_y_bottom
        )

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
            if not self.running:
                return

        # Conversión de cuentas ADC a voltios.
        data_volts = data_i16.astype(np.float32) * ADC_SCALE

        n = data_volts.shape[0]

        if n >= MAX_POINTS:
            self.buffer[:, :] = data_volts[-MAX_POINTS:, :]
            self.wr = 0
            self.filled = True
        else:
            end = self.wr + n

            if end <= MAX_POINTS:
                self.buffer[self.wr:end, :] = data_volts
            else:
                first = MAX_POINTS - self.wr
                self.buffer[self.wr:, :] = data_volts[:first, :]
                self.buffer[:end % MAX_POINTS, :] = data_volts[first:, :]
                self.filled = True

            self.wr = end % MAX_POINTS

        if self.filled:
            y_all = np.vstack((self.buffer[self.wr:, :], self.buffer[:self.wr, :]))
        else:
            y_all = self.buffer[:self.wr, :]

        if y_all.shape[0] == 0:
            return

        # Eje X en milisegundos.
        x = (np.arange(y_all.shape[0]) / FS_HZ) * 1000.0

        # Graficamos los 8 canales todos centrados en Y = 0.
        for ch in range(CH_PHY):
            y = y_all[:, ch]

            if ch == 0:
                if self.ch1_coupling == "OFF":
                    self.curves[ch].setData([], [])
                    continue

                elif self.ch1_coupling == "AC":
                    y = y - np.mean(y)

                elif self.ch1_coupling == "DC":
                    pass

                # Escala vertical SOLO para CH1.
                # Convertimos volts a divisiones.
                y_display = y / self.ch1_volts_per_div

            else:
            # Los demás canales quedan sin esta escala por ahora.
                    # Usamos 1 V/div como referencia visual.
                y_display = y / 1.0

            self.curves[ch].setData(x, y_display)


        # =========================
        # Mediciones CH1 según acoplamiento
        # =========================

        y_ch1 = y_all[:, 0]

        if self.ch1_coupling == "OFF":
            self.text.setText(
                "CH1\n"
                "OFF"
            )
            return

        elif self.ch1_coupling == "AC":
            y_ch1 = y_ch1 - np.mean(y_ch1)

        med = calcular_mediciones(y_ch1, FS_HZ)

        self.text.setText(
            f"CH1\n"
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