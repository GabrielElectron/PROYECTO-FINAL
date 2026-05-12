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

        self.plot = pg.PlotWidget()
        self.setCentralWidget(self.plot)

        self.plot.setBackground("k")
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.setTitle("Osciloscopio 8CH - Señales centradas en 0 V")
        self.plot.setLabel("bottom", "Muestras")
        self.plot.setTitle("CH1 a CH8 - Señales en tiempo real")

        self.plot.setXRange(0, MAX_POINTS)
        self.plot.setYRange(-10, 10)
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

        # Cada vez que cambia el zoom o el desplazamiento,
        # actualizamos la posición del texto de CH1.
        self.plot.getViewBox().sigRangeChanged.connect(self.update_ch1_text_position)
        self.update_ch1_text_position()

        print("Abriendo puerto:", PORT)
        print("Baudrate:", BAUD)

        self.reader = Reader(PORT, BAUD)
        self.reader.packet.connect(self.on_packet)
        self.reader.start()

    def update_ch1_text_position(self):
        x_range, y_range = self.plot.getViewBox().viewRange()

        x_min, x_max = x_range
        y_min, y_max = y_range

        margen_x = (x_max - x_min) * 0.015
        margen_y = (y_max - y_min) * 0.04

        self.text.setPos(
            x_min + margen_x,
            y_max - margen_y
        )

    def on_packet(self, data_i16, seq):
        self.packets += 1

        # Ignora pérdidas falsas durante el arranque.
        if self.packets <= WARMUP_PACKETS:
            self.last_seq = seq
            self.lost_packets = 0
        else:
            if self.last_seq is not None and seq != self.last_seq + 1:
                self.lost_packets += max(0, seq - self.last_seq - 1)

            self.last_seq = seq

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

        x = np.arange(y_all.shape[0])

        # Graficamos los 8 canales todos centrados en Y = 0.
        for ch in range(CH_PHY):
            y = y_all[:, ch]
            self.curves[ch].setData(x, y)

        # Mediciones de CH1 por ahora.
        ch1 = y_all[:, 0]
        med = calcular_mediciones(ch1, FS_HZ)

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